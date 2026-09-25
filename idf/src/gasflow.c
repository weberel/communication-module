/*
 * gasflow.c  --  see gasflow.h.
 */
#include "gasflow.h"

#include <math.h>
#include <string.h>

#define R_GAS    8.314462618        /* J/(mol K) */
#define T_ABS    273.15
#define Q40_US   1099511.627776     /* raw Q40 seconds -> us */
#define Q24      16777216.0
#define Q40      1099511627776.0
#define PI       3.14159265358979

/* ---- components -----------------------------------------------------------
 * Cp: NIST Shomate at 298 K with its local slope. mu: power law fitted to NIST
 * at 1 bar, 270..330 K. Good to ~0.5 % over 0..60 C. */
static const gf_comp_t CH4 = { "CH4", 0.016043, 35.69, 0.0590, 11.07e-6, 0.77 };
static const gf_comp_t CO2 = { "CO2", 0.044010, 37.12, 0.0400, 14.93e-6, 0.93 };
static const gf_comp_t AIR = { "air", 0.028965, 29.12, 0.0035, 18.46e-6, 0.76 };
static const gf_comp_t N2  = { "N2",  0.028014, 29.12, 0.0010, 17.81e-6, 0.72 };

static double cp_of(const gf_comp_t *g, double tk) { return g->cp_a + g->cp_b * (tk - 298.15); }
static double mu_of(const gf_comp_t *g, double tk) { return g->mu_ref * pow(tk / 298.15, g->mu_exp); }

/* ---- mixture ------------------------------------------------------------- */
static double mix_molar(const gf_cfg_t *c, double x)
{
    if (!c->b) return c->a->molar;
    return x * c->a->molar + (1.0 - x) * c->b->molar;
}

static double mix_c(const gf_cfg_t *c, double x, double t_c)
{
    double tk = t_c + T_ABS;
    double cp = c->b ? x * cp_of(c->a, tk) + (1.0 - x) * cp_of(c->b, tk)
                     : cp_of(c->a, tk);
    double g  = cp / (cp - R_GAS);
    return sqrt(g * R_GAS * tk / mix_molar(c, x));
}

/* Wilke's rule for a binary mixture. */
static double mix_mu(const gf_cfg_t *c, double x, double t_c)
{
    double tk = t_c + T_ABS;
    const gf_comp_t *a = c->a, *b = c->b;
    if (!b || x >= 1.0) return mu_of(a, tk);
    double m1 = mu_of(a, tk), m2 = mu_of(b, tk);
    if (x <= 0.0) return m2;
    double r12 = sqrt(m1 / m2) * pow(b->molar / a->molar, 0.25);
    double r21 = sqrt(m2 / m1) * pow(a->molar / b->molar, 0.25);
    double p12 = (1.0 + r12) * (1.0 + r12) / sqrt(8.0 * (1.0 + a->molar / b->molar));
    double p21 = (1.0 + r21) * (1.0 + r21) / sqrt(8.0 * (1.0 + b->molar / a->molar));
    double y = 1.0 - x;
    return x * m1 / (x + y * p12) + y * m2 / (y + x * p21);
}

/* c is monotonic in x for every pair here, so bisection works either way. */
static double x_from_c(const gf_cfg_t *c, double c_mps, double t_c, bool *clamped)
{
    *clamped = false;
    if (!c->b) return 1.0;
    double ca = mix_c(c, 1.0, t_c), cb = mix_c(c, 0.0, t_c);
    bool up = ca > cb;                                  /* c rises with x */
    if (up ? c_mps >= ca : c_mps <= ca) { *clamped = true; return 1.0; }
    if (up ? c_mps <= cb : c_mps >= cb) { *clamped = true; return 0.0; }
    double lo = 0.0, hi = 1.0;
    for (int i = 0; i < 40; i++) {
        double mid = 0.5 * (lo + hi);
        if ((mix_c(c, mid, t_c) < c_mps) == up) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
}

/* Single gas: the temperature at which it has this speed of sound. */
static double t_from_c(const gf_cfg_t *c, double c_mps)
{
    double lo = -60.0, hi = 150.0;
    for (int i = 0; i < 40; i++) {
        double mid = 0.5 * (lo + hi);
        if (mix_c(c, 1.0, mid) < c_mps) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
}

/* ---- profiles ------------------------------------------------------------ */
static const char *const NAMES[GF_GAS_COUNT] = { "biogas", "air", "air_co2", "n2" };

const char *gf_gas_name(gf_gas_t kind)
{
    return ((unsigned) kind < GF_GAS_COUNT) ? NAMES[kind] : "?";
}

bool gf_profile(gf_cfg_t *c, gf_gas_t kind)
{
    memset(c, 0, sizeof(*c));
    c->kind = kind;

    /* The CELL, common to every gas. L and t0 fitted 2026-09-25 to the ends of
     * a CH4/CO2 sweep on the 75 mm cell (pure CH4 162.6 us, pure CO2 267.1 us,
     * ~23 C). Bore 15.5 mm; K was fitted against Re computed with it, so a new
     * bore means a new K fit. */
    c->path_m   = 0.07220;
    c->t0_us    = 1.2;
    c->pipe_d_m = 0.0155;

    /* Zero on top of the module's compiled 3.78 ns: mean of the static zeros at
     * 90/50/10 % CH4 with the capture window at 140 us (+1.9/-3.8/-2.8 ns). The
     * 140 us window removed the amplitude term, so off_b = 0. Offsets measured
     * with the old 110 us window do not apply. */
    c->off_a_ps = -1600.0;
    c->off_b    = 0.0;

    /* 0.5 SL/min: zero-flow scatter is 0.03..0.36 SL/min per 1 Hz sample. A
     * real flow that stays below it is lost (up to 30 SL/h). */
    c->q_cutoff_lpm = 0.5;

    /* The MFCs' own standard conditions (Bronkhorst, 20 C / 1.01325 bar). */
    c->p_n_mbar = 1013.25;
    c->t_n_c    = 20.0;

    /* Window 140 us after excitation, gain 25, 6 trill cycles (12 pulses),
     * 170/240 kHz multitone: validated in biogas. For the other gases the
     * window still sits ahead of the echo (air ~210 us), but that is geometry,
     * not a measurement. */
    c->module.gap_adcsmp = 1700;
    c->module.gain       = 25;
    c->module.pulses     = 6;
    c->module.f1_hz      = 170000;
    c->module.f2_hz      = 240000;

    /* K, biogas, 2026-09-25: 11 points, 10/50/90 % CH4 x 2..14 SL/min,
     * Re 180..1730, 1.5 % rms. */
    c->k_a = 0.3025e7;
    c->k_b = 0.1293e7;
    c->k_c = -0.0706e7;

    c->amp_n         = 0;
    c->amp_ratio_max = 2.0;

    switch (kind) {
    case GF_GAS_BIOGAS: {
        /* Echo at gain 25 vs CH4 fraction, 2026-09-25 composition sweep at
         * 3 SL/min. Air at the same c gave ~1050, 3x the ~350 expected at
         * x ~ 0.61. Fails safe: dirt or a water film only weakens the echo. */
        static const double X[11] = { 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0 };
        static const double A[11] = { 443, 449, 432, 412, 397, 376, 353, 317, 278, 234, 208 };
        c->validated = true;
        c->a = &CH4;
        c->b = &CO2;
        c->amp_n = 11;
        for (int i = 0; i < 11; i++) { c->amp_x[i] = X[i]; c->amp_ref[i] = A[i]; }
        return true;
    }
    case GF_GAS_AIR:
    case GF_GAS_N2:
        /* Air, 2026-09-25 (dry, 23.9 C, 2..14 SL/min): the biogas K(Re) shape
         * held, its level was 5.2 +- 0.4 % low at every Re -> both terms x
         * 1.0525, composition term folded in at its midpoint. N2 inherits air's
         * level: physically near-identical, not measured. */
        c->validated = (kind == GF_GAS_AIR);
        c->a = (kind == GF_GAS_AIR) ? &AIR : &N2;
        c->b = NULL;
        c->k_a = (c->k_a + 0.5 * c->k_c) * 1.0525;
        c->k_b = c->k_b * 1.0525;
        c->k_c = 0.0;
        return true;
    case GF_GAS_AIR_CO2:
        c->validated = false;
        c->a = &AIR;
        c->b = &CO2;
        c->k_a += 0.5 * c->k_c;
        c->k_c  = 0.0;
        return true;
    default:
        return false;
    }
}

/* ---- derivation ---------------------------------------------------------- */
static double amp_expected(const gf_cfg_t *c, double x)
{
    int n = c->amp_n;
    if (n == 0) return 0.0;
    if (x <= c->amp_x[0])     return c->amp_ref[0];
    if (x >= c->amp_x[n - 1]) return c->amp_ref[n - 1];
    for (int i = 1; i < n; i++) {
        if (x <= c->amp_x[i]) {
            double f = (x - c->amp_x[i - 1]) / (c->amp_x[i] - c->amp_x[i - 1]);
            return c->amp_ref[i - 1] + f * (c->amp_ref[i] - c->amp_ref[i - 1]);
        }
    }
    return c->amp_ref[n - 1];
}

static double re_of(const gf_cfg_t *c, double q_lpm, double nu)
{
    return 4.0 * fabs(q_lpm / 60000.0) / (PI * c->pipe_d_m * nu);
}

/* Below Re 100 (under the fitted 180, and at zero flow) hold K at its Re-100
 * value instead of following the log toward zero. */
static double k_of(const gf_cfg_t *c, double re, double x)
{
    if (re < 100.0) re = 100.0;
    return c->k_a + c->k_b * log(re) + c->k_c * x;
}

/* actual -> standard conditions */
static double to_std(const gf_cfg_t *c, double p_mbar, double t_c)
{
    return (p_mbar / c->p_n_mbar) * ((c->t_n_c + T_ABS) / (t_c + T_ABS));
}

void gf_sample(const gf_cfg_t *c, const gf_raw_t *raw,
               double p_mbar, double t_c, gf_sample_t *o)
{
    memset(o, 0, sizeof(*o));
    if (raw->code != 122 || raw->tof_ups_q40 == 0 || raw->tof_dns_q40 == 0 ||
        p_mbar < 100.0 || t_c < -40.0 || t_c > 100.0)
        return;

    o->p_mbar = p_mbar;
    o->t_c    = t_c;
    double tu = raw->tof_ups_q40 / Q40_US;
    double td = raw->tof_dns_q40 / Q40_US;
    o->tof_us = 0.5 * (tu + td);
    o->c_mps  = c->path_m / ((o->tof_us - c->t0_us) * 1e-6);

    if (c->b) {
        double ca = mix_c(c, 1.0, t_c), cb = mix_c(c, 0.0, t_c);
        double lo = (ca < cb ? ca : cb) * (1.0 - GF_C_MARGIN);
        double hi = (ca > cb ? ca : cb) * (1.0 + GF_C_MARGIN);
        if (o->c_mps < lo || o->c_mps > hi) { o->tof_bad = true; return; }
        o->x_a = x_from_c(c, o->c_mps, t_c, &o->clamped);
    } else {
        if (fabs(o->c_mps / mix_c(c, 1.0, t_c) - 1.0) > GF_C_MARGIN) {
            o->tof_bad = true;
            return;
        }
        o->x_a = 1.0;
        /* Cross-check against the MS5837 only; properties use the measured T. */
        o->t_acoustic_c = t_from_c(c, o->c_mps);
    }

    /* Is it this gas at all? PGA ~0.85 dB per index step. */
    o->amp25 = 0.5 * ((double) raw->amp_ups + (double) raw->amp_dns)
             * pow(10.0, -0.85 * ((double) raw->gain - 25.0) / 20.0);
    if (c->amp_n) {
        o->amp_expected = amp_expected(c, o->x_a);
        o->not_gas = o->amp_expected > 0.0 &&
                     o->amp25 > c->amp_ratio_max * o->amp_expected;
    }

    o->molar = mix_molar(c, o->x_a);
    o->rho   = p_mbar * 100.0 * o->molar / (R_GAS * (t_c + T_ABS));
    o->mu    = mix_mu(c, o->x_a, t_c);
    o->nu    = o->mu / o->rho;

    unsigned amp = raw->amp_ups < raw->amp_dns ? raw->amp_ups : raw->amp_dns;
    o->offset_ps = c->off_a_ps + (amp > 0 ? c->off_b / amp : 0.0);

    /* X per second: the unit the totalizer integrates (dS1/2^24 per s). */
    double x_rate = ((double) raw->dtof_ps - o->offset_ps) * 1000.0
                  / ((tu * 1000.0) * (td * 1000.0));

    /* K needs Re, Re needs Q: K is only logarithmic in Re, so four
     * fixed-point passes land well under 0.1 %. */
    double k = k_of(c, 400.0, o->x_a);
    for (int i = 0; i < 4; i++) {
        o->re = re_of(c, k * x_rate * 60.0 / 1e6, o->nu);
        k = k_of(c, o->re, o->x_a);
    }
    o->k         = k;
    o->q_act_lpm = k * x_rate * 60.0 / 1e6;
    o->q_n_lpm   = o->q_act_lpm * to_std(c, p_mbar, t_c);
    o->q_a_n_lpm = o->not_gas ? 0.0 : o->q_n_lpm * o->x_a;
    if (c->q_cutoff_lpm > 0.0 && fabs(o->q_n_lpm) < c->q_cutoff_lpm) {
        o->cut = true;
        o->q_act_lpm = o->q_n_lpm = o->q_a_n_lpm = 0.0;
    }
    o->ok = true;
}

bool gf_window(const gf_cfg_t *c, int64_t ds1, int64_t ds0, uint32_t dt_ms,
               const gf_sample_t *rep, gf_window_t *w)
{
    memset(w, 0, sizeof(*w));
    if (!rep || !rep->ok || ds0 <= 0 || dt_ms == 0) return false;

    double x = (double) ds1 / Q24 - rep->offset_ps * (double) ds0 / Q40;
    double dt_s = dt_ms / 1000.0;

    /* K at the window's MEAN flow, not the representative sample's: the window
     * integrates whatever flowed, and K follows the flow. */
    double k = rep->k;
    for (int i = 0; i < 4; i++) {
        w->re = re_of(c, k * x / dt_s * 60.0 / 1e6, rep->nu);
        k = k_of(c, w->re, rep->x_a);
    }
    w->k        = k;
    w->v_act_ul = k * x;
    w->v_n_ul   = w->v_act_ul * to_std(c, rep->p_mbar, rep->t_c);
    w->v_a_n_ul = rep->not_gas ? 0.0 : w->v_n_ul * rep->x_a;
    w->q_n_lpm  = w->v_n_ul / 1e6 / (dt_ms / 60000.0);
    if (c->q_cutoff_lpm > 0.0 && fabs(w->q_n_lpm) < c->q_cutoff_lpm) {
        w->cut = true;
        w->v_act_ul = w->v_n_ul = w->v_a_n_ul = w->q_n_lpm = 0.0;
    }
    return true;
}
