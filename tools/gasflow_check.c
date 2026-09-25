/*
 * gasflow_check.c  --  host test: the comm board's gasflow.c must produce the
 * same numbers as uss-link's uss_derive.c (the reference the MFC fit was done
 * with). Sweeps every profile over composition, T, P, flow, amplitude and gain,
 * and compares every derived field plus the window volumes.
 *
 *   gcc -std=c99 -O2 -Wall -Wextra -I../../uss-link/include -I../idf/src \
 *       gasflow_check.c ../idf/src/gasflow.c ../../uss-link/src/uss_derive.c \
 *       -lm -o gasflow_check && ./gasflow_check
 *
 * Exit status 0 = identical within 1e-9 relative.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "uss_derive.h"
#include "gasflow.h"

/* uss_derive.c references the uss-link driver for the module preset; the
 * check never calls it. */
bool uss_param_set(uss_t *d, uint8_t id, int32_t value, int32_t *in_effect,
                   uint8_t *status)
{
    (void) d; (void) id; (void) value; (void) in_effect; (void) status;
    return false;
}

static double worst;
static long   n_cmp, n_fail;
/* How often each branch fired: a clean pass means nothing if a branch never ran. */
static long   n_ok, n_bad, n_notgas, n_cut, n_clamped;
static const char *worst_what = "";

static void cmp(const char *what, double a, double b)
{
    double scale = fmax(fmax(fabs(a), fabs(b)), 1e-12);
    double rel = fabs(a - b) / scale;
    if (fabs(a - b) < 1e-15) rel = 0.0;
    n_cmp++;
    if (rel > worst) { worst = rel; worst_what = what; }
    if (rel > 1e-9 && n_fail++ < 10)
        printf("MISMATCH %s: gasflow %.12g vs uss_derive %.12g\n", what, a, b);
}

static void flag(const char *what, bool a, bool b)
{
    n_cmp++;
    if (a != b && n_fail++ < 10)
        printf("MISMATCH %s: gasflow %d vs uss_derive %d\n", what, a, b);
}

int main(void)
{
    static const double XS[]    = { 0.0, 0.05, 0.3, 0.55, 0.61, 0.8, 1.0 };
    static const double TS[]    = { -5.0, 12.0, 23.0, 38.0 };
    static const double PS[]    = { 880.0, 965.0, 1013.25, 1060.0 };
    static const double DTOF[]  = { -40000.0, -1600.0, 0.0, 900.0, 3000.0, 25000.0, 120000.0 };
    static const double CSHIFT[] = { 1.0, 0.95, 1.06 };      /* in / out of the gate */
    static const int    GAINS[] = { 17, 25, 40 };
    static const int    AMPS[]  = { 150, 400, 1100 };
    static const uint32_t DTMS[] = { 60000, 300000, 1800000 };

    for (int g = 0; g < GF_GAS_COUNT; g++) {
        gf_cfg_t  mine;
        uss_derive_cfg_t ref;
        if (!gf_profile(&mine, (gf_gas_t) g) ||
            !uss_derive_profile(&ref, (uss_gas_kind_t) g)) {
            printf("profile %d missing\n", g);
            return 1;
        }
        if (strcmp(gf_gas_name((gf_gas_t) g), uss_gas_name((uss_gas_kind_t) g)))
            { printf("name mismatch %d\n", g); return 1; }
        cmp("module.gap", mine.module.gap_adcsmp, ref.module.gap_adcsmp);
        cmp("module.gain", mine.module.gain, ref.module.gain);
        cmp("module.pulses", mine.module.pulses, ref.module.pulses);
        cmp("module.f1", mine.module.f1_hz, ref.module.f1_hz);
        cmp("module.f2", mine.module.f2_hz, ref.module.f2_hz);
        flag("validated", mine.validated, ref.validated);

        for (unsigned ix = 0; ix < sizeof XS / sizeof *XS; ix++)
        for (unsigned it = 0; it < sizeof TS / sizeof *TS; it++)
        for (unsigned ip = 0; ip < sizeof PS / sizeof *PS; ip++)
        for (unsigned id = 0; id < sizeof DTOF / sizeof *DTOF; id++)
        for (unsigned is = 0; is < sizeof CSHIFT / sizeof *CSHIFT; is++)
        for (unsigned ig = 0; ig < sizeof GAINS / sizeof *GAINS; ig++)
        for (unsigned ia = 0; ia < sizeof AMPS / sizeof *AMPS; ia++) {
            double c = uss_gas_c(&ref, XS[ix], TS[it]) * CSHIFT[is];
            double tof_us = ref.path_m / c * 1e6 + ref.t0_us;
            double half = DTOF[id] * 1e-6 / 2.0;             /* ps -> us, split */

            uss_result_t r;
            memset(&r, 0, sizeof r);
            r.code        = 122;
            r.tof_ups_q40 = (uint32_t) llround((tof_us + half) * 1099511.627776);
            r.tof_dns_q40 = (uint32_t) llround((tof_us - half) * 1099511.627776);
            r.dtof_ps     = (int32_t) DTOF[id];
            r.amp_ups     = (uint16_t) AMPS[ia];
            r.amp_dns     = (uint16_t) (AMPS[ia] * 9 / 10);
            r.gain        = (uint8_t) GAINS[ig];

            gf_raw_t raw = { r.code, r.tof_ups_q40, r.tof_dns_q40, r.dtof_ps,
                             r.amp_ups, r.amp_dns, r.gain };
            gf_sample_t a;
            uss_derived_t b;
            gf_sample(&mine, &raw, PS[ip], TS[it], &a);
            uss_derive_sample(&ref, &r, PS[ip], TS[it], &b);

            flag("ok", a.ok, b.ok);
            n_ok += a.ok; n_bad += a.tof_bad; n_notgas += a.not_gas;
            n_cut += a.cut; n_clamped += a.clamped;
            flag("tof_bad", a.tof_bad, b.tof_implausible);
            flag("clamped", a.clamped, b.comp_clamped);
            flag("cut", a.cut, b.cut);
            flag("not_gas", a.not_gas, b.not_this_gas);
            cmp("tof_us", a.tof_us, b.tof_us);
            cmp("c_mps", a.c_mps, b.c_mps);
            cmp("x_a", a.x_a, b.x_a);
            cmp("t_acoustic", a.t_acoustic_c, b.t_acoustic_c);
            cmp("amp25", a.amp25, b.amp25);
            cmp("amp_expected", a.amp_expected, b.amp_expected);
            cmp("molar", a.molar, b.molar_kg);
            cmp("rho", a.rho, b.rho);
            cmp("mu", a.mu, b.mu);
            cmp("nu", a.nu, b.nu);
            cmp("offset_ps", a.offset_ps, b.offset_ps);
            cmp("re", a.re, b.re);
            cmp("k", a.k, b.k);
            cmp("q_act", a.q_act_lpm, b.q_act_lpm);
            cmp("q_n", a.q_n_lpm, b.q_n_lpm);
            cmp("q_a_n", a.q_a_n_lpm, b.q_a_n_lpm);

            /* Window: S1/S0 as the module would integrate this flow for dt. */
            for (unsigned iw = 0; iw < sizeof DTMS / sizeof *DTMS; iw++) {
                double tu_ns = r.tof_ups_q40 / 1099511.627776 * 1000.0;
                double td_ns = r.tof_dns_q40 / 1099511.627776 * 1000.0;
                int64_t ds1 = (int64_t) (DTMS[iw] * (double) r.dtof_ps
                                         / (tu_ns * td_ns) * 16777216.0);
                int64_t ds0 = (int64_t) (DTMS[iw] / (tu_ns * td_ns) * 1099511627776.0);
                gf_window_t w;
                double va = 0, vn = 0, van = 0;
                bool wa = gf_window(&mine, ds1, ds0, DTMS[iw], &a, &w);
                bool wb = uss_derive_window(&ref, ds1, ds0, DTMS[iw], &b, &va, &vn, &van);
                flag("win_ok", wa, wb);
                cmp("v_act", w.v_act_ul, va);
                cmp("v_n", w.v_n_ul, vn);
                cmp("v_a_n", w.v_a_n_ul, van);
            }
        }
    }

    printf("samples: %ld ok, %ld tof_bad, %ld not_gas, %ld cut, %ld clamped\n",
           n_ok, n_bad, n_notgas, n_cut, n_clamped);
    printf("%ld comparisons, %ld mismatches, worst relative diff %.3g (%s)\n",
           n_cmp, n_fail, worst, worst_what);
    return n_fail ? 1 : 0;
}
