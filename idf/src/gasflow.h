/*
 * gasflow.h  --  physical quantities from the ultrasonic module's RAW values.
 *
 * The module ships raw time-of-flight and raw integrand sums (S1/S0); nothing
 * on it is calibrated. This turns those, plus the MS5837 gas pressure and
 * temperature (it sits IN the gas line, config.h), into:
 *
 *     speed of sound -> composition (binary gas) or acoustic T (single gas)
 *                    -> M, rho, mu, nu
 *     dTOF           -> zero-corrected flow -> Re -> K(Re, x) -> Q actual
 *     Q actual, P, T -> Q standard, Q of component A; same for window volumes
 *
 * OWN IMPLEMENTATION, deliberately separate from uss-link's uss_derive.c: the
 * comm board keeps its own driver and its own maths. The PHYSICS and the
 * CONSTANTS are the ones fitted on the MFC rig (Calibration-data-collection/
 * calibration/USS.md, 2026-09-25), and must stay numerically identical to
 * uss_derive.c -- a host test comparing the two lives in tools/gasflow_check.c.
 * Refit one, refit both, and bump GF_DERIVE_VER.
 *
 * Pure C99 + <math.h>, no ESP-IDF headers, so it builds on the host too.
 *
 * Known gaps (see USS.md):
 *  - Ideal-gas c with T-dependent Cp; real-gas terms absorbed by the L/t0 fit.
 *  - Binary profile = exactly two components. Water vapour is NOT modelled:
 *    saturated biogas at 35 C reads ~3-4 % CH4 off.
 *  - The 75 mm cell runs laminar (Re 180..1730 at 2..14 SL/min);
 *    K = k_a + k_b ln(Re) + k_c x, 1.5 % rms in-sample.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Which constant set produced a derived number. Published with every derived
 * value so the server knows what to recompute. Bump on EVERY refit. */
#define GF_DERIVE_VER   2               /* 2: per-cell profiles (44 mm added) */

/* Gate on c, as a fraction beyond the pure-component limits (binary) or around
 * c(T) (single gas). A multitone lock one pattern period off moves c by 4-6 %,
 * a burst lock by 30 %+. */
#define GF_C_MARGIN     0.03

typedef enum {
    GF_GAS_BIOGAS = 0,      /* CH4 / CO2, x = CH4        validated 2026-09-25 */
    GF_GAS_AIR,             /* dry air, single component validated 2026-09-25 */
    GF_GAS_AIR_CO2,         /* air / CO2 blends, x = air NOT validated        */
    GF_GAS_N2,              /* nitrogen                  NOT validated        */
    GF_GAS_COUNT
} gf_gas_t;

/* The measuring cell. The module no longer knows it (one image for every cell,
 * 2026-09-25): the ESP32 pushes the cell's capture window and ToF gate at every
 * module boot, and the derivation needs the cell's path and K scale. */
typedef enum {
    GF_CELL_75 = 0,         /* 75 mm cell, 72.2 mm path  validated 2026-09-25 */
    GF_CELL_44,             /* 44 mm cell               NOT calibrated        */
    GF_CELL_COUNT
} gf_cell_t;

/* One component. Cp(T) = cp_a + cp_b (T - 298.15) J/(mol K);
 * mu(T) = mu_ref (T/298.15)^mu_exp Pa s. */
typedef struct {
    const char *name;
    double molar;           /* kg/mol */
    double cp_a, cp_b;
    double mu_ref, mu_exp;
} gf_comp_t;

/* Module settings that belong to a gas, written over I2C PARAM_SET. */
typedef struct {
    int32_t gap_adcsmp;     /* capture window start, ASQ counts (~5 per us) */
    int32_t gain;           /* PGA index                                    */
    int32_t pulses;         /* trill cycles (2 pulses each)                 */
    int32_t f1_hz, f2_hz;
    int32_t tofg_min_ns;    /* abs-ToF gate: the module rejects a capture     */
    int32_t tofg_max_ns;    /* outside it as 0xE1. Per CELL.                  */
} gf_module_t;

typedef struct {
    gf_gas_t kind;
    gf_cell_t cell;
    bool     validated;
    const gf_comp_t *a;     /* x is the fraction of this one   */
    const gf_comp_t *b;     /* NULL for a single-component gas */
    double path_m;          /* acoustic path L                              */
    double t0_us;           /* fixed delay inside the reported abs ToF      */
    double pipe_d_m;        /* bore of the measuring section                */
    double k_a, k_b, k_c;   /* K in uL per unit of X = dS1/2^24             */
    double off_a_ps, off_b; /* zero on top of the module's: a + b/amp_min   */
    double q_cutoff_lpm;    /* standard L/min, derived values only; 0 = off */
    double p_n_mbar, t_n_c; /* standard conditions (Bronkhorst: 20 C)       */
    gf_module_t module;
    /* Echo amplitude at gain 25 vs x_a, ascending x; 0 points = check off.
     * A sample more than amp_ratio_max x the expected echo is not this gas
     * (air in a biogas line reads ~61 % CH4 by c alone, but echoes ~3x). */
    uint8_t amp_n;
    double  amp_x[12], amp_ref[12];
    double  amp_ratio_max;
} gf_cfg_t;

/* Gas profile on a cell. False for an unknown gas or cell. */
bool        gf_profile(gf_cfg_t *c, gf_gas_t kind, gf_cell_t cell);
const char *gf_gas_name(gf_gas_t kind);
int         gf_cell_mm(gf_cell_t cell);       /* 75, 44; 0 if unknown */

/* The raw inputs, exactly as they sit in a LogRecord. */
typedef struct {
    uint8_t  code;          /* 122 = valid */
    uint32_t tof_ups_q40, tof_dns_q40;
    int32_t  dtof_ps;
    uint16_t amp_ups, amp_dns;
    uint8_t  gain;
} gf_raw_t;

typedef struct {
    bool   ok;              /* code 122, P and T sane, c plausible          */
    bool   tof_bad;         /* c impossible for this gas at this T          */
    bool   clamped;         /* c just past a pure component: x clamped      */
    bool   cut;             /* below the low-flow cutoff: flows set to 0    */
    bool   not_gas;         /* echo far too strong for this gas at this c;
                             * total flow still reported, comp A forced 0   */
    double p_mbar, t_c;
    double tof_us, c_mps, x_a;
    double t_acoustic_c;    /* single gas: T implied by c, else 0           */
    double amp25, amp_expected;
    double molar, rho, mu, nu;
    double offset_ps;
    double re, k;
    double q_act_lpm, q_n_lpm, q_a_n_lpm;
} gf_sample_t;

void gf_sample(const gf_cfg_t *c, const gf_raw_t *raw,
               double p_mbar, double t_c, gf_sample_t *o);

/* Volumes over a totalizer window, using a representative sample for
 * composition, P and T. K is taken at the window's MEAN flow. Zero if the mean
 * standard flow is below the cutoff. False if rep is not ok, ds0 <= 0 or
 * dt_ms == 0. */
typedef struct {
    double v_act_ul, v_n_ul, v_a_n_ul;
    double q_n_lpm;         /* window-mean standard flow, after the cutoff  */
    double re, k;
    bool   cut;
} gf_window_t;

bool gf_window(const gf_cfg_t *c, int64_t ds1, int64_t ds0, uint32_t dt_ms,
               const gf_sample_t *rep, gf_window_t *w);

#ifdef __cplusplus
}
#endif
