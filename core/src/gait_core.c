/* gait_core.c — tripod gait engine implementation, v1.
 *
 * This workstream, 2026-09-10, under D380 (HANDOFF_63). See gait_core.h for
 * the signature provenance note: gait_set_stale is D144's ratified form from
 * HANDOFF_30-AI section 2.1, NOT the stale form still printed in the register.
 *
 * S-a through S-g are implemented explicitly and each is named at its site.
 *
 * Units: mm, mm/s, deg, deg/s, ms. All angles in and out are KINEMATIC SPACE,
 * not vendor command angles (D262) — same convention as ik_core.h and
 * hex_config.h.
 *
 * Frame discipline: every foot target is computed in BODY frame, where the
 * velocity command (vx, vy, omega) already lives, then converted to LEG frame
 * with ik_body_to_leg immediately before ik_solve_leg. Nothing here assumes a
 * leg's local x axis is the direction of travel — that assumption silently
 * ignores beta_mount per leg. It was written, caught and removed before this
 * revision shipped.
 */
#include "gait_core.h"
#include "ik_core.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Tripod group assignment. HEX_R1=0 HEX_R2=1 HEX_R3=2 HEX_L1=3 HEX_L2=4
 * HEX_L3=5 (hex_config.h). Two groups of three, alternating stance/swing,
 * chosen so no two adjacent legs share a group — the shape every stability
 * margin decision in the register (tripod share, D208) assumes.
 *
 *   Group A: R1, R3, L2
 *   Group B: R2, L1, L3   (phase offset 0.5)
 *
 * THIS WORKSTREAM'S DESIGN CHOICE under D107 — no decision fixes it.
 */
static int leg_in_group_a(int leg)
{
    return leg == HEX_R1 || leg == HEX_R3 || leg == HEX_L2;
}

/* --- module state. Single robot per process (D6/D6.1 one-MCU deployment).
 * Static storage rather than a caller-owned struct: no allocation either way,
 * and the frozen signature carries no handle for a caller to own. */
#define MODE_HELD      0
#define MODE_MOTION    1
#define MODE_FINISHING 2

static const hex_config_t *g_cfg = NULL;
static hex_derived_t g_derived;

static float g_phase = 0.0f;            /* 0..1, group A's phase */
static int   g_mode  = MODE_HELD;
static int   g_finish_group_a = 0;      /* which group was swinging at finish entry */

static int   g_stale = 0;               /* S-a: LEVEL, not edge */
static float g_stale_elapsed_ms = 0.0f;

static float g_hold[12];                /* S-d: the held stance pose */
static int   g_have_hold = 0;

static float g_ux[HEX_LEGS], g_uy[HEX_LEGS];  /* last commanded direction per leg */
static float g_cycle_ms = 0.0f;               /* last commanded cycle time */
static float g_finish_cycle_ms = 0.0f;        /* joint-rate-limited, see S-c note */

void gait_init(const hex_config_t *cfg)
{
    g_cfg = cfg;
    hex_derive(cfg, &g_derived);
    g_phase = 0.0f;
    g_mode = MODE_HELD;
    g_finish_group_a = 0;
    g_stale = 0;                 /* S-g: init clears the flag to FRESH */
    g_stale_elapsed_ms = 0.0f;
    g_have_hold = 0;
    g_cycle_ms = 0.0f;
    g_finish_cycle_ms = 0.0f;
    for (int i = 0; i < HEX_LEGS; i++) { g_ux[i] = 0.0f; g_uy[i] = 0.0f; }
    for (int i = 0; i < 12; i++) g_hold[i] = 0.0f;
}

/* S-a: level-triggered, idempotent, O(1), no floating-point arithmetic.
 * Returns the previous state so the caller can log edges. */
int gait_set_stale(int stale)
{
    int previous = g_stale;
    int now = (stale != 0) ? 1 : 0;
    if (now != previous) g_stale_elapsed_ms = 0.0f;  /* assignment, not arithmetic */
    g_stale = now;
    return previous;
}

/* Integral of a half-sine velocity profile, normalised to reach 1.0 at t01=1.
 * peak/mean = pi/2 (D145) is a property of this shape, not a factor applied. */
static float half_sine_fraction(float t01)
{
    return 0.5f * (1.0f - cosf((float)M_PI * t01));
}

/* Write one leg's angles from a stride fraction along its own direction and a
 * lift. Returns an ik_status_t; the caller decides what is fatal. */
static ik_status_t write_leg(int leg, float frac, float z_lift, float out[12])
{
    float dx = g_ux[leg] * frac * g_cfg->stride_mm;
    float dy = g_uy[leg] * frac * g_cfg->stride_mm;

    /* Neutral foot, LEG -> BODY, add the body-frame stride displacement, then
     * back to LEG for the solver. Both frames share z; only x/y rotate by
     * beta_mount (D23). beta_neutral is a command offset and is NOT applied
     * here — folding the two together is the D23 error. */
    float bx0, by0, bz0, bx, by, bz, lx, ly, lz, t1, t2;
    ik_leg_to_body(g_cfg, leg, g_derived.a_eff_nom_mm, 0.0f,
                   -g_derived.body_height_mm, &bx0, &by0, &bz0);
    bx = bx0 + dx;
    by = by0 + dy;
    bz = bz0 + z_lift;
    ik_body_to_leg(g_cfg, leg, bx, by, bz, &lx, &ly, &lz);

    ik_status_t st = ik_solve_leg(g_cfg, &g_derived, lx, ly, lz,
                                  IK_PROJ_RADIAL, &t1, &t2);
    if (st == IK_E_UNREACHABLE_FAR || st == IK_E_UNREACHABLE_NEAR) return st;

    out[HEX_COXA(leg)]  = t1;
    out[HEX_FEMUR(leg)] = t2;
    return st;
}

/* Compute all twelve angles at the current phase. force_all_stance drives
 * every leg on its stance arc regardless of phase — used at the moment the
 * finishing group plants, so the frozen pose has all six feet down. */
static int write_all(int force_all_stance, float out[12])
{
    for (int leg = 0; leg < HEX_LEGS; leg++) {
        float leg_phase = leg_in_group_a(leg) ? g_phase : fmodf(g_phase + 0.5f, 1.0f);
        int in_swing = !force_all_stance && (leg_phase >= g_cfg->duty_factor);

        float frac, z_lift;
        if (in_swing) {
            float local01 = (leg_phase - g_cfg->duty_factor)
                          / (1.0f - g_cfg->duty_factor);
            frac   = -0.5f + half_sine_fraction(local01);
            z_lift = g_cfg->swing_clearance_mm * sinf((float)M_PI * local01);
        } else {
            float local01 = leg_phase / g_cfg->duty_factor;
            if (local01 > 1.0f) local01 = 1.0f;    /* boundary, force_all_stance */
            frac   = 0.5f - local01;
            z_lift = 0.0f;
        }
        ik_status_t st = write_leg(leg, frac, z_lift, out);
        if (st == IK_E_UNREACHABLE_FAR || st == IK_E_UNREACHABLE_NEAR) return -2;
    }
    return 0;
}

static void store_hold(const float out[12])
{
    for (int i = 0; i < 12; i++) g_hold[i] = out[i];
    g_have_hold = 1;
}

/* Neutral stance, used only before any hold exists. */
static int write_neutral(float out[12])
{
    for (int leg = 0; leg < HEX_LEGS; leg++) {
        float t1, t2;
        ik_status_t st = ik_solve_leg(g_cfg, &g_derived,
                                      g_derived.a_eff_nom_mm, 0.0f,
                                      -g_derived.body_height_mm,
                                      IK_PROJ_RADIAL, &t1, &t2);
        if (st == IK_E_UNREACHABLE_FAR || st == IK_E_UNREACHABLE_NEAR) return -2;
        out[HEX_COXA(leg)]  = t1;
        out[HEX_FEMUR(leg)] = t2;
    }
    return 0;
}

int gait_step(float vx, float vy, float omega, float dt_ms, float out_angles[12])
{
    if (g_cfg == NULL) return -1;

    /* S-b: the ramp applies to COMMANDED BODY VELOCITY, not to a trajectory
     * in progress. vx/vy/omega decay to zero over stale_ramp_ms. Freezing the
     * pose on the stale edge would leave mid-swing legs in the air — a fall
     * produced by the safety path. */
    float ramp = 1.0f;
    if (g_stale) {
        g_stale_elapsed_ms += dt_ms;
        if (g_cfg->stale_ramp_ms > 0.0f) {
            ramp = 1.0f - g_stale_elapsed_ms / g_cfg->stale_ramp_ms;
            if (ramp < 0.0f) ramp = 0.0f;
        } else {
            ramp = 0.0f;
        }
    }

    float vx_b = vx * ramp;
    float vy_b = vy * ramp;
    float omega_rad_s = omega * ramp * (float)M_PI / 180.0f;

    /* Per-leg body-frame velocity field: v_leg = v_body + omega x r_leg, with
     * r_leg the leg's own mount point. Each leg gets its own speed and
     * direction even though all six share one phase clock. */
    float speed_leg[HEX_LEGS];
    float peak_speed = 0.0f;
    for (int leg = 0; leg < HEX_LEGS; leg++) {
        float vlx = vx_b - omega_rad_s * g_cfg->coxa_y_mm[leg];
        float vly = vy_b + omega_rad_s * g_cfg->coxa_x_mm[leg];
        speed_leg[leg] = sqrtf(vlx * vlx + vly * vly);
        if (speed_leg[leg] > 1e-6f) {
            g_ux[leg] = vlx / speed_leg[leg];
            g_uy[leg] = vly / speed_leg[leg];
        }
        if (speed_leg[leg] > peak_speed) peak_speed = speed_leg[leg];
    }

    /* --- S-c: no NEW swing starts once commanded speed is below the epsilon.
     * A leg already in swing completes it and plants, so the state this hands
     * to the hardware-owned safe crouch always has all six feet down. */
    if (peak_speed > g_cfg->swing_eps_mm_s) {
        float swing_s = g_cfg->stride_mm / peak_speed;
        float coxa_sweep_deg = (g_cfg->stride_mm / g_derived.r_nom_mm)
                             * (180.0f / (float)M_PI);
        float min_swing_s = (g_cfg->swing_peak_factor * coxa_sweep_deg)
                          / g_cfg->dtheta_peak_deg_s;
        if (swing_s < min_swing_s) swing_s = min_swing_s;

        g_cycle_ms = (swing_s / (1.0f - g_cfg->duty_factor)) * 1000.0f;

        /* The FASTEST legal swing, set by the peak joint rate alone and
         * independent of commanded speed. Used to complete an interrupted
         * swing (S-c) — see the note at MODE_FINISHING below. */
        g_finish_cycle_ms = (min_swing_s / (1.0f - g_cfg->duty_factor)) * 1000.0f;

        g_phase += dt_ms / g_cycle_ms;
        g_phase -= floorf(g_phase);
        g_mode = MODE_MOTION;

        int rc = write_all(0, out_angles);
        if (rc != 0) return rc;
        store_hold(out_angles);
        return 0;
    }

    if (g_mode == MODE_MOTION) {
        /* Entering the finish. Record which group is airborne right now; that
         * group, and only that group, is allowed to complete. */
        g_finish_group_a = (g_phase >= g_cfg->duty_factor) ? 1 : 0;
        g_mode = MODE_FINISHING;
    }

    if (g_mode == MODE_FINISHING) {
        float before = g_finish_group_a
                     ? g_phase
                     : fmodf(g_phase + 0.5f, 1.0f);
        /* S-c completes the swing at the JOINT-RATE-LIMITED cycle, not at the
         * last commanded one. The commanded cycle at the moment the ramp
         * crosses the epsilon is enormous — stride / swing_eps, which at the
         * shipped constants is a 24 s cycle, so a foot would hang for twelve
         * seconds during a comms fault. S-c's stated purpose is to "guarantee
         * termination with all six legs planted", which a twelve-second plant
         * does not serve. The joint-rate-limited cycle is the fastest the
         * servo can legally execute and bounds the plant at half of it.
         *
         * THIS BOUND IS THIS WORKSTREAM'S READING OF S-c, not S-c's own text.
         * HANDOFF_30-AI section 2.1 does not say at what rate an interrupted
         * swing completes. Disclosed rather than assumed. */
        if (g_cycle_ms > 0.0f) {
            float fc = (g_finish_cycle_ms > 0.0f) ? g_finish_cycle_ms : g_cycle_ms;
            g_phase += dt_ms / fc;
            g_phase -= floorf(g_phase);
        }
        float after = g_finish_group_a
                    ? g_phase
                    : fmodf(g_phase + 0.5f, 1.0f);

        /* The finishing group plants when its own phase wraps past 1.0. */
        int planted = (before >= g_cfg->duty_factor) && (after < before);
        int rc = write_all(planted, out_angles);
        if (rc != 0) return rc;
        store_hold(out_angles);
        if (planted) g_mode = MODE_HELD;
        return 0;
    }

    /* --- S-d: held stance pose, twelve valid floats, every frame,
     * indefinitely. Never a sentinel, never a NaN, never the last output by
     * omission. S-e: no crouch, no clamp, no output disable — those are
     * hardware-owned and stay outside this file. S-f: att_* is never called. */
    if (g_have_hold) {
        for (int i = 0; i < 12; i++) out_angles[i] = g_hold[i];
        return 0;
    }
    return write_neutral(out_angles);
}
