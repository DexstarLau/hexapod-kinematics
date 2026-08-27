/* hex_config.c — validation, derivation and layout introspection.
 *
 * Owner: algorithm workstream (D6.1, D151). C99. No allocation, no I/O,
 * no blocking. Every function here is pure.
 *
 * Single precision throughout, deliberately: hex_config_t and hex_derived_t
 * are float and the 50 Hz path runs on an ESP32 FPU that is single precision.
 * cosf/sinf/atan2f/sqrtf, never the double forms.
 */
#include "hex_config.h"
#include <math.h>

#define HEX_DEG2RAD  0.01745329251994329577f
#define HEX_RAD2DEG  57.29577951308232088f

hex_cfg_err_t hex_derive(const hex_config_t *cfg, hex_derived_t *out)
{
    float L2, L3, t3, c3, s3, R, psi, theta0;

    if (cfg == NULL || out == NULL) return HEX_CFG_E_MEMBER;

    L2 = cfg->femur_length_mm;
    L3 = cfg->tibia_length_mm;
    if (!(cfg->coxa_length_mm > 0.0f) || !(L2 > 0.0f) || !(L3 > 0.0f))
        return HEX_CFG_E_MEMBER;

    t3 = cfg->theta3_deg * HEX_DEG2RAD;
    c3 = cosf(t3);
    s3 = sinf(t3);

    /* R = sqrt(L2^2 + L3^2 + 2*L2*L3*cos t3). Degenerate only at L2 == L3
     * with t3 == 180 deg, where the two members fold onto each other. */
    R = sqrtf(L2 * L2 + L3 * L3 + 2.0f * L2 * L3 * c3);
    if (!(R > 0.0f)) return HEX_CFG_E_MEMBER;

    /* psi == 0 iff L3 == 0, or t3 == 0 (mod 360), or t3 == 180 (mod 360)
     * AND L3 < L2. The sign of the DENOMINATOR decides the 180 deg case;
     * testing sinf(t3) == 0 is not machine-checkable — sinf of radians(180)
     * is 1.2246e-16, not zero. atan2f already gets all of this right, so the
     * condition is documented rather than coded. */
    psi = atan2f(L3 * s3, L2 + L3 * c3) * HEX_RAD2DEG;

    theta0 = cfg->theta2_nom_deg + psi;

    out->rigid_len_mm   = R;
    out->psi_deg        = psi;
    out->theta_nom_deg  = theta0;
    out->a_eff_nom_mm   = R * cosf(theta0 * HEX_DEG2RAD);
    out->body_height_mm = R * sinf(theta0 * HEX_DEG2RAD);
    out->r_nom_mm       = cfg->coxa_length_mm + out->a_eff_nom_mm;

    return HEX_CFG_OK;
}

hex_cfg_err_t hex_config_validate(const hex_config_t *cfg)
{
    hex_derived_t d;
    hex_cfg_err_t e;
    float r_ext, half_stride, sweep_deg, swing_ms;

    if (cfg == NULL) return HEX_CFG_E_MEMBER;

    e = hex_derive(cfg, &d);
    if (e != HEX_CFG_OK) return e;

    if (!(cfg->duty_factor > 0.0f) || !(cfg->duty_factor < 1.0f))
        return HEX_CFG_E_DUTY;
    if (!(cfg->stride_mm > 0.0f))
        return HEX_CFG_E_STRIDE;
    if (!(cfg->swing_clearance_mm > 0.0f) ||
        !(cfg->swing_clearance_mm < d.body_height_mm))
        return HEX_CFG_E_CLEARANCE;
    if (!(cfg->dtheta_peak_deg_s > 0.0f) || !(cfg->frame_period_us > 0.0f))
        return HEX_CFG_E_RATE;
    if (!(cfg->swing_peak_factor >= 1.0f))
        return HEX_CFG_E_PROFILE;

    /* D240. Both positive; the grid is not required to be finer than the
     * accuracy, because it is not — 0.1350 against 0.2400. */
    if (!(cfg->command_step_deg > 0.0f) || !(cfg->joint_accuracy_deg > 0.0f))
        return HEX_CFG_E_RESOLUTION;

    /* D258. One pair for all six legs (D221), checked once, here. */
    if (!(cfg->theta3_min_deg < cfg->theta3_max_deg) ||
        !(cfg->theta3_deg >= cfg->theta3_min_deg) ||
        !(cfg->theta3_deg <= cfg->theta3_max_deg))
        return HEX_CFG_E_THETA3;

    /* The foot travels a STRAIGHT line of length stride_mm, swept by the coxa
     * alone about the coxa axis at radius r_nom. So the radial extreme is the
     * hypotenuse, not r_nom + stride/2:
     *
     *   r_ext      = hypot(r_nom, stride/2)
     *   half_sweep = atan((stride/2) / r_nom)
     *
     * This is the model D211 used: at r_nom = 118.94 and half-stride 30 it
     * returns 14.1559 deg, D211's coxa half-sweep, to 4 dp.
     *
     * a_eff at that extreme is r_ext - L1 and must stay strictly inside R. */
    half_stride = 0.5f * cfg->stride_mm;
    r_ext = sqrtf(d.r_nom_mm * d.r_nom_mm + half_stride * half_stride);
    if (!(r_ext - cfg->coxa_length_mm < d.rigid_len_mm))
        return HEX_CFG_E_REACH;

    /* Swing duration, from the same chain as D146/D220 and nothing new:
     *
     *   sweep_deg = 2*atan((stride/2)/r_nom)
     *   mean rate = dtheta_peak / swing_peak_factor      (D145 half-sine)
     *   swing_ms  = 1000 * sweep_deg * swing_peak_factor / dtheta_peak
     *
     * The stale ramp must OUTLAST one swing, else a hold ends mid-air. */
    sweep_deg = 2.0f * atan2f(half_stride, d.r_nom_mm) * HEX_RAD2DEG;
    swing_ms  = 1000.0f * sweep_deg * cfg->swing_peak_factor /
                cfg->dtheta_peak_deg_s;
    if (!(cfg->stale_ramp_ms > swing_ms))
        return HEX_CFG_E_RAMP;

    if (!(cfg->margin_factor >= 1.0f))
        return HEX_CFG_E_MARGIN;

    return HEX_CFG_OK;
}

/* ------------------------------------------------------------------ */
/* Layout introspection. Declaration order, one entry per member.       */
/* ------------------------------------------------------------------ */

#define HEX_FIELD(m) { #m, offsetof(hex_config_t, m), sizeof(((hex_config_t *)0)->m) }

static const struct { const char *name; size_t off; size_t size; }
hex_fields[] = {
    HEX_FIELD(coxa_length_mm),
    HEX_FIELD(femur_length_mm),
    HEX_FIELD(tibia_length_mm),
    HEX_FIELD(theta3_deg),
    HEX_FIELD(coxa_x_mm),
    HEX_FIELD(coxa_y_mm),
    HEX_FIELD(beta_mount_deg),
    HEX_FIELD(beta_neutral_deg),
    HEX_FIELD(theta2_nom_deg),
    HEX_FIELD(stride_mm),
    HEX_FIELD(swing_clearance_mm),
    HEX_FIELD(duty_factor),
    HEX_FIELD(swing_peak_factor),
    HEX_FIELD(dtheta_peak_deg_s),
    HEX_FIELD(frame_period_us),
    HEX_FIELD(stale_ramp_ms),
    HEX_FIELD(swing_eps_mm_s),
    HEX_FIELD(command_step_deg),
    HEX_FIELD(joint_accuracy_deg),
    HEX_FIELD(joint_min_deg),
    HEX_FIELD(joint_max_deg),
    HEX_FIELD(theta3_min_deg),
    HEX_FIELD(theta3_max_deg),
    HEX_FIELD(mass_kg),
    HEX_FIELD(tau_servo_kgcm),
    HEX_FIELD(margin_factor)
};

#define HEX_FIELD_N ((int)(sizeof hex_fields / sizeof hex_fields[0]))

size_t hex_config_sizeof(void)      { return sizeof(hex_config_t); }
int    hex_config_field_count(void) { return HEX_FIELD_N; }

const char *hex_config_field_name(int index)
{
    if (index < 0 || index >= HEX_FIELD_N) return NULL;
    return hex_fields[index].name;
}

size_t hex_config_field_offset(int index)
{
    if (index < 0 || index >= HEX_FIELD_N) return (size_t)-1;
    return hex_fields[index].off;
}

size_t hex_config_field_size(int index)
{
    if (index < 0 || index >= HEX_FIELD_N) return (size_t)-1;
    return hex_fields[index].size;
}
