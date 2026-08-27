/* hex_config.h — configuration surface for the hexapod cores.
 *
 * Owner: algorithm workstream (D6.1, D151). Not editable by any other role.
 * C99. No allocation, no I/O, no blocking.
 *
 * THE CORES SHIP NO DEFAULT CONFIGURATION. There is deliberately no
 * hex_config_default(). Every field is supplied by the caller at init.
 * This is the strongest available form of the D143 item 7 guard: a core with
 * no defaults cannot silently run on a stale constant, because it cannot run
 * at all without being told the constants.
 *
 * Constants reach this struct from config/hexapod.json via the caller. The
 * cores never read the file (D6.1 forbids I/O) and never contain a copy of
 * any value in it.
 *
 * Revision 2026-08-27: D240 adds joint_accuracy_deg and HEX_CFG_E_RESOLUTION.
 * D258 replaces the scalar joint limits with per-joint arrays, adds the tibia
 * envelope pair, and appends HEX_CFG_E_THETA3. D261 fixes margin_factor.
 */
#ifndef HEX_CONFIG_H
#define HEX_CONFIG_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HEX_LEGS   6
#define HEX_JOINTS 12   /* coxa + femur per leg. The tibia is NOT commanded (D199). */

/* Frozen leg order: R1 R2 R3 L1 L2 L3. */
enum { HEX_R1 = 0, HEX_R2, HEX_R3, HEX_L1, HEX_L2, HEX_L3 };

/* Joint index layout for every [HEX_JOINTS] array and for gait_core's out[12].
 *
 * INTERLEAVED, two joints per leg, coxa first:
 *
 *   index  0  1   2  3   4  5   6  7   8  9  10 11
 *   leg    R1 R1  R2 R2  R3 R3  L1 L1  L2 L2 L3 L3
 *   joint  c  f   c  f   c  f   c  f   c  f  c  f
 *
 * "coxa precedes femur" is a statement about the pair, not about the array.
 * Use the macros; do not write the arithmetic out by hand anywhere.
 */
#define HEX_COXA(leg)   (2 * (leg))
#define HEX_FEMUR(leg)  (2 * (leg) + 1)

typedef struct {
    /* --- members. D160, D170, D199 --- */
    float coxa_length_mm;              /* L1 */
    float femur_length_mm;             /* L2 */
    float tibia_length_mm;             /* L3 */
    float theta3_deg;                  /* fixed tibia angle. v1 constant, v2 a solve variable */

    /* --- mounting, per leg. D23, D24, D25 --- */
    float coxa_x_mm[HEX_LEGS];
    float coxa_y_mm[HEX_LEGS];
    float beta_mount_deg[HEX_LEGS];    /* frame yaw of the coxa axis */
    float beta_neutral_deg[HEX_LEGS];  /* commanded yaw at neutral. Kept separate, D23 */

    /* --- stance posture --- */
    float theta2_nom_deg;              /* commanded femur pitch at neutral */
    float stride_mm;
    float swing_clearance_mm;

    /* --- gait --- */
    float duty_factor;                 /* D146 */
    float swing_peak_factor;           /* D145 half-sine => pi/2. peak divided by mean */
    float dtheta_peak_deg_s;           /* peak joint rate the profile is scaled to. D62 */
    float frame_period_us;

    /* --- stale handling. D144 --- */
    float stale_ramp_ms;
    float swing_eps_mm_s;

    /* --- actuator. D186, D240 --- */
    float command_step_deg;            /* bus command grid */
    float joint_accuracy_deg;          /* datasheet accuracy. The binding limit, D240 */

    /* --- joint envelopes. D258. Scalars removed, not deprecated ---
     *
     * KINEMATIC SPACE, like every other angle in this struct (D262). These are
     * NOT vendor servo-command angles. The vendor's two sides are mirrored —
     * pwm_R = 3000 - pwm_L (D248), so angle_R = -angle_L — and converting a
     * vendor table into these fields is a per-JOINT sign map, not a per-leg
     * one:
     *
     *   coxa   theta1 : identity on all six legs
     *   femur  theta2 : identity on the left three, negated on the right three
     *   tibia  theta3 : identity on the left three, negated on the right three
     *
     * theta1 is antisymmetric between the sides and theta2/theta3 are
     * symmetric, so the vendor's uniform mirror cancels for the coxa and does
     * not cancel for the other two. Applying one sign to a whole leg silently
     * reflects its coxa window. The command OFFSET is separate, is not assumed
     * zero, and is Spider Hardware's (D262).
     */
    float joint_min_deg[HEX_JOINTS];   /* interleaved: HEX_COXA(leg), HEX_FEMUR(leg) */
    float joint_max_deg[HEX_JOINTS];
    float theta3_min_deg;              /* tibia envelope, one pair for all six legs (D221) */
    float theta3_max_deg;              /* checked once at init against theta3_deg */

    /* --- invariant check inputs. D162 form, D188 constant --- */
    float mass_kg;
    float tau_servo_kgcm;
    float margin_factor;               /* D261 fixes this at 2.5000. No default is
                                        * supplied here; the caller still states it. */
} hex_config_t;

/* Derived once at init, never stored in the config, never hand-written.
 *
 * With theta3 constant the three-segment leg reduces EXACTLY to a two-segment
 * leg — an identity, not an approximation:
 *
 *   L2*cos(t2) + L3*cos(t2+t3) = R*cos(t2+psi)
 *   L2*sin(t2) + L3*sin(t2+t3) = R*sin(t2+psi)
 *
 *   R     = sqrt(L2^2 + L3^2 + 2*L2*L3*cos(theta3))
 *   psi   = atan2(L3*sin(theta3), L2 + L3*cos(theta3))
 *   Theta = theta2 + psi
 *
 * a_eff (D161) and the horizontal reach from the femur axis are the SAME
 * quantity: a_eff = R*cos(Theta).
 */
typedef struct {
    float rigid_len_mm;     /* R */
    float psi_deg;          /* psi */
    float theta_nom_deg;    /* Theta0 = theta2_nom_deg + psi_deg */
    float a_eff_nom_mm;     /* R*cos(Theta0) */
    float body_height_mm;   /* R*sin(Theta0) */
    float r_nom_mm;         /* coxa_length_mm + a_eff_nom_mm */
} hex_derived_t;

/* Zero is success. Every other value names exactly one failure.
 * APPEND ONLY. Appending does not renumber and no caller breaks. */
typedef enum {
    HEX_CFG_OK = 0,
    HEX_CFG_E_MEMBER,      /* a member length is non-positive */
    HEX_CFG_E_DUTY,        /* duty_factor outside (0,1) */
    HEX_CFG_E_STRIDE,      /* stride_mm non-positive */
    HEX_CFG_E_CLEARANCE,   /* swing_clearance_mm not strictly below body height */
    HEX_CFG_E_RATE,        /* dtheta_peak_deg_s or frame_period_us non-positive */
    HEX_CFG_E_PROFILE,     /* swing_peak_factor < 1 */
    HEX_CFG_E_REACH,       /* the stride extreme is not reachable: r_ext - L1 >= R */
    HEX_CFG_E_RAMP,        /* stale_ramp_ms does not exceed one swing duration (S-c) */
    HEX_CFG_E_MARGIN,      /* margin_factor < 1 */
    HEX_CFG_E_RESOLUTION,  /* D240: command_step_deg or joint_accuracy_deg non-positive */
    HEX_CFG_E_THETA3       /* D258: theta3_deg outside [theta3_min_deg, theta3_max_deg],
                            * or the envelope pair is not ordered */
} hex_cfg_err_t;

hex_cfg_err_t hex_config_validate(const hex_config_t *cfg);
hex_cfg_err_t hex_derive(const hex_config_t *cfg, hex_derived_t *out);

/* --- layout introspection. COREDROP_02 §3.2, for the ctypes binding. ---
 *
 * Pure. No state, no allocation. index is 0 .. hex_config_field_count()-1 in
 * declaration order. An array member is ONE field; its size is the whole array.
 * Out of range: NULL for the name, (size_t)-1 for both offset and size. */
size_t      hex_config_sizeof(void);
int         hex_config_field_count(void);
const char *hex_config_field_name(int index);
size_t      hex_config_field_offset(int index);
size_t      hex_config_field_size(int index);

#ifdef __cplusplus
}
#endif
#endif /* HEX_CONFIG_H */
