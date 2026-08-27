/* ik_core.c — leg kinematics.
 *
 * Owner: algorithm workstream (D6.1, D151). C99.
 * STATELESS. There is no ik_init and none may be created (D61). Every function
 * below is pure: same inputs, same outputs, no retained state, no allocation,
 * no I/O, no blocking.
 *
 * LEG FRAME. Origin on the coxa axis. +x along the coxa neutral direction,
 * +y to the left of it, +z up. theta1 is the coxa yaw measured in this frame,
 * theta2 the femur pitch above horizontal. The foot is BELOW the axis, so
 * z is negative in a standing pose.
 *
 * With theta3 constant the leg is exactly two-segment (D207):
 *
 *   Theta = theta2 + psi
 *   a_eff = R * cos(Theta)     horizontal reach from the femur axis
 *   drop  = R * sin(Theta)     foot below the femur axis
 *   r     = L1 + a_eff         horizontal reach from the coxa axis
 *
 * The reachable set is therefore a SURFACE, not a volume: every reachable
 * foot position sits at exactly distance R from the femur axis. A general
 * (x,y,z) is off that surface and ik_solve_leg says so.
 *
 * NOT MODELLED: D227's -0.5000 mm vertical drop from the coxa axis to the
 * femur axis. hex_config_t has no field for it, so the coxa is treated as
 * horizontal. The effect is a constant -0.5000 mm bias in leg-frame z,
 * identical on all six legs. Reported, not silently absorbed.
 */
#include "ik_core.h"
#include <math.h>

#define IK_DEG2RAD  0.01745329251994329577f
#define IK_RAD2DEG  57.29577951308232088f

/* Acceptance half-width for IK_PROJ_NONE, as a RELATIVE quantity, so it
 * scales with R instead of being a magic millimetre. 64 * FLT_EPSILON is
 * about 7.6e-6 relative, i.e. about 1.4e-3 mm at R = 180. Measured worst
 * ik_fk_leg -> ik_solve_leg round-trip residual is far inside it, and it is
 * three orders below the 0.1350 deg command grid's foot step, so it can
 * never mask a real reach failure. */
#define IK_REL_TOL  7.62939453125e-06f   /* 64 * 2^-23 */

void ik_fk_leg(const hex_config_t *cfg, const hex_derived_t *d,
               float theta1_deg, float theta2_deg,
               float *x_mm, float *y_mm, float *z_mm)
{
    float theta, r, t1;

    theta = (theta2_deg + d->psi_deg) * IK_DEG2RAD;
    r     = cfg->coxa_length_mm + d->rigid_len_mm * cosf(theta);
    t1    = theta1_deg * IK_DEG2RAD;

    *x_mm = r * cosf(t1);
    *y_mm = r * sinf(t1);
    *z_mm = -d->rigid_len_mm * sinf(theta);
}

ik_status_t ik_solve_leg(const hex_config_t *cfg, const hex_derived_t *d,
                         float x_mm, float y_mm, float z_mm,
                         ik_proj_t mode,
                         float *theta1_deg, float *theta2_deg)
{
    float R, L1, r, a, h, dist, tol, theta, t1, t2;

    R  = d->rigid_len_mm;
    L1 = cfg->coxa_length_mm;

    t1 = atan2f(y_mm, x_mm);
    r  = sqrtf(x_mm * x_mm + y_mm * y_mm);

    a  = r - L1;        /* horizontal reach from the femur axis */
    h  = -z_mm;         /* drop below the femur axis */

    dist = sqrtf(a * a + h * h);
    tol  = R * IK_REL_TOL;

    if (dist > R + tol || dist < R - tol) {
        if (mode == IK_PROJ_NONE)
            return (dist > R) ? IK_E_UNREACHABLE_FAR : IK_E_UNREACHABLE_NEAR;

        if (mode == IK_PROJ_RADIAL) {
            /* Hold height, clamp radial reach. Needs |h| <= R. */
            if (h > R || h < -R) return IK_E_UNREACHABLE_FAR;
            a = sqrtf(R * R - h * h);
            if (a < 0.0f) a = 0.0f;
        } else {                       /* IK_PROJ_VERTICAL */
            /* Hold radial reach, clamp height. Needs |a| <= R. */
            if (a > R || a < -R) return IK_E_UNREACHABLE_FAR;
            h = sqrtf(R * R - a * a);
        }
    }

    /* Theta is unambiguous: a and h are the two legs of the same right
     * triangle whose hypotenuse is R. Elbow choice does not arise, because
     * theta3 is fixed and there is only one free joint left. */
    theta = atan2f(h, a);
    t2    = theta * IK_RAD2DEG - d->psi_deg;
    t1    = t1 * IK_RAD2DEG;

    *theta1_deg = t1;
    *theta2_deg = t2;

    /* IK_E_LIMIT is NOT returned here. This signature carries no leg index,
     * so the per-joint envelope of D258 cannot be selected from inside this
     * function. The envelope is checked by the caller, which knows the leg,
     * at HEX_COXA(leg) and HEX_FEMUR(leg). Stated rather than skipped. */
    return IK_OK;
}

void ik_body_to_leg(const hex_config_t *cfg, int leg,
                    float bx_mm, float by_mm, float bz_mm,
                    float *lx_mm, float *ly_mm, float *lz_mm)
{
    float b, cb, sb, dx, dy;

    /* beta_neutral_deg is NOT applied. It is a command offset, not a frame
     * rotation, and folding the two together is the D23 error. */
    b  = cfg->beta_mount_deg[leg] * IK_DEG2RAD;
    cb = cosf(b);
    sb = sinf(b);

    dx = bx_mm - cfg->coxa_x_mm[leg];
    dy = by_mm - cfg->coxa_y_mm[leg];

    *lx_mm =  dx * cb + dy * sb;
    *ly_mm = -dx * sb + dy * cb;
    *lz_mm =  bz_mm;
}

void ik_leg_to_body(const hex_config_t *cfg, int leg,
                    float lx_mm, float ly_mm, float lz_mm,
                    float *bx_mm, float *by_mm, float *bz_mm)
{
    float b, cb, sb;

    b  = cfg->beta_mount_deg[leg] * IK_DEG2RAD;
    cb = cosf(b);
    sb = sinf(b);

    *bx_mm = lx_mm * cb - ly_mm * sb + cfg->coxa_x_mm[leg];
    *by_mm = lx_mm * sb + ly_mm * cb + cfg->coxa_y_mm[leg];
    *bz_mm = lz_mm;
}
