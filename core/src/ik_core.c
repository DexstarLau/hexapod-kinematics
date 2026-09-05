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
 * The reachable set is therefore the HALF-surface with r = L1 + R*cos(Theta)
 * strictly positive: every reachable foot sits at exactly distance R from the
 * femur axis, on the near side of the coxa axis. ik_fk_leg does not require
 * r > 0; where the foot folds back past the coxa axis the azimuth flips by
 * 180 deg and sqrtf(x*x + y*y) cannot recover the sign. ik_solve_leg tests
 * that branch explicitly and returns IK_W_REFLECTED with the exact pose
 * written (D342, D344). A general (x,y,z) is off the surface altogether and
 * ik_solve_leg says so.
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

/* Normalise a degree value into (-180, +180]. D360.
 *
 * Both recovered angles are determined only up to a multiple of 360.
 * atan2f returns (-180, +180], so where the input theta2 + psi lay outside
 * that interval the exact solution comes back one full turn away. It is the
 * SAME POSE, but the caller's D258 envelope check runs on the returned number
 * and not on the pose, so the value that is returned has to be the canonical
 * one. Coordination's 38,269-row sweep found 7,597 rows whose raw theta2 sat
 * a full turn from the input and none whose theta1 did; the vendor corpus does
 * not reach the wrap, and gait_core is not confined to the corpus.
 *
 * The loops run AT MOST ONCE for anything this file produces: t1 is in
 * (-360, +360] after the reflected 180 and t2 is in (-360, +360), because
 * theta and psi are each in (-180, +180]. This is a bounded cost on the 50 Hz
 * path. The single 360.0f subtraction is exact to within one ULP at that
 * scale, about 3.05e-05 deg, which is the worst |dtheta2| the sweep measured. */
static float ik_wrap180f(float deg)
{
    while (deg >   180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

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
    float R, L1, r, a, h, dist, tol, theta, t1, t2, a_ref, dist_ref;
    int   reflected = 0;

    R  = d->rigid_len_mm;
    L1 = cfg->coxa_length_mm;

    t1 = atan2f(y_mm, x_mm);
    r  = sqrtf(x_mm * x_mm + y_mm * y_mm);

    a  = r - L1;        /* horizontal reach from the femur axis */
    h  = -z_mm;         /* drop below the femur axis */

    dist = sqrtf(a * a + h * h);
    tol  = R * IK_REL_TOL;

    if (dist > R + tol || dist < R - tol) {
        /* D342. The REFLECTED pre-image is tested here, before any projection
         * branch runs, and in all three modes.
         *
         * It has to be here rather than after projection for two reasons. It
         * is computable from the inputs and L1 alone, so nothing is waiting
         * on it. And a reflected target IS REACHABLE: projection returns the
         * nearest reachable point, and here the nearest reachable point is the
         * requested one, at distance zero. Placed after projection, a has
         * already been overwritten with +sqrtf(R*R - h*h) and the pre-image is
         * unrecoverable — that is D342 section 2's 2*L1 = 84.0000 mm
         * displacement, which was identical on all 1,066 folded corpus rows.
         *
         * a_ref and a cannot both match R unless r or L1 is zero, since
         * (r-L1)^2 == (r+L1)^2 requires 4*r*L1 == 0, so the order of the two
         * tests does not change any answer. The direct branch is tested first
         * only so that the extra sqrtf stays off the hit path. */
        a_ref    = -(r + L1);
        dist_ref = sqrtf(a_ref * a_ref + h * h);

        if (dist_ref <= R + tol && dist_ref >= R - tol) {
            a         = a_ref;   /* a < 0 puts theta in Q2, which is right */
            reflected = 1;
        } else if (mode == IK_PROJ_NONE) {
            return (dist > R) ? IK_E_UNREACHABLE_FAR : IK_E_UNREACHABLE_NEAR;

        } else if (mode == IK_PROJ_RADIAL) {
            /* Hold height, clamp radial reach. Needs |h| <= R. */
            if (h > R || h < -R) return IK_E_UNREACHABLE_FAR;
            a = sqrtf(R * R - h * h);
            if (a < 0.0f) a = 0.0f;
        } else {                       /* IK_PROJ_VERTICAL */
            /* Hold radial reach, clamp height. Needs |a| <= R. */
            if (a > R || a < -R) return IK_E_UNREACHABLE_FAR;
            h = sqrtf(R * R - a * a);
        }
        /* D342 clause 4: only a target that misses the reflected branch too
         * is projected, and it is projected exactly as it was before. */
    }

    /* Theta is unambiguous: a and h are the two legs of the same right
     * triangle whose hypotenuse is R. Elbow choice does not arise, because
     * theta3 is fixed and there is only one free joint left. */
    theta = atan2f(h, a);
    t2    = theta * IK_RAD2DEG - d->psi_deg;
    t1    = t1 * IK_RAD2DEG;

    /* On the reflected branch the true coxa azimuth is half a turn from the
     * one atan2f recovered from the position, because the foot is on the far
     * side of the coxa axis. D342 section 3. */
    if (reflected)
        t1 += 180.0f;

    /* D360. Both angles are congruent modulo 360 and are written canonical. */
    *theta1_deg = ik_wrap180f(t1);
    *theta2_deg = ik_wrap180f(t2);

    /* IK_E_LIMIT is NOT returned here. This signature carries no leg index,
     * so the per-joint envelope of D258 cannot be selected from inside this
     * function. The envelope is checked by the caller, which knows the leg,
     * at HEX_COXA(leg) and HEX_FEMUR(leg). Stated rather than skipped. */
    return reflected ? IK_W_REFLECTED : IK_OK;
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
