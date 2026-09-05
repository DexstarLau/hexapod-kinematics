/* ik_core.h — leg kinematics.
 *
 * Owner: algorithm workstream (D6.1, D151).
 * STATELESS. There is no ik_init and none may be created (D61). Every function
 * is pure: same inputs, same outputs, no retained state, no allocation, no I/O.
 * The caller owns the hex_derived_t and passes it in.
 *
 * Revision 2026-08-28: D287. IK_E_LIMIT's comment named the scalar joint limits
 * that D258 withdrew. Comment only; no signature, enum value or type changed.
 *
 * Revision 2026-09-05: D342 appends IK_W_REFLECTED; D344 corrects the reachable
 * set to the HALF-surface; D360 states that both recovered angles are congruent
 * modulo 360 and are written normalised. The enum is APPEND ONLY — no existing
 * enumerator changes value, IK_OK is still 0, and no signature changes.
 */
#ifndef IK_CORE_H
#define IK_CORE_H

#include "hex_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IK_OK = 0,
    IK_E_UNREACHABLE_FAR,   /* target beyond L1 + R */
    IK_E_UNREACHABLE_NEAR,  /* target inside the inner bound */
    IK_E_LIMIT,             /* solved, but outside the joint's envelope.
                             * D258 replaced the scalar limits with the
                             * joint_min_deg[HEX_JOINTS] / joint_max_deg[HEX_JOINTS]
                             * arrays, indexed HEX_COXA(leg) and HEX_FEMUR(leg).
                             *
                             * ik_solve_leg NEVER RETURNS THIS. Its signature
                             * carries no leg index, so it cannot select the
                             * envelope row. The value exists for the caller,
                             * which knows the leg, to return from its own check.
                             * Stated here so nobody waits for it. */
    IK_W_REFLECTED          /* solved on the reflected branch, and the solution is
                             * EXACT. The target was produced with
                             * r = L1 + R*cos(Theta) < 0, so the foot folds back past
                             * the coxa axis and sqrtf(x*x+y*y) cannot recover the
                             * sign. ik_solve_leg tests this branch before projecting
                             * and writes theta1/theta2 for it. The pose is correct;
                             * the status exists because the caller cannot tell from
                             * the position alone that its coxa is on the far side.
                             * Returned by all three ik_proj_t modes. D342.
                             * theta1/theta2 are normalised into (-180,+180]. Both
                             * are determined modulo 360 and the caller's envelope
                             * check runs on the normalised value. D360. */
} ik_status_t;

/* What to do when the target is unreachable. IK_PROJ_NONE returns the error
 * rather than a pose, so a caller that wants a hard failure gets one. */
typedef enum {
    IK_PROJ_NONE = 0,   /* do not project. Return the status and write nothing */
    IK_PROJ_RADIAL,     /* clamp radial reach, hold height */
    IK_PROJ_VERTICAL    /* clamp height, hold radial reach */
} ik_proj_t;

/* Forward: joint angles -> foot position, LEG frame, mm.
 * Always succeeds; there is no unreachable forward case. */
void ik_fk_leg(const hex_config_t *cfg, const hex_derived_t *d,
               float theta1_deg, float theta2_deg,
               float *x_mm, float *y_mm, float *z_mm);

/* Inverse: foot position, LEG frame -> joint angles.
 * On IK_OK, on IK_W_REFLECTED and on a successful projection, theta1/theta2
 * are written. On IK_E_* with IK_PROJ_NONE, neither is written.
 *
 * The reachable set is the HALF-surface with L1 + R*cos(Theta) > 0. ik_fk_leg
 * does not require r > 0; where the foot folds back past the coxa axis the
 * azimuth flips by 180 deg and sqrtf(x*x + y*y) cannot recover the sign.
 * ik_solve_leg tests that branch explicitly and returns IK_W_REFLECTED with
 * the exact pose written (D342, D344).
 *
 * theta1 and theta2 are written normalised into (-180, +180]. Both are
 * determined only up to a multiple of 360; the caller's envelope check under
 * D258 runs on the normalised value, and a pose that is physically inside its
 * envelope can be returned outside it. Where a caller needs the un-normalised
 * branch it computes it itself, from the pose, and ik_solve_leg does not carry
 * a hint. D360. */
ik_status_t ik_solve_leg(const hex_config_t *cfg, const hex_derived_t *d,
                         float x_mm, float y_mm, float z_mm,
                         ik_proj_t mode,
                         float *theta1_deg, float *theta2_deg);

/* Body frame <-> leg frame, using coxa_x/y_mm and beta_mount_deg.
 * beta_neutral_deg is NOT applied here — it is a command offset, not a frame
 * rotation, and folding the two together is the D23 error. */
void ik_body_to_leg(const hex_config_t *cfg, int leg,
                    float bx_mm, float by_mm, float bz_mm,
                    float *lx_mm, float *ly_mm, float *lz_mm);

void ik_leg_to_body(const hex_config_t *cfg, int leg,
                    float lx_mm, float ly_mm, float lz_mm,
                    float *bx_mm, float *by_mm, float *bz_mm);

#ifdef __cplusplus
}
#endif
#endif /* IK_CORE_H */
