/* ik_core.h — leg kinematics.
 *
 * Owner: algorithm workstream (D6.1, D151).
 * STATELESS. There is no ik_init and none may be created (D61). Every function
 * is pure: same inputs, same outputs, no retained state, no allocation, no I/O.
 * The caller owns the hex_derived_t and passes it in.
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
    IK_E_LIMIT              /* solved, but outside [joint_min_deg, joint_max_deg] */
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
 * On IK_OK and on a successful projection, theta1/theta2 are written.
 * On IK_E_* with IK_PROJ_NONE, neither is written. */
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
