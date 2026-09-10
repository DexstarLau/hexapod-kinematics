/* gait_core.h — tripod gait engine, frozen API (D6.1, D144-D146).
 *
 * Owner: algorithm workstream. STATEFUL: gait_init sets internal phase and
 * config; gait_step and gait_set_stale act on that state. No allocation, no
 * I/O, no blocking (D6.1). One robot per process — the state is a single
 * static instance, matching D6's one-MCU-per-robot deployment.
 *
 * Started 2026-09-10 under D380 (HANDOFF_63).
 *
 * SIGNATURE PROVENANCE — READ THIS BEFORE CHANGING ANYTHING HERE.
 *
 *   gait_init and gait_step are D6.1's, unchanged. The register confirms
 *   gait_step's signature does not change ("vbat does not enter gait_core").
 *
 *   gait_set_stale is HANDOFF_30-AI section 2.1's, ratified WITHOUT AMENDMENT
 *   by D144, which also states "the interface is closed". It takes an int
 *   LEVEL and returns the previous level. It does NOT take an age in ms.
 *
 *   DECISION_REGISTER_D1_D269_CONSOLIDATED.md still prints the pre-D144 form
 *   `void gait_set_stale(uint32_t age_ms)` in D6.1's code block, and uses the
 *   same form again in D144's own neighbourhood. THAT TEXT IS STALE; D144
 *   supersedes it. This workstream built against the stale form first and had
 *   to rebuild. The register defect is raised to coordination separately so
 *   the next reader does not repeat it.
 *
 * ONE ITEM FLAGGED AS THIS WORKSTREAM'S OWN CHOICE, not a decision:
 *   the tripod phase-group assignment ({R1,R3,L2} vs {R2,L1,L3}) in
 *   gait_core.c. No decision fixes the grouping and none is blocked on one,
 *   so the choice is made under D107 and disclosed rather than waited on.
 */
#ifndef GAIT_CORE_H
#define GAIT_CORE_H

#include "hex_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Set the config and clear all internal state. The caller owns cfg and it
 * must outlive every gait_step call. S-g: the stale flag is cleared to FRESH
 * here, so a core that has never been told anything behaves as commanded. */
void gait_init(const hex_config_t *cfg);

/* One 50 Hz frame. Writes twelve angles, interleaved HEX_COXA(leg) /
 * HEX_FEMUR(leg), KINEMATIC SPACE (D262), degrees.
 *
 * Returns 0 on success, -1 if gait_init was never called, -2 if a foot target
 * was unreachable even after projection.
 *
 * Never blocks, allocates or performs I/O. Always writes all twelve floats on
 * a 0 return and never writes a NaN (S-d). */
int gait_step(float vx, float vy, float omega, float dt_ms, float out_angles[12]);

/* gait_set_stale — mark the last host command as stale or fresh. D144,
 * HANDOFF_30-AI section 2.1, semantics S-a through S-g.
 *
 *   stale != 0   the (vx, vy, omega) triple most recently passed to gait_step
 *                is no longer trusted. gait_step MUST CONTINUE to be called
 *                at 50 Hz.
 *   stale == 0   the command is trusted again.
 *
 * Returns the previous state, 0 or 1, so the caller can log edges without
 * keeping a shadow copy.
 *
 * Level-triggered, idempotent, no allocation, no floating-point work, O(1).
 * Single writer assumed: call from the same context that calls gait_step, or
 * before it in the same 50 Hz frame. */
int gait_set_stale(int stale);

#ifdef __cplusplus
}
#endif
#endif /* GAIT_CORE_H */
