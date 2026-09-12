/* att_core.h — attitude estimator, frozen API (D6.1, D20, D30).
 *
 * Owner: algorithm workstream. STATEFUL and NOT RE-ENTRANT: single caller.
 * No allocation, no I/O, no blocking (D6.1). Called from the 100 Hz task in
 * TASK CONTEXT, not interrupt context (D20).
 *
 * Started 2026-09-10. HANDOFF_65 section 1.1 cleared the start and reports
 * that D20's signatures are current. This workstream re-read D20 (register
 * lines 609-633) and D30 (lines 806-834) directly rather than taking the
 * status line: D30 corrects D20's REASONING and states "D20's conclusion
 * stands". The API is not among what it corrects. Confirmed at source, which
 * closes the one assumption HANDOFF_65's Unverified block left open.
 *
 * WHAT D20 FIXES, AND IS NOT NEGOTIABLE HERE:
 *   - both signatures below, verbatim
 *   - gyro in deg/s, accel in m/s^2, BOTH ALREADY ROTATED INTO THE BODY FRAME
 *     by firmware. This filter knows nothing about how the IMU is glued down
 *   - out_rpy in degrees
 *   - attitude does NOT feed gait_step in v1. The estimate serves telemetry,
 *     logging and the RL observation vector only. att_core and gait_core have
 *     no ordering dependency and are scheduled independently
 *
 * WHAT NO DECISION FIXES, AND IS THEREFORE THIS WORKSTREAM'S PROPOSAL:
 *   att_config_t's FIELDS. D20 names the type and defines no member. The
 *   struct below is filed for ratification the way HANDOFF_30-AI section 2.1
 *   filed gait_set_stale — this workstream defining its own interface, not
 *   requesting one. Until it is ratified the struct may move; the two
 *   function signatures may not.
 *
 * YAW HAS NO ABSOLUTE REFERENCE. READ THIS BEFORE CONSUMING out_rpy[2].
 *   Roll and pitch are observable: gravity gives them an absolute reference
 *   whenever the accelerometer is measuring mostly gravity. YAW IS NOT. With
 *   a gyro and an accelerometer and no magnetometer, yaw is an integral with
 *   no correction available, and it drifts without bound.
 *   out_rpy[2] is therefore RELATIVE TO WHERE att_init WAS CALLED and is
 *   meaningful over seconds, not minutes. D30 consequence 2 puts the attitude
 *   estimate in the RL observation vector from the start; a drifting yaw in an
 *   observation vector means different things at different times and a policy
 *   will learn on it anyway. THIS IS RAISED TO COORDINATION, not decided here.
 */
#ifndef ATT_CORE_H
#define ATT_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

/* PROPOSED, not ratified — see the header note. No defaults ship, matching
 * hex_config_t's rule: a core with no defaults cannot silently run on a stale
 * constant, because it cannot run at all without being told the constants. */
typedef struct {
    /* Local gravity, m/s^2. Not hard-coded: the accelerometer's scale factor
     * and the local value both enter here, and D78 requires the log to state
     * its scale factors rather than assume them. */
    float gravity_mps2;

    /* Complementary time constant, seconds. Above it the gyro dominates;
     * below it the accelerometer does. Sets the crossover between gyro drift
     * and accelerometer noise. Gait bob is a clean periodic signal at gait
     * frequency (D11's observability argument) and this constant decides
     * whether the filter tracks it or rejects it.
     *
     * D392: tau_s IS NOT CHOSEN BY TASTE. Under an undeclared residual gyro
     * bias the steady-state tilt error of this filter is
     *
     *     error_deg = residual_bias_dps * tau_s          [deg/s * s = deg]
     *
     * verified against this implementation across sixteen (bias, tau) pairs,
     * ratio 1.0000 in every one. So given a tolerable tilt error and a
     * measured residual bias, tau_s follows:
     *
     *     tau_s = tolerable_error_deg / residual_bias_dps
     *
     * NOT HELD, and it is what stops the formula being usable today: the
     * residual bias after D78's stationary segment. Every result on the record
     * is from synthetic input. Do not invent one. */
    float tau_s;

    /* Accelerometer trust gate, m/s^2. When | ||a|| - gravity_mps2 | exceeds
     * this, the sample is not measuring gravity alone and the accelerometer
     * correction is SUPPRESSED for that step, leaving pure gyro integration.
     * Without this gate, every footfall transient is read as a tilt. */
    float accel_gate_mps2;

    /* Stationary gyro bias, deg/s, body frame, subtracted from every sample.
     * D78's log carries >= 120 s stationary for exactly this. Supplied by the
     * caller; this core does not estimate it, because a core that quietly
     * re-zeroes its own bias hides a failing IMU. */
    float gyro_bias_dps[3];
} att_config_t;

/* D20, verbatim. Clears all state. The first att_step after this seeds roll
 * and pitch from the accelerometer rather than converging from zero, and sets
 * yaw to 0 — which is what "relative to where att_init was called" means. */
void att_init(const att_config_t *cfg);

/* D20, verbatim.
 *
 * Called from the 100 Hz task, TASK CONTEXT, not interrupt context.
 * gyro deg/s, accel m/s^2, both raw and ALREADY rotated into the body
 * frame by firmware. out_rpy in degrees.
 * State owned internally. NOT re-entrant: single caller only.
 * No allocation, no I/O, no blocking.
 *
 * Always writes three finite floats. out_rpy[0] roll, [1] pitch, [2] yaw,
 * each wrapped into (-180, +180]. Yaw carries no absolute reference; see the
 * header note.
 *
 * RETURN, D389, amending D20 part 1 to this extent and no further:
 *   0        the estimate is valid
 *   non-zero the estimate is NOT valid. At minimum this is returned when
 *            att_init has not been called, which is the ONLY non-zero case
 *            this implementation currently produces. Test for zero, not for a
 *            particular value: no code set is defined and none is implied.
 * out_rpy IS STILL WRITTEN ON EVERY CALL, three finite floats, never NaN,
 * never left indeterminate — including on a non-zero return. A caller that
 * ignores the return therefore gets v1's behaviour exactly. */
int att_step(const float gyro[3], const float accel[3],
             float dt_ms, float out_rpy[3]);

#ifdef __cplusplus
}
#endif
#endif /* ATT_CORE_H */
