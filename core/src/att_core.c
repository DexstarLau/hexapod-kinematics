/* att_core.c — complementary attitude filter, v1.
 *
 * This workstream, 2026-09-10. D7 puts the complementary filter (and later an
 * EKF) on the MCU under D6.1; this is the complementary half.
 *
 * Units in: gyro deg/s, accel m/s^2, dt ms. Units out: degrees.
 * Frame: BODY, both inputs already rotated by firmware (D20). Nothing here
 * knows the IMU's mounting.
 */
#include "att_core.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG (180.0f / (float)M_PI)
#define RAD ((float)M_PI / 180.0f)

static att_config_t g_cfg;
static int   g_ready = 0;      /* att_init has run */
static int   g_seeded = 0;     /* first accelerometer sample has seeded r/p */
static float g_roll = 0.0f, g_pitch = 0.0f, g_yaw = 0.0f;   /* degrees */

/* Wrap into (-180, +180], matching ik_core's convention under D360 so the two
 * cores do not disagree about what an angle is. */
static float wrap180(float d)
{
    while (d >   180.0f) d -= 360.0f;
    while (d <= -180.0f) d += 360.0f;
    return d;
}

void att_init(const att_config_t *cfg)
{
    g_cfg = *cfg;                 /* copied, not aliased: att_step must not
                                   * depend on the caller keeping cfg alive at
                                   * 100 Hz. gait_init holds a pointer because
                                   * hex_config_t is large; this struct is not */
    g_ready = 1;
    g_seeded = 0;
    g_roll = g_pitch = g_yaw = 0.0f;
}

int att_step(const float gyro[3], const float accel[3],
             float dt_ms, float out_rpy[3])
{
    /* D389 supplied the channel v1 reported as missing. The gap is now closed
     * by decision rather than by this workstream's initiative: non-zero says
     * the estimate is not valid, and out_rpy is still written so no caller is
     * left holding indeterminate floats. */
    if (!g_ready) {
        out_rpy[0] = out_rpy[1] = out_rpy[2] = 0.0f;
        /* A single non-zero value, not a new enum. D389 authorises "non-zero";
         * a code SET is an interface this workstream may not add (RULE 3). */
        return 1;
    }

    float dt_s = dt_ms * 0.001f;
    if (!(dt_s > 0.0f)) dt_s = 0.0f;    /* also catches NaN: !(NaN > 0) is true */

    /* --- 1. Bias-corrected gyro, body frame, deg/s. */
    float gx = gyro[0] - g_cfg.gyro_bias_dps[0];
    float gy = gyro[1] - g_cfg.gyro_bias_dps[1];
    float gz = gyro[2] - g_cfg.gyro_bias_dps[2];

    /* --- 2. Accelerometer reference for roll and pitch.
     * Valid ONLY where the accelerometer is measuring gravity and little
     * else. The gate below is what decides that. */
    float ax = accel[0], ay = accel[1], az = accel[2];
    float a_norm = sqrtf(ax * ax + ay * ay + az * az);
    int accel_trusted = (a_norm > 1e-3f) &&
        (fabsf(a_norm - g_cfg.gravity_mps2) <= g_cfg.accel_gate_mps2);

    float roll_a = 0.0f, pitch_a = 0.0f;
    if (a_norm > 1e-3f) {
        roll_a  = atan2f(ay, az) * DEG;
        pitch_a = atan2f(-ax, sqrtf(ay * ay + az * az)) * DEG;
    }

    /* --- 3. Seed. Converging from zero after init would put a false transient
     * of up to the full tilt into the first tau_s of every log. */
    if (!g_seeded) {
        if (a_norm > 1e-3f) { g_roll = roll_a; g_pitch = pitch_a; }
        g_yaw = 0.0f;                       /* yaw is relative to HERE */
        g_seeded = 1;
        out_rpy[0] = g_roll; out_rpy[1] = g_pitch; out_rpy[2] = g_yaw;
        return 0;                           /* seeded estimate is valid */
    }

    /* --- 4. Gyro propagation, EULER KINEMATICS, not the naive sum.
     *
     *   roll_dot  = gx + sin(roll) tan(pitch) gy + cos(roll) tan(pitch) gz
     *   pitch_dot =      cos(roll)            gy - sin(roll)            gz
     *   yaw_dot   = (   sin(roll)             gy + cos(roll)            gz ) / cos(pitch)
     *
     * The naive roll += gx*dt is the small-angle case of this and carries a
     * systematic error that grows with tilt. D30 establishes that the body
     * pose subspace is 3-dimensional and that the RL reward must not assume a
     * level body, so the tilted case is not a corner here.
     *
     * SINGULAR at pitch = +/-90: yaw_dot and the tan terms both diverge. The
     * guard below freezes the coupling terms there rather than emitting an
     * infinity, and the condition is REPORTABLE in that out_rpy stops
     * tracking yaw. A hexapod at 90 degrees of pitch has a larger problem than
     * its attitude estimate, but a NaN propagating into telemetry and an RL
     * observation vector is still worse than a frozen number. */
    float sr = sinf(g_roll * RAD), cr = cosf(g_roll * RAD);
    float cp = cosf(g_pitch * RAD);
    float tp = tanf(g_pitch * RAD);

    const float CP_MIN = 0.017452f;         /* cos(89 deg) */
    if (fabsf(cp) < CP_MIN) {
        cp = (cp < 0.0f) ? -CP_MIN : CP_MIN;
        tp = (tp < 0.0f) ? -(1.0f / CP_MIN) : (1.0f / CP_MIN);
    }

    float roll_g  = g_roll  + dt_s * (gx + sr * tp * gy + cr * tp * gz);
    float pitch_g = g_pitch + dt_s * (     cr      * gy - sr      * gz);
    float yaw_g   = g_yaw   + dt_s * (    (sr      * gy + cr      * gz) / cp);

    /* --- 5. Complementary blend. alpha = tau / (tau + dt): gyro above the
     * crossover, accelerometer below it. */
    if (accel_trusted && g_cfg.tau_s > 0.0f) {
        float alpha = g_cfg.tau_s / (g_cfg.tau_s + dt_s);

        /* Blend the DIFFERENCE, wrapped, rather than the two angles directly.
         * A straight weighted mean of -179 and +179 gives 0, which is 180
         * degrees from both. */
        float d_roll  = wrap180(roll_a  - roll_g);
        float d_pitch = wrap180(pitch_a - pitch_g);
        g_roll  = roll_g  + (1.0f - alpha) * d_roll;
        g_pitch = pitch_g + (1.0f - alpha) * d_pitch;
    } else {
        /* Gate closed: this sample is not measuring gravity alone. Pure gyro
         * for this step. Every footfall transient would otherwise be read as
         * a tilt. */
        g_roll  = roll_g;
        g_pitch = pitch_g;
    }

    /* --- 6. Yaw: gyro only. No accelerometer correction exists for it and
     * none is invented. See the header note. */
    g_yaw = yaw_g;

    g_roll  = wrap180(g_roll);
    g_pitch = wrap180(g_pitch);
    g_yaw   = wrap180(g_yaw);

    out_rpy[0] = g_roll;
    out_rpy[1] = g_pitch;
    out_rpy[2] = g_yaw;

    return 0;
}
