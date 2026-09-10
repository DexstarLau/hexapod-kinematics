/* smoke_test.c — exercises S-a..S-g and the walk path. Not the delivered test
 * suite; a bench check run by this workstream before delivery. */
#include "hex_config.h"
#include "ik_core.h"
#include "gait_core.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static hex_config_t cfg;
static int fails = 0;

static void check(int cond, const char *what) {
    printf("  [%s] %s\n", cond ? "PASS" : "FAIL", what);
    if (!cond) fails++;
}

static int all_finite(const float a[12]) {
    for (int i = 0; i < 12; i++) if (!isfinite(a[i])) return 0;
    return 1;
}
static int same12(const float a[12], const float b[12]) {
    for (int i = 0; i < 12; i++) if (fabsf(a[i]-b[i]) > 1e-6f) return 0;
    return 1;
}

int main(void) {
    memset(&cfg, 0, sizeof(cfg));
    cfg.coxa_length_mm=42.0f; cfg.femur_length_mm=74.2f; cfg.tibia_length_mm=112.6231f;
    cfg.theta3_deg=-30.0f;
    /* Realistic mounts: R1/R2/R3 down the right side, L1/L2/L3 down the left,
     * each coxa axis splayed outward. The first fixture gave all three right
     * legs beta_mount = 0, which points them the same way and makes a forward
     * stride purely radial -- the coxa never rotates, so a coxa sweep test has
     * no power. Caught by the sweep assertion, not by inspection. */
    float xs[6]={ 60.0f,  0.0f, -60.0f,  60.0f,  0.0f, -60.0f};
    float ys[6]={ 60.0f, 80.0f,  60.0f, -60.0f,-80.0f, -60.0f};
    float bm[6]={ 45.0f,  0.0f, -45.0f, 135.0f,180.0f,-135.0f};
    for (int i=0;i<HEX_LEGS;i++){
        cfg.coxa_x_mm[i]=xs[i];
        cfg.coxa_y_mm[i]=ys[i];
        cfg.beta_mount_deg[i]=bm[i];
        cfg.beta_neutral_deg[i]=0.0f;
    }
    cfg.theta2_nom_deg=40.5f; cfg.stride_mm=60.0f; cfg.swing_clearance_mm=20.0f;
    cfg.duty_factor=0.5f; cfg.swing_peak_factor=1.5707963f;
    cfg.dtheta_peak_deg_s=250.0f; cfg.frame_period_us=20000.0f;
    cfg.stale_ramp_ms=300.0f; cfg.swing_eps_mm_s=5.0f;
    cfg.command_step_deg=0.135f; cfg.joint_accuracy_deg=1.0f;
    for (int i=0;i<HEX_JOINTS;i++){cfg.joint_min_deg[i]=-90.0f;cfg.joint_max_deg[i]=90.0f;}
    cfg.theta3_min_deg=-60.0f; cfg.theta3_max_deg=0.0f;
    cfg.mass_kg=2.15f; cfg.tau_servo_kgcm=20.0f; cfg.margin_factor=2.5f;

    printf("hex_config_validate: %d (0 = HEX_CFG_OK)\n\n", (int)hex_config_validate(&cfg));

    float out[12], prev[12];
    int rc;

    printf("S-g  gait_init clears the flag to FRESH\n");
    gait_init(&cfg);
    check(gait_set_stale(0) == 0, "previous state after init reads 0 (fresh)");
    gait_init(&cfg);

    printf("\nS-a  level-triggered, idempotent, returns previous state\n");
    check(gait_set_stale(1) == 0, "0 -> 1 returns previous 0");
    check(gait_set_stale(1) == 1, "1 -> 1 returns previous 1 (idempotent)");
    check(gait_set_stale(0) == 1, "1 -> 0 returns previous 1");
    check(gait_set_stale(7) == 0, "any non-zero is stale; returns previous 0");
    gait_init(&cfg);

    printf("\nwalk: 60 frames forward at 80 mm/s, 20 ms each\n");
    int tripod_alternated = 0;
    float first_coxa_r1 = 0.0f, min_r1 = 1e9f, max_r1 = -1e9f;
    for (int i=0;i<60;i++){
        rc = gait_step(80.0f, 0.0f, 0.0f, 20.0f, out);
        if (rc!=0){ printf("  frame %d rc=%d\n", i, rc); fails++; break; }
        if (!all_finite(out)) { fails++; break; }
        if (i==0) first_coxa_r1 = out[HEX_COXA(HEX_R1)];
        if (out[HEX_COXA(HEX_R1)] < min_r1) min_r1 = out[HEX_COXA(HEX_R1)];
        if (out[HEX_COXA(HEX_R1)] > max_r1) max_r1 = out[HEX_COXA(HEX_R1)];
    }
    (void)first_coxa_r1; (void)tripod_alternated;
    check(rc==0, "60 frames all return 0");
    check(max_r1 - min_r1 > 1.0f, "R1 coxa actually sweeps (not frozen)");
    printf("  R1 coxa range: %.4f .. %.4f deg (sweep %.4f)\n", min_r1, max_r1, max_r1-min_r1);

    printf("\ntripod: the two groups are never both airborne\n");
    /* femur angle differing from the stance value indicates lift; compare
     * group A and group B femurs across a full cycle. */
    gait_init(&cfg);
    int both_up_frames = 0;
    for (int i=0;i<400;i++){
        gait_step(80.0f, 0.0f, 0.0f, 20.0f, out);
        float a = out[HEX_FEMUR(HEX_R1)];   /* group A */
        float b = out[HEX_FEMUR(HEX_R2)];   /* group B */
        int a_up = fabsf(a - cfg.theta2_nom_deg) > 3.0f;
        int b_up = fabsf(b - cfg.theta2_nom_deg) > 3.0f;
        if (a_up && b_up) both_up_frames++;
    }
    check(both_up_frames == 0, "no frame has both tripod groups lifted");

    printf("\nS-c/S-b  stale ramp: legs finish their swing, then the pose holds\n");
    gait_init(&cfg);
    for (int i=0;i<37;i++) gait_step(80.0f,0.0f,0.0f,20.0f,out); /* mid-swing */
    gait_set_stale(1);
    int settle = -1;
    for (int i=0;i<200;i++){
        memcpy(prev,out,sizeof(out));
        rc = gait_step(80.0f,0.0f,0.0f,20.0f,out);
        if (rc!=0){ printf("  rc=%d at %d\n", rc, i); fails++; break; }
        if (!all_finite(out)) { printf("  NaN at %d\n", i); fails++; break; }
        if (i>0 && same12(prev,out) && settle<0) settle = i;
    }
    check(rc==0 && settle>=0, "output settles to a constant held pose");
    printf("  settled after %d frames past the stale edge (ramp %.0f ms = %.0f frames)\n",
           settle, cfg.stale_ramp_ms, cfg.stale_ramp_ms/20.0f);

    printf("\nS-d  held pose persists indefinitely, twelve valid floats\n");
    memcpy(prev,out,sizeof(out));
    int drift = 0;
    for (int i=0;i<500;i++){
        gait_step(80.0f,0.0f,0.0f,20.0f,out);
        if (!all_finite(out)) { drift++; break; }
        if (!same12(prev,out)) { drift++; break; }
    }
    check(drift==0, "500 further frames emit the same twelve finite floats");

    printf("\nall six feet planted in the held pose\n");
    int lifted = 0;
    for (int leg=0; leg<HEX_LEGS; leg++)
        if (fabsf(out[HEX_FEMUR(leg)] - cfg.theta2_nom_deg) > 6.0f) lifted++;
    check(lifted==0, "no leg left airborne when motion stopped");

    printf("\nrecovery: clearing stale resumes motion\n");
    check(gait_set_stale(0)==1, "clearing returns previous 1");
    memcpy(prev,out,sizeof(out));
    for (int i=0;i<10;i++) gait_step(80.0f,0.0f,0.0f,20.0f,out);
    check(!same12(prev,out), "the robot moves again after the flag clears");

    printf("\nturn in place, 60 frames at 45 deg/s\n");
    gait_init(&cfg);
    rc = 0;
    for (int i=0;i<60 && rc==0;i++) rc = gait_step(0.0f,0.0f,45.0f,20.0f,out);
    check(rc==0 && all_finite(out), "pure rotation runs clean");

    printf("\nuninitialised guard\n");
    /* cannot un-init; documented -1 path is exercised by inspection only */

    printf("\n%s  (%d failures)\n", fails? "SOME CHECKS FAILED":"ALL CHECKS PASSED", fails);
    return fails ? 1 : 0;
}
