/* att_bench_check.c — bench harness for att_core. Property checks and two
 * negative controls. Not the delivered test suite. */
#include "att_core.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

static int fails = 0;
static void check(int c, const char *w){ printf("  [%s] %s\n", c?"PASS":"FAIL", w); if(!c) fails++; }
#define G 9.80665f
#define RAD (float)(3.14159265358979323846/180.0)

static att_config_t base_cfg(void){
    att_config_t c; memset(&c,0,sizeof(c));
    c.gravity_mps2 = G;
    c.tau_s = 0.50f;
    c.accel_gate_mps2 = 0.60f;
    c.gyro_bias_dps[0]=c.gyro_bias_dps[1]=c.gyro_bias_dps[2]=0.0f;
    return c;
}
/* gravity as the body-frame accelerometer sees it at a given roll/pitch */
static void grav_body(float roll_d, float pitch_d, float a[3]){
    float r=roll_d*RAD, p=pitch_d*RAD;
    a[0] = -sinf(p)*G;
    a[1] =  sinf(r)*cosf(p)*G;
    a[2] =  cosf(r)*cosf(p)*G;
}
static int finite3(const float v[3]){ return isfinite(v[0])&&isfinite(v[1])&&isfinite(v[2]); }

int main(void){
    float rpy[3], g3[3]={0,0,0}, a3[3];
    att_config_t cfg = base_cfg();

    printf("1. level and stationary -> roll and pitch go to zero and stay\n");
    att_init(&cfg); grav_body(0,0,a3);
    for(int i=0;i<1000;i++) att_step(g3,a3,10.0f,rpy);
    check(fabsf(rpy[0])<1e-3f && fabsf(rpy[1])<1e-3f, "roll and pitch < 0.001 deg after 10 s");
    check(finite3(rpy), "all three outputs finite");

    printf("\n2. seeded from the accelerometer, not converged from zero\n");
    att_init(&cfg); grav_body(20.0f,-15.0f,a3);
    att_step(g3,a3,10.0f,rpy);
    printf("     first sample: roll %.4f  pitch %.4f  (true 20.0000 / -15.0000)\n", rpy[0], rpy[1]);
    check(fabsf(rpy[0]-20.0f)<0.01f && fabsf(rpy[1]+15.0f)<0.01f, "FIRST sample already at the true tilt");

    printf("\n3. static tilt held indefinitely\n");
    for(int i=0;i<2000;i++) att_step(g3,a3,10.0f,rpy);
    check(fabsf(rpy[0]-20.0f)<0.01f && fabsf(rpy[1]+15.0f)<0.01f, "still there after 20 s");

    printf("\n4. gyro bias rejection -- the accelerometer must pin roll despite a lying gyro\n");
    att_init(&cfg); grav_body(0,0,a3);
    float gbias[3]={2.0f,0,0};   /* 2 deg/s of pure lie on the roll axis */
    for(int i=0;i<3000;i++) att_step(gbias,a3,10.0f,rpy);
    printf("     roll after 30 s of a 2 deg/s bias: %.4f deg (unfiltered integral = 60.0000)\n", rpy[0]);
    check(fabsf(rpy[0])<2.0f, "bounded, not the 60 deg the raw integral gives");

    printf("\n5. the bias field actually removes it\n");
    cfg = base_cfg(); cfg.gyro_bias_dps[0]=2.0f;
    att_init(&cfg); grav_body(0,0,a3);
    for(int i=0;i<3000;i++) att_step(gbias,a3,10.0f,rpy);
    check(fabsf(rpy[0])<1e-3f, "declared bias subtracted, roll returns to zero");

    printf("\n6. accelerometer gate: a footfall transient must not read as tilt\n");
    cfg = base_cfg(); att_init(&cfg);
    grav_body(0,0,a3);
    for(int i=0;i<500;i++) att_step(g3,a3,10.0f,rpy);
    float shock[3]={0.0f, 6.0f, G};    /* norm 11.5, 1.7 m/s^2 outside the gate */
    float worst=0.0f;
    for(int i=0;i<50;i++){ att_step(g3,shock,10.0f,rpy); if(fabsf(rpy[0])>worst) worst=fabsf(rpy[0]); }
    printf("     worst roll during 0.5 s of lateral shock: %.4f deg\n", worst);
    check(worst < 0.01f, "gate held; shock rejected");

    printf("\n   negative control: the same shock with the gate opened wide\n");
    cfg = base_cfg(); cfg.accel_gate_mps2 = 100.0f; att_init(&cfg);
    grav_body(0,0,a3);
    for(int i=0;i<500;i++) att_step(g3,a3,10.0f,rpy);
    float worst_open=0.0f;
    for(int i=0;i<50;i++){ att_step(g3,shock,10.0f,rpy); if(fabsf(rpy[0])>worst_open) worst_open=fabsf(rpy[0]); }
    printf("     worst roll with the gate disabled: %.4f deg\n", worst_open);
    check(worst_open > 1.0f, "gate has power -- disabling it lets the shock through");

    printf("\n7. Euler kinematics vs the naive sum, at tilt\n");
    /* roll 45, pitch 45, gyro purely about body z. The naive form puts all of
     * it in yaw; the true kinematics couple it into roll and pitch. */
    cfg = base_cfg(); cfg.tau_s = 1e9f;   /* accelerometer off: pure propagation */
    att_init(&cfg); grav_body(45.0f,45.0f,a3);
    att_step(g3,a3,10.0f,rpy);
    float gz_only[3]={0,0,10.0f};
    for(int i=0;i<100;i++) att_step(gz_only,a3,10.0f,rpy);   /* 1 s at 10 deg/s */
    printf("     after 1 s of 10 deg/s about body z from (45,45):\n");
    printf("     roll %.4f  pitch %.4f  yaw %.4f\n", rpy[0], rpy[1], rpy[2]);
    check(fabsf(rpy[0]-45.0f)>0.5f, "roll moved -- the cross-coupling term is present");
    check(fabsf(rpy[2]-10.0f)>0.5f, "yaw is NOT a bare integral of gz at tilt");

    printf("\n8. gimbal region: pitch near 90 must not produce NaN\n");
    cfg = base_cfg(); att_init(&cfg);
    grav_body(0.0f,89.9f,a3);
    att_step(g3,a3,10.0f,rpy);
    float spin[3]={0,0,50.0f};
    int nan_seen=0;
    for(int i=0;i<500;i++){ att_step(spin,a3,10.0f,rpy); if(!finite3(rpy)) nan_seen=1; }
    check(!nan_seen, "500 frames at pitch 89.9 with 50 deg/s yaw rate, no NaN");

    printf("\n9. yaw drifts and the header says so -- this is a PROPERTY, not a defect\n");
    cfg = base_cfg(); att_init(&cfg); grav_body(0,0,a3);
    float yaw_bias[3]={0,0,1.0f};
    for(int i=0;i<6000;i++) att_step(yaw_bias,a3,10.0f,rpy);
    printf("     yaw after 60 s of an undeclared 1 deg/s bias: %.4f deg\n", rpy[2]);
    check(fabsf(rpy[2])>30.0f, "yaw drifted, as documented -- no reference exists to stop it");
    check(fabsf(rpy[0])<1.0f && fabsf(rpy[1])<1.0f, "roll and pitch did NOT drift -- gravity holds them");

    printf("\n10. degenerate inputs\n");
    cfg = base_cfg(); att_init(&cfg);
    float zero_a[3]={0,0,0};
    att_step(g3,zero_a,10.0f,rpy);  check(finite3(rpy), "zero accelerometer vector survives");
    grav_body(0,0,a3);
    att_step(g3,a3,0.0f,rpy);       check(finite3(rpy), "dt = 0 survives");
    att_step(g3,a3,-5.0f,rpy);      check(finite3(rpy), "negative dt survives");

    printf("\n%s  (%d failures)\n", fails?"SOME CHECKS FAILED":"ALL CHECKS PASSED", fails);
    return fails?1:0;
}
