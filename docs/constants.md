# Constant table

**Generated from `config/hexapod.json` by `sim/emit_constants_table.py`. Do not edit by hand.**

Platform: Yeahbot hexapod (assembled kit), PROJECT_04 §1  
Precision: D147 + D220 - double precision throughout. NO INTERMEDIATE QUANTITY IS ROUNDED BEFORE IT IS MULTIPLIED (D211: the 2x amplifier on theta_extreme has produced three errors in one quantity). Angles, lengths and rates all to 4 dp. D147 binds emitted tables and analysis, NOT the 50 Hz path.  
Last updated: 2026-08-21

## Surrogate constants

**Nothing below has been measured.** These numbers exist so that code can
run. They carry no claim about the physical machine, and any result derived
from one is stamped by `Constants.stamp()`.

- **`dtheta_peak_deg_s`** = 375.0000 deg/s - PROJECT_04 §2 no-load figure, D62
- **`theta_3_deg`** = -30.0000 deg - D160, PROJECT_02 §2, §3

## Blocked constants

These have no authoritative value. Reading one raises `ConstantError`.
No code may proceed past them with a guessed number.

- **`servo_speed_loaded_deg_s`** (deg/s) - PROJECT_04 §2 - to be measured by the hardware workstream.
- **`stale_ramp_ms`** (ms) - hex_config.h, D144 - needed by gait_core, due 30 Sep
- **`swing_eps_mm_s`** (mm/s) - hex_config.h, D144 - needed by gait_core, due 30 Sep

## All constants

| Constant | Value | Unit | Status | Source |
|---|---|---|---|---|
| `beta_mount_deg` | R1: -45.0; R2: -90.0; R3: -135.0; L1: 45.0; L2: 90.0; L3: 135.0 | deg | measured | D227, PROJECT_06 §3 |
| `beta_neutral_deg` | R1: 0.0; R2: 0.0; R3: 0.0; L1: 0.0; L2: 0.0; L3: 0.0 | deg | **PROVISIONAL** | D238, PROJECT_06 §3 |
| `body_bob_budget_mm` | 5.0000 | mm | decided | D11, D64 |
| `command_step_deg` | 0.1350 | deg | **PROVISIONAL** | D229, D230, D240, PROJECT_07 §2 |
| `controlled_dof_per_leg` | 2 | count | decided | D3 |
| `coxa_length_mm` | 42.0000 | mm | measured | D227, PROJECT_06 §3 |
| `coxa_positions_mm` | R1: [102.201, -62.9665]; R2: [0.0, -79.0]; R3: [-102.201, -62.9665]; L1: [102.201, 62.9665]; L2: [0.0, 79.0]; L3: [-102.201, 62.9665] | mm | measured | D227, PROJECT_06 §3 |
| `dtheta_peak_deg_s` | 375.0000 | deg/s | **SURROGATE** | PROJECT_04 §2 no-load figure, D62 |
| `duty_factor` | 0.5000 | ratio | decided | D146 |
| `femur_length_mm` | 74.2000 | mm | measured | D227, PROJECT_06 §3 |
| `joint_accuracy_deg` | 0.2400 | deg | **PROVISIONAL** | D230, D240, PROJECT_07 §2 |
| `joint_envelopes_deg` | coxa_front: {'left': [-47.25, 47.25], 'right': [-47.25, 47.25], 'span': 94.5}; coxa_middle: {'left': [-54.0, 13.5], 'right': [-13.5, 54.0], 'span': 67.5}; coxa_rear: {'left': [-47.25, 13.5], 'right': [-13.5, 47.25], 'span': 60.75}; femur_all: {'left': [-47.25, 94.5], 'right': [-94.5, 47.25], 'span': 141.75}; tibia_front_middle: {'left': [-67.5, 121.5], 'right': [-121.5, 67.5], 'span': 189.0}; tibia_rear: {'left': [-67.5, 135.0], 'right': [-135.0, 67.5], 'span': 202.5} | deg | **PROVISIONAL** | D247-D249, PROJECT_07 §7 |
| `joint_max_deg` | 135.0000 | deg | **PROVISIONAL** | D229, PROJECT_06 §3 |
| `joint_min_deg` | -135.0000 | deg | **PROVISIONAL** | D229, PROJECT_06 §3 |
| `leg_count` | 6 | count | decided | PROJECT_01 §5.1 |
| `members_per_leg` | 3 | count | decided | D160, PROJECT_02 §1 |
| `nominal_stride_mm` | 60.0000 | mm | decided | D27 |
| `pack_voltage_v` | 7.4000 | V | decided | PROJECT_04 §1 |
| `payload_mass_kg` | 2.1500 | kg | **PROVISIONAL** | D212, PROJECT_06 §3 |
| `pwm_centre_count` | 1500 | count | decided | D229, PROJECT_07 §4 |
| `pwm_count_max` | 2500 | count | decided | D229 |
| `pwm_count_min` | 500 | count | decided | D229 |
| `servo_count_actuated` | 12 | count | decided | PROJECT_02 §3 |
| `servo_count_bought` | 20 | count | decided | PROJECT_02 §3 |
| `servo_count_installed` | 18 | count | decided | PROJECT_02 §3, PROJECT_04 §1 |
| `servo_model` | ZX20D | text | decided | PROJECT_04 §1 |
| `servo_speed_loaded_deg_s` | - | deg/s | **BLOCKED** | PROJECT_04 §2 - to be measured by the hardware workstream. |
| `servo_speed_no_load_deg_s` | 375.0000 | deg/s | decided | PROJECT_04 §2 |
| `stale_ramp_ms` | - | ms | **BLOCKED** | hex_config.h, D144 - needed by gait_core, due 30 Sep |
| `stall_torque_kgcm` | 20.0000 | kg*cm | decided | D209, PROJECT_06 §3 |
| `swing_clearance_max_mm` | 20.0000 | mm | decided | D69 |
| `swing_clearance_mm` | 15.0000 | mm | decided | D69 |
| `swing_eps_mm_s` | - | mm/s | **BLOCKED** | hex_config.h, D144 - needed by gait_core, due 30 Sep |
| `swing_velocity_profile` | half_sine | text | decided | D145 |
| `theta_2_neutral_deg` | 40.0000 | deg | **PROVISIONAL** | D70, D71 |
| `theta_3_deg` | -30.0000 | deg | **SURROGATE** | D160, PROJECT_02 §2, §3 |
| `tibia_length_mm` | 112.6231 | mm | measured | D227, PROJECT_06 §3 |
| `torque_margin` | 2.5000 | ratio | decided | D209, PROJECT_05 |
| `tripod_support_legs` | 3 | count | decided | D208 |
| `update_rate_hz` | 50.0000 | Hz | decided | PROJECT_01 §5.1 |

## Notes

**`beta_mount_deg`** - Frame yaw of each coxa axis. All six axes parallel to 1.0000 deg.
THE CORNER-LEG TRAP: corner legs have position angle 31.6374 deg but beta_mount 45.0000 - a 13.3626 deg difference. A model assuming legs point radially outward from the body centre is wrong on four legs of six. The middle legs ARE radial (90.0000 vs 90.0000), so a single-leg validation on a middle leg passes and hides it. See the fifth guard.

**`beta_neutral_deg`** - INFERRED under D159. Commanded coxa yaw at which the leg lies along beta_mount_deg. The assembly datum is the servo centre and all six coxae read PWM 1500 = 0.0000 deg. beta_mount_deg and beta_neutral_deg stay separate fields and separate concepts (D23); D238 rules the value, not the distinction.

**`body_bob_budget_mm`** - Peak-to-peak. This, not servo torque, is what limits stride.

**`command_step_deg`** - The bus command grid. angle_deg = (pwm - 1500) * 0.1350; 2000 counts over 270 deg about a centre of 1500. PAPER, not measured - D190 measures it. Supersedes 0.3000 (D186) and the PCA9685 0.4392, which is void in every form (D185).

**`controlled_dof_per_leg`** - Coxa yaw + femur pitch actuated. The tibia member is installed and held at fixed theta_3_deg. v2 makes theta_3 a solve variable with no interface change (PROJECT_02 §2).

**`coxa_length_mm`** - From the D197 STEP model. D213/D214 closed. The old 50.0 seed was 19.0% wrong. There is also a -0.5000 mm vertical drop from the coxa axis to the femur axis, not modelled by the planar reduction and not represented in hex_config_t.

**`coxa_positions_mm`** - Body frame: X forward, Y left, Z up, origin at the coxa centroid. Model residual asymmetry +/-0.0300 mm, symmetrised. Corner legs sit at radius 120.0409 mm and position angle 31.6374 deg.

**`dtheta_peak_deg_s`** - Peak joint rate the swing profile is scaled to. The no-load figure standing in for a loaded one. Sweep outputs 12-14 scale linearly with this, so it is emitted alongside them (D62) and the ratios across duty are what survive, not the absolutes.

**`duty_factor`** - By decision, exposed as config.

**`joint_accuracy_deg`** - Datasheet accuracy - where the servo actually LANDS. THIS IS THE BINDING LIMIT, not the command grid: 0.2400 / 0.1350 = 1.7778 counts, so the grid never binds and a sweep modelling only the grid understates foot-position error by that factor. New field of hex_config_t, issued to the algorithm workstream in HANDOFF_40. PAPER - D190 measures it.

**`joint_envelopes_deg`** - Observed across all 394 vendor action groups in Lm2六足机器人动作组V3.ini. pwm_R = 3000 - pwm_L mirrors exactly on all eighteen channels, confirming the D228 pairing and 1500 as the shared centre. THE COXA ENVELOPES ARE NOT SYMMETRIC ABOUT NEUTRAL AND DIFFER BY LEG POSITION - that is D227's 13.3626 deg beta_mount offset showing up in vendor data, not noise.

**`joint_max_deg`** - Datasheet span, 0500-2500 counts. D246: bias is set per joint over +/-200 counts and the commanded value is clamped, so a bias of b counts costs |b| counts of usable travel at one end - usable_span_deg = 270.0000 - |bias| * 0.1350, i.e. 243.0000 deg at full bias. Real limits are set after the biases are measured, never from the datasheet span.
SCALAR FIELDS CANNOT CARRY D247's PER-JOINT ENVELOPES - see joint_envelopes_deg and FINDING_05.

**`members_per_leg`** - Coxa, femur, tibia. Mechanically three-segment, kinematically two-DOF.

**`pack_voltage_v`** - 5200 mAh pack.

**`payload_mass_kg`** - INFERRED from the vendor listing under D159 and unmeasured (D212). Measured at D190 step 3. The mass-ceiling framing is retired project-wide: mass is a measured input, not a ceiling to design against. The design variable is a_eff.

**`pwm_centre_count`** - angle_deg = (pwm - 1500) * command_step_deg. The vendor's field is named 'pwm' in #IndexPpwmTtime! and it is a POSITION COUNT, not a pulse width - PWM as a drive method is void (D185), the vendor's legacy field name is not. Documentation names it as the manual names it so the repository can be checked against the manual.

**`servo_count_bought`** - 2 spare.

**`servo_count_installed`** - 18 x ZX20D serial bus servos, 3 per leg. 12 actuated, 6 held at theta_3.

**`servo_model`** - Serial bus servo, 24-channel control board.

**`servo_speed_loaded_deg_s`** - MP2 deliverable. Any timing claim that needs a loaded speed must fail loudly until this exists.

**`servo_speed_no_load_deg_s`** - Vendor no-load figure. Supersedes the provisional 200 deg/s.

**`stall_torque_kgcm`** - At 7.4 V.

**`swing_clearance_max_mm`** - Exceeding this requires a re-sweep.

**`swing_velocity_profile`** - The peak/mean ratio of pi/2 is DELIBERATELY NOT STORED HERE. It is a consequence of the profile and must be computed. Storing it would be the exact failure the hard-coded-constant guard exists to catch.

**`theta_2_neutral_deg`** - Femur pitch at mid-stance. A solve variable with a floor, not a free choice. AT THE D227 MEASURED GEOMETRY THE 40.0000 SURROGATE VIOLATES D209: it puts a_eff at 167.7526 mm against a 111.6279 mm ceiling, 50.3% over. theta2_nom >= 70.0098 deg is needed at theta3 = -30, which stands at 142.1370 mm - inside PROJECT_02's 140-170 mm band. NOT CHANGED HERE: D209 rules the freed budget is not to be spent before the re-sweep. Reported as FINDING_06.

**`theta_3_deg`** - Fixed tibia angle, commanded not mechanical: D221 sets the six tibia servos once at initialisation, outside gait_core. So this arrives as a DECIDED value, not a measured one (COREDROP_01_HW). What gets measured afterwards is the residual, commanded against true shaft angle by external instrument.
THE SURROGATE IS DELIBERATELY NON-ZERO. Exact condition for psi == 0 (COREDROP_02 §1): L3 == 0, OR theta3 = 0 (mod 360), OR theta3 = 180 (mod 360) AND L3 < L2. Note theta3 = 180 with L3 > L2 gives psi = 180, and L2 == L3 there gives R == 0. A zero surrogate pins psi to zero and collapses the leg onto a two-member case in every test using this table.
TRAP: sin(radians(180.0)) is 1.2246e-16, not 0.0, so an equality test against zero passes at theta3 = 0 and fails at 180.

**`tibia_length_mm`** - Perpendicular distance from the tibia axis to the foot-pad contact point. Hard geometry; the pad is compliant rubber and this is not a contact model. Femur and tibia axes coplanar to +/-0.0855 mm, so D207's planar two-link reduction is exact in geometry as well as algebra.

**`torque_margin`** - margin_factor in hex_config.h. A DIVISOR ON DEMAND: tau_femur_peak_kgcm * margin_factor <= tau_servo_kgcm. Reverts automatically to 3.0000 if D190 measures loaded femur torque below 16 kg*cm at 7.4 V, or mass above 2.30 kg - no further ruling needed. a_eff_max is NOT stored: it is tau_servo*10*members/(mass*margin) and moves with all three. See tests/test_torque.py.

**`tripod_support_legs`** - Legs sharing the load at duty 0.50. Single-leg support needs five feet off the ground, which is a fault and not a transient, so the whole-mass torque product is retired as a constraint and kept only as the tau_femur_singleleg_kgcm diagnostic.
