# Constant table

**Generated from `config/hexapod.json` by `sim/emit_constants_table.py`. Do not edit by hand.**

Platform: Yeahbot hexapod (assembled kit), PROJECT_04 §1  
Precision: D147 + D220 - double precision throughout. NO INTERMEDIATE QUANTITY IS ROUNDED BEFORE IT IS MULTIPLIED (D211: the 2x amplifier on theta_extreme has produced three errors in one quantity). Angles, lengths and rates all to 4 dp. D147 binds emitted tables and analysis, NOT the 50 Hz path. CROSS-PRECISION COMPARISON: hex_config_t and hex_derived_t are float. Comparing a double result against a float one at a fixed number of decimal places is invalid at ANY number of places - the rate falls and never reaches zero. Agreement is judged in units in the last place against a per-field scale, K = 32 (COREDROP_04 §4.4), excluding theta3 == 180 mod 360 where psi has a 360 deg branch cut.  
Last updated: 2026-09-05

## Surrogate constants

**Nothing below has been measured.** These numbers exist so that code can
run. They carry no claim about the physical machine, and any result derived
from one is stamped by `Constants.stamp()`.

- **`theta3_deg`** = -30.0000 deg - D160, PROJECT_02 §2, §3

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
| `dtheta_peak_deg_s` | 250.0000 | deg/s | **DISPUTED** | D324, amending D186 and D265 in status |
| `duty_factor` | 0.5000 | ratio | decided | D146 |
| `femur_length_mm` | 74.2000 | mm | measured | D227, PROJECT_06 §3 |
| `joint_accuracy_deg` | 1.0000 | deg | **PROVISIONAL** | D325, amending the value D240 makes binding |
| `joint_envelopes_deg` | coxa_front: {'left': [-47.25, 47.25], 'right': [-47.25, 47.25], 'span': 94.5}; coxa_middle: {'left': [-54.0, 13.5], 'right': [-13.5, 54.0], 'span': 67.5}; coxa_rear: {'left': [-47.25, 13.5], 'right': [-13.5, 47.25], 'span': 60.75}; femur_all: {'left': [-47.25, 94.5], 'right': [-94.5, 47.25], 'span': 141.75}; tibia_front_middle: {'left': [-67.5, 121.5], 'right': [-121.5, 67.5], 'span': 189.0}; tibia_rear: {'left': [-67.5, 135.0], 'right': [-135.0, 67.5], 'span': 202.5} | deg | **PROVISIONAL** | D247-D249, PROJECT_07 §7 |
| `leg_count` | 6 | count | decided | PROJECT_01 §5.1 |
| `margin_factor` | 2.5000 | ratio | decided | D209, PROJECT_05 |
| `mass_kg` | 2.1500 | kg | **PROVISIONAL** | D212, PROJECT_06 §3 |
| `members_per_leg` | 3 | count | decided | D160, PROJECT_02 §1 |
| `pack_voltage_v` | 7.4000 | V | decided | PROJECT_04 §1 |
| `pwm_centre_count` | 1500 | count | decided | D229, PROJECT_07 §4 |
| `pwm_count_max` | 2500 | count | decided | D229 |
| `pwm_count_min` | 500 | count | decided | D229 |
| `servo_count_actuated` | 12 | count | decided | PROJECT_02 §3 |
| `servo_count_bought` | 20 | count | decided | PROJECT_02 §3 |
| `servo_count_installed` | 18 | count | decided | PROJECT_02 §3, PROJECT_04 §1 |
| `servo_model` | ZX20D | text | decided | PROJECT_04 §1 |
| `servo_speed_loaded_deg_s` | - | deg/s | **BLOCKED** | PROJECT_04 §2 - to be measured by the hardware workstream. |
| `servo_speed_no_load_deg_s` | 375.0000 | deg/s | **DISPUTED** | PROJECT_04 §2, status amended by D324 clause 3 |
| `stale_ramp_ms` | - | ms | **BLOCKED** | hex_config.h, D144 - needed by gait_core, due 30 Sep |
| `stride_mm` | 60.0000 | mm | decided | D27 |
| `swing_clearance_max_mm` | 20.0000 | mm | decided | D69 |
| `swing_clearance_mm` | 15.0000 | mm | decided | D69 |
| `swing_eps_mm_s` | - | mm/s | **BLOCKED** | hex_config.h, D144 - needed by gait_core, due 30 Sep |
| `swing_velocity_profile` | half_sine | text | decided | D145 |
| `tau_servo_kgcm` | 20.0000 | kg*cm | decided | D209, PROJECT_06 §3 |
| `theta2_nom_deg` | 40.0000 | deg | **PROVISIONAL** | D70, D71 |
| `theta3_deg` | -30.0000 | deg | **SURROGATE** | D160, PROJECT_02 §2, §3 |
| `tibia_length_mm` | 112.6231 | mm | measured | D227, PROJECT_06 §3 |
| `tripod_support_legs` | 3 | count | decided | D208 |
| `update_rate_hz` | 50.0000 | Hz | decided | PROJECT_01 §5.1 |

## Notes

**`beta_mount_deg`** - Frame yaw of each coxa axis. All six axes parallel to 1.0000 deg.
THE CORNER-LEG TRAP: corner legs have position angle 31.6374 deg but beta_mount 45.0000 - a 13.3626 deg difference. A model assuming legs point radially outward from the body centre is wrong on four legs of six. The middle legs ARE radial (90.0000 vs 90.0000), so a single-leg validation on a middle leg passes and hides it. See the fifth guard.

**`beta_neutral_deg`** - INFERRED under D159. Commanded coxa yaw at which the leg lies along beta_mount_deg. The assembly datum is the servo centre and all six coxae read PWM 1500 = 0.0000 deg. beta_mount_deg and beta_neutral_deg stay separate fields and separate concepts (D23); D238 rules the value, not the distinction.

**`body_bob_budget_mm`** - Peak-to-peak. This, not servo torque, is what limits stride.

**`command_step_deg`** - The bus command grid, 270 deg over 2000 counts (D239/D240). NOT the binding limit - joint_accuracy_deg 1.0000 is 7.4074 counts wide, so the grid never binds (D325).

**`controlled_dof_per_leg`** - Coxa yaw + femur pitch actuated. The tibia member is installed and held at fixed theta_3_deg. v2 makes theta_3 a solve variable with no interface change (PROJECT_02 §2).

**`coxa_length_mm`** - From the D197 STEP model. D213/D214 closed. The old 50.0 seed was 19.0% wrong. There is also a -0.5000 mm vertical drop from the coxa axis to the femur axis, not modelled by the planar reduction and not represented in hex_config_t.

**`coxa_positions_mm`** - Body frame: X forward, Y left, Z up, origin at the coxa centroid. Model residual asymmetry +/-0.0300 mm, symmetrised. Corner legs sit at radius 120.0409 mm and position angle 31.6374 deg.

**`dtheta_peak_deg_s`** - A DISPUTED NOMINAL CEILING, not the ZX20D's measured speed (D324 clause 2). Two vendor sources disagree and only the faster carries a voltage qualifier: 使用手册 §2.1.1 p8 gives 转速 0.16 s/60 deg at 7.4 V = 375 deg/s, and 使用说明 slide 8 gives 响应速度 0.24 s/60 deg unqualified = 250 deg/s. D324 takes the slower, because over-planning fails SILENTLY as gait phase slip while under-planning only walks slower - the asymmetry is one-directional. 375.0 IS NOT VOID (D324 clause 3) and returns if measurement supports it; D186 and D265 move from INFERRED to DISPUTED. Sweep outputs 12-14 scale linearly with this, so EVERY BODY-SPEED OUTPUT IS AN UPPER BOUND AND MUST BE LABELLED ONE (D265). D190 measures the loaded figure; D324 clause 4 puts the no-load 60 deg step on the A-day work order. PROJECT_04 §2's three dead constants do NOT include this one - see _schema.void_documents.

**`duty_factor`** - By decision, exposed as config.

**`joint_accuracy_deg`** - Where the servo actually LANDS, and THIS IS THE BINDING LIMIT, not the command grid. BOTH figures are on the datasheet and D325 separates what D240 had merged: 理论精度 0.24 deg is the theoretical accuracy and is disputed; 综合实际使用精度 1 deg is the practical one and is what binds. 1.0000 / 0.1350 = 7.4074 counts, so the grid never binds and a sweep modelling only the grid understates foot-position error by that factor. The conclusion D240 drew is unchanged and stronger - the grid is 7.4074x finer, not 1.7778x. Stays provisional under D240 because D190 is already scheduled to replace it; a paper figure with a replacement scheduled is provisional by the schema's own definition. New field of hex_config_t, issued to the algorithm workstream in HANDOFF_40.

**`joint_envelopes_deg`** - VENDOR SERVO-COMMAND SPACE, NOT KINEMATIC SPACE. The two sides are mirrored: pwm_R = 3000 - pwm_L (D248), so angle_R = -angle_L for the same physical pose. Converting these into hex_config_t's joint_min_deg / joint_max_deg is a PER-JOINT sign map, never a per-leg one - coxa identity on all six legs, femur and tibia identity on the left three and negated on the right three (COREDROP_05 §3). theta1 is antisymmetric across the mirror and theta2/theta3 are symmetric, so the vendor mirror cancels for the coxa and does not cancel for the other two. Applying one sign to a whole leg silently reflects its coxa window. Source is D248 (the ID map and envelope table), not D247 (which locates the file). Per-joint command OFFSET is separate, is not assumed zero, and is hardware's (D262). Reset per joint once bias is measured (D246); bias costs |b| * 0.1350 deg at ONE end only.

**`margin_factor`** - margin_factor in hex_config.h. A DIVISOR ON DEMAND: tau_femur_peak_kgcm * margin_factor <= tau_servo_kgcm. Reverts automatically to 3.0000 if D190 measures loaded femur torque below 16 kg*cm at 7.4 V, or mass above 2.30 kg - no further ruling needed. a_eff_max is NOT stored: it is tau_servo*10*members/(mass*margin) and moves with all three. See tests/test_torque.py.

**`mass_kg`** - INFERRED from the vendor listing under D159 and unmeasured (D212). Measured at D190 step 3. The mass-ceiling framing is retired project-wide: mass is a measured input, not a ceiling to design against. The design variable is a_eff.

**`members_per_leg`** - Coxa, femur, tibia. Mechanically three-segment, kinematically two-DOF.

**`pack_voltage_v`** - 5200 mAh pack.

**`pwm_centre_count`** - angle_deg = (pwm - 1500) * command_step_deg. The vendor's field is named 'pwm' in #IndexPpwmTtime! and it is a POSITION COUNT, not a pulse width - PWM as a drive method is void (D185), the vendor's legacy field name is not. Documentation names it as the manual names it so the repository can be checked against the manual.

**`servo_count_bought`** - 2 spare.

**`servo_count_installed`** - 18 x ZX20D serial bus servos, 3 per leg. 12 actuated, 6 held at theta_3.

**`servo_model`** - Serial bus servo, 24-channel control board.

**`servo_speed_loaded_deg_s`** - MP2 deliverable. Any timing claim that needs a loaded speed must fail loudly until this exists.

**`servo_speed_no_load_deg_s`** - Vendor no-load figure, kept readable rather than voided (D324 clause 3). It is one of two vendor readings that disagree: 使用手册 §2.1.1 p8, 转速 0.16 s/60 deg AT 7.4 V, giving this 375.0; and 使用说明 slide 8, 响应速度 0.24 s/60 deg with NO voltage qualifier, giving 250.0. D324 planned on the slower. This entry is provenance for dtheta_peak_deg_s and is not a hex_config_t field. Supersedes the provisional 200 deg/s.

**`swing_clearance_max_mm`** - Exceeding this requires a re-sweep.

**`swing_velocity_profile`** - The peak/mean ratio of pi/2 is DELIBERATELY NOT STORED HERE. It is a consequence of the profile and must be computed. Storing it would be the exact failure the hard-coded-constant guard exists to catch.

**`tau_servo_kgcm`** - At 7.4 V.

**`theta2_nom_deg`** - Femur pitch at mid-stance. A solve variable with a floor (D70), not a free choice. AT D227's MEASURED GEOMETRY THE 40.0000 SURROGATE BREAKS TWO CONSTRAINTS, NOT ONE: (a) D209 torque - a_eff 167.7526 mm against a 111.6279 mm ceiling, 50.2784 % over; (b) D11/D64 bob budget - 5.5906 mm against body_bob_budget_mm 5.0, 111.81 % spent. theta2_nom >= 70.0098 deg is needed at theta3 = -30 (derived from the torque ceiling, NOT from 40.0000, so it is outside D260's quarantine). QUARANTINED (D260): no document may quote a body height, bob figure or stance derived from theta2_nom = 40.0000 until the D197 re-sweep. NOT CHANGED HERE - theta2_nom and theta3_deg move Theta0 together and setting one before the other is guessing twice. Reported as FINDING_06, ratified by D260.

**`theta3_deg`** - Fixed tibia angle, commanded not mechanical: D221 sets the six tibia servos once at initialisation, outside gait_core. So this arrives as a DECIDED value, not a measured one (COREDROP_01_HW). What gets measured afterwards is the residual, commanded against true shaft angle by external instrument.
THE SURROGATE IS DELIBERATELY NON-ZERO. Exact condition for psi == 0 (COREDROP_02 §1): L3 == 0, OR theta3 = 0 (mod 360), OR theta3 = 180 (mod 360) AND L3 < L2. Note theta3 = 180 with L3 > L2 gives psi = 180, and L2 == L3 there gives R == 0. A zero surrogate pins psi to zero and collapses the leg onto a two-member case in every test using this table.
TRAP: sin(radians(180.0)) is 1.2246e-16, not 0.0, so an equality test against zero passes at theta3 = 0 and fails at 180.

**`tibia_length_mm`** - Perpendicular distance from the tibia axis to the foot-pad contact point. Hard geometry; the pad is compliant rubber and this is not a contact model. Femur and tibia axes coplanar to +/-0.0855 mm, so D207's planar two-link reduction is exact in geometry as well as algebra.

**`tripod_support_legs`** - Legs sharing the load at duty 0.50. Single-leg support needs five feet off the ground, which is a fault and not a transient, so the whole-mass torque product is retired as a constraint and kept only as the tau_femur_singleleg_kgcm diagnostic.
