# Constant table

**Generated from `config/hexapod.json` by `sim/emit_constants_table.py`. Do not edit by hand.**

Platform: Yeahbot hexapod (assembled kit), PROJECT_04 §1  
Precision: D147 - double precision throughout, no intermediate rounding. Angles 4 dp in degrees, lengths 4 dp in millimetres.  
Last updated: 2026-08-21

## Surrogate constants

**Nothing below has been measured.** These numbers exist so that code can
run. They carry no claim about the physical machine, and any result derived
from one is stamped by `Constants.stamp()`.

- **`coxa_length_mm`** = 50.0000 mm - PROJECT_02 §3 (superseded D34/D49/D71)
- **`femur_length_mm`** = 90.0000 mm - PROJECT_02 §3 (superseded D34/D49/D71)
- **`payload_mass_kg`** = 2.1500 kg - PROJECT_04 §2
- **`theta_3_deg`** = 0.0000 deg - D160, PROJECT_02 §2, §3
- **`tibia_length_mm`** = 90.0000 mm - D160, PROJECT_02 §3

## Blocked constants

These have no authoritative value. Reading one raises `ConstantError`.
No code may proceed past them with a guessed number.

- **`servo_speed_loaded_deg_s`** (deg/s) - PROJECT_04 §2 - to be measured by the hardware workstream.
- **`torque_margin`** (ratio) - Coordination ruling required.

## All constants

| Constant | Value | Unit | Status | Source |
|---|---|---|---|---|
| `body_bob_budget_mm` | 5.0000 | mm | decided | D11, D64 |
| `command_step_deg` | 0.3000 | deg | decided | PROJECT_04 §2 |
| `controlled_dof_per_leg` | 2 | count | decided | D3 |
| `coxa_length_mm` | 50.0000 | mm | **SURROGATE** | PROJECT_02 §3 (superseded D34/D49/D71) |
| `coxa_positions_mm` | R1: [100.0, -56.57]; R2: [0.0, -80.0]; R3: [-100.0, -56.57]; L1: [100.0, 56.57]; L2: [0.0, 80.0]; L3: [-100.0, 56.57] | mm | **PROVISIONAL** | D24, D25 |
| `duty_factor` | 0.5000 | ratio | decided | D146 |
| `femur_length_mm` | 90.0000 | mm | **SURROGATE** | PROJECT_02 §3 (superseded D34/D49/D71) |
| `leg_count` | 6 | count | decided | PROJECT_01 §5.1 |
| `leg_neutral_direction_deg` | R1: -90.0; R2: -90.0; R3: -90.0; L1: 90.0; L2: 90.0; L3: 90.0 | deg | decided | D15 |
| `members_per_leg` | 3 | count | decided | D160, PROJECT_02 §1 |
| `nominal_stride_mm` | 60.0000 | mm | decided | D27 |
| `pack_voltage_v` | 7.4000 | V | decided | PROJECT_04 §1 |
| `payload_mass_kg` | 2.1500 | kg | **SURROGATE** | PROJECT_04 §2 |
| `servo_count_actuated` | 12 | count | decided | PROJECT_02 §3 |
| `servo_count_bought` | 20 | count | decided | PROJECT_02 §3 |
| `servo_count_installed` | 18 | count | decided | PROJECT_02 §3, PROJECT_04 §1 |
| `servo_model` | ZX20D | text | decided | PROJECT_04 §1 |
| `servo_speed_loaded_deg_s` | - | deg/s | **BLOCKED** | PROJECT_04 §2 - to be measured by the hardware workstream. |
| `servo_speed_no_load_deg_s` | 375.0000 | deg/s | decided | PROJECT_04 §2 |
| `stall_torque_kgcm` | 20.0000 | kg*cm | decided | PROJECT_04 §2 |
| `swing_clearance_max_mm` | 20.0000 | mm | decided | D69 |
| `swing_clearance_mm` | 15.0000 | mm | decided | D69 |
| `swing_velocity_profile` | half_sine | text | decided | D145 |
| `theta_2_neutral_deg` | 40.0000 | deg | **PROVISIONAL** | D70, D71 |
| `theta_3_deg` | 0.0000 | deg | **SURROGATE** | D160, PROJECT_02 §2, §3 |
| `tibia_length_mm` | 90.0000 | mm | **SURROGATE** | D160, PROJECT_02 §3 |
| `torque_margin` | - | ratio | **BLOCKED** | Coordination ruling required. |
| `update_rate_hz` | 50.0000 | Hz | decided | PROJECT_01 §5.1 |

## Notes

**`body_bob_budget_mm`** - Peak-to-peak. This, not servo torque, is what limits stride.

**`command_step_deg`** - Per bus command. Supersedes 0.4392 deg per PCA9685 count. Third member of the hard-coded-constant guard (PROJECT_04 §4).

**`controlled_dof_per_leg`** - Coxa yaw + femur pitch actuated. The tibia member is installed and held at fixed theta_3_deg. v2 makes theta_3 a solve variable with no interface change (PROJECT_02 §2).

**`coxa_length_mm`** - L1. PROJECT_02 §3 declares L2 and L3 unknown until measured but is silent on L1. QUESTION RAISED WITH COORDINATION: is L1 also unknown? Treated as surrogate until ruled.

**`coxa_positions_mm`** - SUPERSEDED IN PRINCIPLE. PROJECT_04 §2 states this table was derived for a body plate that no longer exists. Awaiting measurement of the Yeahbot frame. Two open questions raised with coordination: (a) 56.57 is given to 2 dp but D147 requires 4 dp; if the intent is 80/sqrt(2) the value is 56.5685. (b) whether the Yeahbot frame is even hexagonal in this arrangement.

**`duty_factor`** - By decision, exposed as config.

**`femur_length_mm`** - L2. PROJECT_02 §3: frame's femur member, unknown until measured. The old 90.0 mm was derived for a frame that no longer exists; it survives here only as a number that lets code run.

**`leg_neutral_direction_deg`** - Lateral neutral. beta_mount and beta_neutral stay separate (D23) - this table is beta_neutral only.

**`members_per_leg`** - Coxa, femur, tibia. Mechanically three-segment, kinematically two-DOF.

**`pack_voltage_v`** - 5200 mAh pack.

**`payload_mass_kg`** - Unweighed. The mass PROJECT_04 §2's a_eff figure was quoted at.

**`servo_count_bought`** - 2 spare.

**`servo_count_installed`** - 18 x ZX20D serial bus servos, 3 per leg. 12 actuated, 6 held at theta_3.

**`servo_model`** - Serial bus servo, 24-channel control board.

**`servo_speed_loaded_deg_s`** - MP2 deliverable. Any timing claim that needs a loaded speed must fail loudly until this exists.

**`servo_speed_no_load_deg_s`** - Vendor no-load figure. Supersedes the provisional 200 deg/s.

**`stall_torque_kgcm`** - At 7.4 V.

**`swing_clearance_max_mm`** - Exceeding this requires a re-sweep.

**`swing_velocity_profile`** - The peak/mean ratio of pi/2 is DELIBERATELY NOT STORED HERE. It is a consequence of the profile and must be computed. Storing it would be the exact failure the hard-coded-constant guard exists to catch.

**`theta_2_neutral_deg`** - Femur pitch at mid-stance. A solve variable with a floor, not a free choice.

**`theta_3_deg`** - Fixed tibia angle. PROJECT_02 §2 requires this to be a configuration field with a default, NOT an absent joint and NOT a hard-coded zero. The 0.0 here is a surrogate default awaiting measurement; it is a config value that the guard mutates, which is exactly what '#define THETA3 0' would not be.

**`tibia_length_mm`** - L3. Frame's tibia member, unknown until measured. Surrogate equal to the femur surrogate purely so that the two are visibly arbitrary and neither looks measured.

**`torque_margin`** - QUESTION RAISED: PROJECT_04 §2's figures (2.15 kg x 93.0 mm = 19.9950 kg*cm against a 20.0 kg*cm stall) imply a margin of 0.99975, i.e. none. A servo cannot run continuously at stall. The torque invariant is stall_torque_kgcm * torque_margin and a_eff_max is that divided by mass - both computed, never stored.
