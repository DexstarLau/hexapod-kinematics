# Constant table

**Generated from `config/hexapod.json` by `sim/emit_constants_table.py`. Do not edit by hand.**

Platform: Yeahbot hexapod (assembled kit), PROJECT_04 §1  
Precision: D147 - double precision throughout, no intermediate rounding. Angles 4 dp in degrees, lengths 4 dp in millimetres.  
Last updated: 2026-08-21

## Blocked constants

These have no authoritative value. Reading one raises `ConstantError`.
No code may proceed past them with a guessed number.

- **`servo_speed_loaded_deg_s`** (deg/s) - PROJECT_04 §2 - to be measured by the hardware workstream.
- **`theta_3_deg`** (deg) - D199, via PROJECT_04 §3; default value defined in PROJECT_02, not received.
- **`tibia_length_mm`** (mm) - PROJECT_04 §3 refers to L3; the defining document is PROJECT_02, which MP-WS has not received.

## All constants

| Constant | Value | Unit | Status | Source |
|---|---|---|---|---|
| `a_eff_max_mm` | 93.0000 | mm | decided | PROJECT_04 §2 |
| `body_bob_budget_mm` | 5.0000 | mm | decided | D11, D64 |
| `command_step_deg` | 0.3000 | deg | decided | PROJECT_04 §2 |
| `controlled_dof_per_leg` | 2 | count | decided | D3 |
| `coxa_length_mm` | 50.0000 | mm | **PROVISIONAL** | D34, D71 |
| `coxa_positions_mm` | R1: [100.0, -56.57]; R2: [0.0, -80.0]; R3: [-100.0, -56.57]; L1: [100.0, 56.57]; L2: [0.0, 80.0]; L3: [-100.0, 56.57] | mm | **PROVISIONAL** | D24, D25 |
| `duty_factor` | 0.5000 | ratio | decided | D146 |
| `femur_length_mm` | 90.0000 | mm | **PROVISIONAL** | D34, D49, D71 |
| `leg_count` | 6 | count | decided | PROJECT_01 §5.1 |
| `leg_neutral_direction_deg` | R1: -90.0; R2: -90.0; R3: -90.0; L1: 90.0; L2: 90.0; L3: 90.0 | deg | decided | D15 |
| `nominal_stride_mm` | 60.0000 | mm | decided | D27 |
| `pack_voltage_v` | 7.4000 | V | decided | PROJECT_04 §1 |
| `payload_mass_kg` | 2.1500 | kg | **PROVISIONAL** | PROJECT_04 §2 |
| `servo_count_installed` | 18 | count | decided | PROJECT_04 §1 |
| `servo_model` | ZX20D | text | decided | PROJECT_04 §1 |
| `servo_speed_loaded_deg_s` | - | deg/s | **BLOCKED** | PROJECT_04 §2 - to be measured by the hardware workstream. |
| `servo_speed_no_load_deg_s` | 375.0000 | deg/s | decided | PROJECT_04 §2 |
| `stall_torque_kgcm` | 20.0000 | kg*cm | decided | PROJECT_04 §2 |
| `swing_clearance_max_mm` | 20.0000 | mm | decided | D69 |
| `swing_clearance_mm` | 15.0000 | mm | decided | D69 |
| `swing_velocity_profile` | half_sine | text | decided | D145 |
| `theta_2_neutral_deg` | 40.0000 | deg | **PROVISIONAL** | D70, D71 |
| `theta_3_deg` | - | deg | **BLOCKED** | D199, via PROJECT_04 §3; default value defined in PROJECT_02, not received. |
| `tibia_length_mm` | - | mm | **BLOCKED** | PROJECT_04 §3 refers to L3; the defining document is PROJECT_02, which MP-WS has not received. |
| `update_rate_hz` | 50.0000 | Hz | decided | PROJECT_01 §5.1 |

## Notes

**`a_eff_max_mm`** - Effective moment arm ceiling at 2.15 kg, derived from the 20 kg*cm stall torque.

**`body_bob_budget_mm`** - Peak-to-peak. This, not servo torque, is what limits stride.

**`command_step_deg`** - Per bus command. Supersedes 0.4392 deg per PCA9685 count. Third member of the hard-coded-constant guard (PROJECT_04 §4).

**`controlled_dof_per_leg`** - Coxa yaw + femur pitch. The tibia servo exists on this platform but is held at a constant commanded angle theta_3_deg (D199).

**`coxa_length_mm`** - Named L1 in PROJECT_01 §5.1. Renamed here to avoid collision with leg identifier 'L1'. Derived for the pre-PROJECT_04 frame; awaiting measurement of the Yeahbot frame.

**`coxa_positions_mm`** - SUPERSEDED IN PRINCIPLE. PROJECT_04 §2 states this table was derived for a body plate that no longer exists. Awaiting measurement of the Yeahbot frame. Two open questions raised with coordination: (a) 56.57 is given to 2 dp but D147 requires 4 dp; if the intent is 80/sqrt(2) the value is 56.5685. (b) whether the Yeahbot frame is even hexagonal in this arrangement.

**`duty_factor`** - By decision, exposed as config.

**`femur_length_mm`** - Named L2 in PROJECT_01 §5.1. Ceiling 97.6 mm. Derived for the pre-PROJECT_04 frame; awaiting measurement.

**`leg_neutral_direction_deg`** - Lateral neutral. beta_mount and beta_neutral stay separate (D23) - this table is beta_neutral only.

**`pack_voltage_v`** - 5200 mAh pack.

**`payload_mass_kg`** - The mass the a_eff_max_mm figure was taken at. Unweighed.

**`servo_count_installed`** - Supersedes the 12x DS3225MG of D48. Platform ships 18x ZX20D serial bus servos, 3 per leg.

**`servo_model`** - Serial bus servo, 24-channel control board.

**`servo_speed_loaded_deg_s`** - MP2 deliverable. Any timing claim that needs a loaded speed must fail loudly until this exists.

**`servo_speed_no_load_deg_s`** - Vendor no-load figure. Supersedes the provisional 200 deg/s.

**`stall_torque_kgcm`** - At 7.4 V.

**`swing_clearance_max_mm`** - Exceeding this requires a re-sweep.

**`swing_velocity_profile`** - The peak/mean ratio of pi/2 is DELIBERATELY NOT STORED HERE. It is a consequence of the profile and must be computed. Storing it would be the exact failure the hard-coded-constant guard exists to catch.

**`theta_2_neutral_deg`** - Femur pitch at mid-stance. A solve variable with a floor, not a free choice.

**`theta_3_deg`** - BLOCKER. Configuration parameter with a fixed default. v1 -> v2 is a config change, no rewiring.

**`tibia_length_mm`** - BLOCKER. PROJECT_01 §5.1 modelled a rigid tibia with no L3. PROJECT_02 introduced one. No value may be invented here.
