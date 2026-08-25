# hexapod-kinematics

Forward kinematics, inverse kinematics and a tripod gait engine for a six-legged
walking robot, with a Python reference bound to the same C source, a test suite,
and a visualiser.

MP1 of a twelve-project series running August 2026 to May 2028.

---

## Status — 21 August 2026

This repository is six days old measured against its first milestone and
contains, so far, only its foundation. What is here:

| Component | State |
|---|---|
| Constant table + loader | working, 12 tests |
| Repository, build, CI | working |
| `ik_core` (C99, FK/IK) | **not written** — algorithm workstream |
| `gait_core` (C99, tripod gait) | **not written** — algorithm workstream |
| Python bindings | **not written** |
| Sweep runner | **not written** |
| Visualiser | **not written** |

Nothing in this README claims work that has not been done.

---

## The machine

Six legs, two controlled degrees of freedom each: coxa yaw and femur pitch. The
platform is an assembled Yeahbot hexapod carrying eighteen ZX20D serial bus
servos — three per leg. The third servo, the tibia, is held at a constant
commanded angle, which is what makes the model two-DOF rather than three. Moving
to three controlled DOF later is a configuration change with no rewiring.

The kit supplies the body. This repository is the brain.

*A dimensioned diagram of the leg geometry belongs here and has not been drawn yet.*

---

## Constants

Every physical number lives in [`config/hexapod.json`](config/hexapod.json) and
reaches code only through `sim/constants.py`. Nothing is hard-coded anywhere.

The generated table is at [`docs/constants.md`](docs/constants.md).

Three constants moved in the three days before 21 August 2026, and a fourth —
the coxa position table — was invalidated outright when the platform changed.
Each carries a `status` field:

| Status | Meaning |
|---|---|
| `decided` | Fixed by a numbered decision or a vendor specification |
| `measured` | Produced by a measurement campaign, with an uncertainty |
| `provisional` | A stand-in for a measurement that has not happened |
| `unspecified` | No authoritative value exists — **reading it raises `ConstantError`** |

That last row is the important one. A constant with no value does not quietly
become `None` and does not fall back to a default. It stops the program and says
which document is missing.

Currently blocked: `tibia_length_mm`, `theta_3_deg`, `servo_speed_loaded_deg_s`.

---

## Run it

```powershell
pip install -r requirements.txt
python -m pytest tests/ -v
python -m sim.emit_constants_table
```

**12 tests passing.**

---

## Layout

```
core/        C99 — ik_core, gait_core. Algorithm workstream owns this. Do not edit.
bindings/    Python bindings to the same C source — not a reimplementation
sim/         Constant loader, sweep runner, visualiser
tests/       Unit tests and the guards
docs/        Derivations, generated constant table
config/      hexapod.json — the single source of truth
```

---

## Licence

Not yet chosen.
