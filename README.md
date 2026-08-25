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
| Constant table + loader | working, 18 tests |
| Repository, build, CI | working |
| Hard-coded-constant guard | wired, **not yet executing** — needs the sweep runner |
| `ik_core` (C99, FK/IK) | **not written** — algorithm workstream |
| `gait_core` (C99, tripod gait) | **not written** — algorithm workstream |
| Python bindings | **not written** |
| Sweep runner | **not written** |
| Visualiser | **not written** |

Nothing in this README claims work that has not been done.

---

## The machine

Six legs. Each leg is **mechanically three-segment and kinematically two-DOF**:
coxa yaw and femur pitch are actuated, and a tibia member is installed and held at
a fixed angle `theta_3`. Twelve servos of eighteen are actuated.

`theta_3` is a configuration field with a default — not an absent joint, and not a
hard-coded zero. Making it a solve variable in v2 is a configuration change with no
interface change and no rewiring.

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
| `surrogate` | **Nothing has been measured.** The number exists only so code can run |
| `unspecified` | No authoritative value exists — **reading it raises `ConstantError`** |

That last row is the important one. A constant with no value does not quietly
become `None` and does not fall back to a default. It stops the program and says
which document is missing.

Currently surrogate: `coxa_length_mm`, `femur_length_mm`, `tibia_length_mm`,
`theta_3_deg`, `payload_mass_kg`. Every one is a frame dimension nobody has
measured. `Constants.stamp()` reports which of them fed any given result, so no
output built on a surrogate can be presented as a measurement.

Currently blocked: `servo_speed_loaded_deg_s`, `torque_margin`.

---

## Run it

```powershell
pip install -r requirements.txt
python -m pytest tests/ -v
python -m sim.emit_constants_table
```

**18 passing, 4 skipped.** The skips are the hard-coded-constant guard,
which cannot run until the sweep runner exists. They are skips rather than passes
so that the suite never reports a guard as covered when it has not executed.

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
