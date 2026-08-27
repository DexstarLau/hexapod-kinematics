# hexapod-kinematics

[![CI](https://github.com/DexstarLau/hexapod-kinematics/actions/workflows/ci.yml/badge.svg)](https://github.com/DexstarLau/hexapod-kinematics/actions/workflows/ci.yml)
![Python](https://img.shields.io/badge/python-3.12%20%7C%203.14-blue)
![Tests](https://img.shields.io/badge/tests-89%20passing-brightgreen)

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
| Constant table + loader | working |
| Torque invariant, tripod share | working, margin ruled at 2.5000 |
| Repository, build, CI | working |
| Derivation and the fourteen sweep outputs | working, checked against externally supplied figures |
| Hard-coded-constant guard | **executing** — 8 constants, each with a reasoned output footprint |
| Corner-leg yaw guard | **executing** — catches a radial leg model on four legs of six |
| The three swing guards | **executing** across 9 (stride, duty) points |
| `ik_core` / `gait_core` headers | received — `core/include/` |
| `ik_core.c`, `hex_config.c` | due 27 Aug — algorithm workstream |
| `gait_core` | due 30 Sep — algorithm workstream |
| Python bindings to the C source | **not written** — next |
| Visualiser | **not written** |

**89 tests passing, 0 skipped.**

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

The leg geometry stopped being guesswork on 24 August: `L1 = 42.0000`,
`L2 = 74.2000`, `L3 = 112.6231`, the six coxa positions and the six `beta_mount`
yaws all come from the manufacturer's CAD model and carry status `measured`.

Currently surrogate: `theta3_deg`, `dtheta_peak_deg_s`. `Constants.stamp()`
reports which of them fed any given result, so no output built on a surrogate can
be presented as a measurement.

Currently blocked: `servo_speed_loaded_deg_s`, `stale_ramp_ms`, `swing_eps_mm_s`.

---

## Run it

```powershell
pip install -r requirements.txt
python -m pytest tests/ -v
python -m sim.emit_constants_table
```

Tested on Python 3.14 with pytest 9. `pyproject.toml` pins the module search
path so behaviour does not depend on the pytest version.

**89 passing, 0 skipped.**

---

## Layout

```
core/include/  ik_core.h, hex_config.h — the frozen API. Algorithm workstream owns this
core/          C99 sources. Algorithm workstream owns this. Do not edit.
bindings/    Python bindings to the same C source — not a reimplementation
sim/         Constant loader, sweep runner, visualiser
tests/       Unit tests and the guards
docs/        Derivations, generated constant table
config/      hexapod.json — the single source of truth
```

---

## Licence

Not yet chosen.


---

## Two computation paths, deliberately

`config/hexapod.json` has two consumers and they are not interchangeable:

```
config/hexapod.json
   |
   +-- sim/constants.py  --> double --> derivation, the fourteen outputs, emitted tables
   |
   +-- the caller        --> hex_config_t (float) --> ik_core, gait_core at 50 Hz
```

`hex_config_t` is `float` because the frozen API is `float` and the target is an
ESP32-S3. Single precision does not reliably carry the four decimal places the
precision convention requires on a 100 mm quantity, so the analysis path reads the
JSON in double and never goes through C.

That separation is only safe if the two are checked against each other, which is
what `tests/test_c_agreement.py` will do once `hex_config.c` lands.

Neither path hard-codes anything. The cores ship no default configuration and
there is no `hex_config_default()` — a core with no defaults cannot run on a stale
constant, because it cannot run at all without being told the constants.
