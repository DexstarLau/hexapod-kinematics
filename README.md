# hexapod-kinematics

[![CI](https://github.com/DexstarLau/hexapod-kinematics/actions/workflows/ci.yml/badge.svg)](https://github.com/DexstarLau/hexapod-kinematics/actions/workflows/ci.yml)
![Python](https://img.shields.io/badge/python-3.12%20%7C%203.14-blue)
![Tests](https://img.shields.io/badge/tests-181%20passing-brightgreen)

Forward and inverse kinematics for a six-legged walking robot, with a Python
reference bound to the same C source and a test suite.

**The tripod gait engine and the visualiser are components 5 and 8 of this
project's scope and are not written yet.** They are listed in the status table
below rather than in this sentence, because a summary line that names them reads
as a claim that they exist.

MP1 of a twelve-project series running August 2026 to May 2028.

---

## Status — 10 September 2026

**MP1 is due 31 October 2026.** The repository is 21 days old measured from its
20 August start. Six of the eight scope components are present; the two that are
not are the largest single piece of remaining work and are owned by the algorithm
workstream, not by this repository's maintainer.

What is here:

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
| `ik_core.c`, `hex_config.c` | received 26 Aug — `core/src/` |
| `gait_core` | due 30 Sep — algorithm workstream |
| Python bindings to the C source | working — 26 fields, layout asserted both ways |
| Vendor pose set, structural check | working — `tools/vendor_poses.py`, [report](docs/vendor_pose_check.md) |
| Vendor pose set, FK residuals | working — [report](reports/D275_vendor_pose_validation.md), 2,364 + 1,704 rows, both CSVs regenerate byte-identical |
| Fold-boundary guard | **executing** — every boundary recomputed independently per `theta3` |
| Status-table guard | **executing** — `IK_STATUS` and `CFG_ERR` compared against the C enums |
| Gait engine | **not written** — scope component 5, algorithm workstream |
| Visualiser | **not written** — scope component 8 |
| `tests/test_c_agreement.py` | working — `hex_derive` and `ik_fk_leg` in `float` against `sim/derive.py` in double, 4,440 comparisons, worst 14.4% of the error budget |

**181 passing locally; 175 passing and 6 skipped on CI and on any machine without
the vendor action-group file.** The six are the vendor-pose tests. The file is the
manufacturer's, is not redistributable, and is therefore not on a runner — see
`docs/THIRD_PARTY.md`.

**The skips are named because a skip is CI green over something that never ran.**
Everything that can run without that file does run: the binding tests compile
`core/` with `-std=c99 -Wall -Wextra -pedantic` and **fail** rather than skip if no
compiler is found, and the two guards added on 9 September read the headers as text
so they need neither the vendor file nor a compiler.

**150 -> 163 -> 175 is twenty-five added tests, not a regression from 156.**

Nothing in this README claims work that has not been done. Where something is not
written, the table above says so by name.

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

# needs your own copy of the vendor action-group file; see docs/THIRD_PARTY.md
python -m tools.vendor_poses --actions "path/to/your/copy.ini"
```

Tested on Python 3.14 with pytest 9. `pyproject.toml` pins the module search
path so behaviour does not depend on the pytest version.

**143 passing, 0 skipped.**

---

## Layout

```
core/include/  ik_core.h, hex_config.h — algorithm workstream owns these. Not a
               frozen interface: the frozen list is gait_init, gait_step,
               gait_set_stale, att_init, att_step, and these headers are not on it
core/          C99 sources. Algorithm workstream owns this. Do not edit.
bindings/    Python bindings to the same C source — not a reimplementation
sim/         Constant loader, sweep runner, visualiser
tools/       Scripts that read data this repository does not ship
tests/       Unit tests and the guards
docs/        Derivations, generated constant table, third-party policy
config/      hexapod.json — the single source of truth
```

---

## Companion repository

MP2's camera work — printable targets, calibration board, collection protocol
and manifest checker — lives in
[`hexapod-vision`](https://github.com/DexstarLau/hexapod-vision). It was moved
out on 30 August: it needs `opencv`, this repository's test suite is standard
library only, and its 2000 collected frames must never enter a git history that
cannot delete them. Neither repository depends on the other.

---

## Line endings are pinned, and it is not a style preference

`.gitattributes` normalises text to LF everywhere and marks `*.pdf` and `*.png`
binary. Both halves are load-bearing, and CI found out the hard way:

- **A text file's raw bytes are not its content.** `docs/vendor_pose_check.md`
  records the SHA-256 of `config/hexapod.json` so a published result is traceable
  to the table that produced it. Checked out with CRLF that file hashes
  `2887e918`; with LF, `cae95c5c`. Not one character differs. The stamp is
  computed over LF-normalised bytes for this reason.
- **Git calls a file binary by looking for a NUL in its first 8000 bytes, and
  neither PDF in `print/` has one.** Both were classified as text and rewritten
  on a Windows checkout, which corrupts them. Anyone cloning on Windows would
  have got a broken calibration target and no indication of it. The local
  machine cannot see this; the Windows half of the CI matrix is what saw it.

`tests/test_vendor_poses.py::test_line_endings_are_pinned` fails if
`.gitattributes` is removed, so the next person does not rediscover this.

---

## Third-party material

The kit manufacturer's manuals, schematic, example source and action-group file
are **referenced, never redistributed, and never copied into source.** Nothing
in this repository is theirs.

`tools/vendor_poses.py` does read their action-group file, to check this
project's constant table against the poses the kit actually ships. It reads a
copy you supply with `--actions`; what is committed is the report — counts,
ranges, derived structure — plus a SHA-256 of the input so a published result
can be traced to the file that produced it without that file being published.

See [`docs/THIRD_PARTY.md`](docs/THIRD_PARTY.md).

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
what `tests/test_c_agreement.py` does. **It was written on 10 September, sixteen
days after `hex_config.c` landed**, and for those sixteen days the claim that the
two paths agreed was an assumption wearing a README sentence.

The tolerance is `8 * FLOAT32_EPS * (L1 + L2 + L3)` — a budget against the leg's
characteristic reach, not against each output's own magnitude. A purely relative
tolerance was tried first and is wrong here: near `r = 0` a small output carries
error accumulated on large intermediates, and measured against itself it looks
like a precision collapse that is not happening. **The tolerance is a proposal
under review by the algorithm workstream and may change.**

**It compares; it does not validate.** Two implementations of the same wrong
formula agree perfectly. Only measurement settles correctness.

Neither path hard-codes anything. The cores ship no default configuration and
there is no `hex_config_default()` — a core with no defaults cannot run on a stale
constant, because it cannot run at all without being told the constants.
