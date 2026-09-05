# D275 — the vendor pose set validated at 2-DOF, theta3 fed per pose

**Table 1 tests `ik_core`'s geometry. It does not test a shipped configuration.**
Every one of the 394 poses is fed its own tibia angle, twelve distinct values
spanning `[-67.5000, +135.0000]`, and most are outside any envelope this project
would ship. `hex_derive` is called directly and `hex_config_validate` is **not**,
because validate would reject configurations the geometry is nonetheless required to
be correct on.

**Table 2 does test a shipped configuration:** the 284 uniform-tibia poses at the one
fixed `theta3_deg` the v1 config carries.

Run 3 September 2026. Reproduce with:

```
python -m tools.d275_fk_residuals <path to the vendor pose file>
```

---

## 1. Inputs

```
pose file    sha256 2d1e4365806d34aec696064731d3ac38af75fcc98102c3dc5a1a38f1c459ecc8
scope        [servo] group_liu_zu.  395 frames; G0000 is the all-1500 home frame
corpus       394 action poses                                              (D302)
geometry     L1 = 42.0000   L2 = 74.2000   L3 = 112.6231 mm    all `measured`
precision    single, throughout. The cores are float and use cosf/sinf/atan2f
flags        -std=c99 -Wall -Wextra -pedantic -O2 -fPIC -shared -ffp-contract=off
```

**The vendor file is referenced, never redistributed** (`docs/THIRD_PARTY.md`). The
test module skips when it is absent.

### 1.1 The predicate

D275's, and every figure below stands or falls with it: **the tibia angle of a right
leg is `(1500 - pwm) x 0.1350` and of a left leg `(pwm - 1500) x 0.1350`**, so both
sides land in one leg-local convention.

It generalises to the other two joints through the per-joint sign map on
`hex_config.h`'s envelope declarations, and **the generalisation is not uniform**:

```
coxa   theta1 : identity on all six legs
femur  theta2 : identity on the left three, negated on the right three
tibia  theta3 : identity on the left three, negated on the right three
```

The vendor mirrors the sides as `pwm_R = 3000 - pwm_L`. `theta1` is antisymmetric
between the sides and `theta2`/`theta3` are symmetric, so the vendor's uniform mirror
**cancels for the coxa and does not cancel for the other two.** Applying one sign to
a whole leg silently reflects its coxa window.

**Channel map:** `#000`–`#008` are the left legs L3, L2, L1, each triple running
tibia, femur, coxa; `#015`–`#023` are the right, running coxa, femur, tibia.

**The command offset (D262) is unmodelled and does not need to be.** It is the same
constant on the way in and on the way out, so it cancels in a round trip. What does
not cancel, and what this measures, is `R` and `psi`.

---

## 2. The four figures D275 publishes, reproduced

| | D275 | this run |
|---|---|---|
| distinct tibia angles | 12 | **12** |
| poses with six tibiae not all equal | 110 of 394 — 27.9188 % | **110 — 27.9188 %** |
| poses with six tibiae all equal | 284 of 394 — 72.0812 % | **284 — 72.0812 %** |
| six-leg theta3 intersection | `[-67.5000, +121.5000]` | **`[-67.5000, +121.5000]`** |
| poses outside it | 1 — G0117, rear pair at 135.0000 | **1 — G0117, L3 and R3 at 135.0000** |

The twelve:

```
-67.5000  0.0000  27.0000  47.2500  54.0000  67.5000
 81.0000 94.5000 108.0000 114.7500 121.5000 135.0000
```

That all five reproduce from an independently written parser is the check on the
channel map and the sign map, not on the geometry. **G0117's outlier lands on L3 and
R3 — the rear pair — which is what D275 calls it.** The map and the ruling agree
about which legs are at the back.

---

## 3. The residual

```
(theta1, theta2)  --ik_fk_leg-->  (x, y, z)  --ik_solve_leg-->  (theta1', theta2')

residual = max(|theta1' - theta1|, |theta2' - theta2|)      degrees
```

`ik_solve_leg` is called with `IK_PROJ_NONE`, so an unreachable target returns a
status and writes nothing rather than being quietly projected onto the surface.

**`IK_E_LIMIT` cannot appear.** `ik_solve_leg`'s signature carries no leg index, so
it cannot select an envelope row; the envelope is the caller's check. This table does
not make it, because Table 1 is about geometry and not about envelopes.

---

## 4. Table 1 — all 394 poses, theta3 per pose

`reports/d275_table1_per_pose_theta3.csv`, 2,364 rows = 394 poses x 6 legs.

**Distinct configurations: 71 of the 394. The largest single configuration accounts
for 24 poses.** The table stays at 394 rows and every one of them was run; what the
row count does not say is that **394 rows are not 394 independent checks.** The 38
vendor sequences reuse their key frames, so a suite reporting 394 green has exercised
71 distinct 18-channel configurations. Counted over `#000`–`#008` and `#015`–`#023`;
`#009`–`#014` and `#024`–`#031` are excluded, and the count is taken on the raw
integer command grid.

```
multiplicity histogram, 394 poses -> 71 configurations

  1x:10   2x:14   3x:3   4x:10   5x:11   6x:2   7x:3   8x:2
  9x:2   10x:3   11x:2  12x:3   13x:3   17x:2  24x:1
```

Reproduce with `python -m tools.distinct_configurations --actions <path>`.

```
rows                    2364
IK_OK                   1298
IK_W_REFLECTED          1066      exact, pose written (D342)
errors                     0

max  |d theta|          7.629395e-06 deg   =  5.651403e-05 command steps
mean |d theta|          2.433412e-06 deg
worst row               G0100 L2, theta3 = 114.7500, d_theta2 = -7.629e-06

max |d theta| on the reflected rows   1.525879e-05 deg

poses with all six plain OK         202
poses with at least one reflected   192
```

**This block changed meaning on 5 September and not one of its numbers moved.** Before
D342 the 1,066 rows were `E_UNREACHABLE_NEAR` — refused, with no pose written. **D342
clause 3 makes them `IK_W_REFLECTED`: solved exactly, with the pose written**, and D344
corrects the reachable set to the half-surface to match. **The partition is the same
partition; what was a refusal is now a warning carrying an answer.**

**The word *refusal* below is kept where it describes the pre-D342 behaviour**, and is
not a claim about the current core. §4.1's finding is untouched by the change: it was
always about the sign of `r`, never about which status the sign produced.

**On every row it solves, the geometry closes at the float32 floor** — four orders of
magnitude below the `0.1350` command grid, so no residual here could mask a real
reach failure.

### 4.1 The 1,066 refusals are one property, and the partition is exact

**Every refusing row has `r = L1 + R*cos(theta2 + psi) < 0`. No solving row does.**
1,066 of 1,066 and 0 of 1,298 — an exact partition in both directions, which is what
makes it a property rather than a coincidence.

`ik_fk_leg` writes `x = r*cos(theta1)`, `y = r*sin(theta1)` and **does not require
`r > 0`**. When the foot folds back past the coxa axis, `r` goes negative and the
point is emitted at azimuth `theta1 + 180deg`. `ik_solve_leg` then recovers the reach
as `sqrt(x^2 + y^2)`, which is `|r|`, and the sign is gone.

**So the forward map is not injective across `r = 0`**, and the inverse cannot
recover the pose from position alone: the same `(x, y, z)` is reached by
`(theta1, theta2)` with `r < 0` and by `(theta1 + 180deg, theta2')` with `r > 0`.

**`ik_solve_leg` refuses rather than returning the wrong one of the two, and that is
correct.** It is reported because `ik_core.h` does not say so. The header describes
the reachable set as the surface at distance `R` from the femur axis; it is the
**half** of that surface with `L1 + R*cos(Theta) > 0`. A caller reading only the
header would expect these poses to solve.

The boundary is `Theta = arccos(-L1 / R)`, and it moves with `theta3` because `R`
does:

| theta3 | R | psi | folds at theta2 > |
|---|---|---|---|
| -67.5000 | 156.7976 | -41.5747 | 147.1117 |
| 0.0000 | 186.8231 | 0.0000 | 102.9918 |
| 54.0000 | 167.3721 | 32.9823 | 71.5507 |
| 94.5000 | 129.9165 | 59.7933 | 49.0684 |
| 121.5000 | 97.2468 | 80.9154 | **34.6722** |
| 135.0000 | 79.8219 | 93.9054 | 27.8418 |

**878 of the 1,066 refusals sit at `theta3 = 121.5000`**, where a femur angle above
34.6722 deg is enough to fold the foot behind the axis. The vendor uses 40.5000 deg
there — see G0001, the first pose in the corpus.

**Not claimed:** that the vendor's poses are wrong, or that the robot cannot hold
them. It holds them; the leg is tucked. **Claimed:** that position alone does not
identify them, and any pipeline that round-trips through Cartesian foot position
loses them.

---

## 5. Table 2 — 284 uniform-tibia poses at one fixed theta3

`reports/d275_table2_fixed_theta3.csv`, 1,704 rows = 284 poses x 6 legs.

**Distinct configurations: 48 of the 284. The largest accounts for 24 poses** — the
same configuration that dominates Table 1, and it is uniform-tibia. **The shipped
configuration passes on 48 distinct configurations, not on 284.**

```
multiplicity histogram, 284 poses -> 48 configurations

  1x:4    2x:10   4x:9    5x:8    7x:3    8x:2
  9x:2   10x:3   12x:3   13x:3   24x:1
```

**The uniform-tibia subset is selected in ANGLE space, not command space.** The two
sides are mirrored — `sign_for` returns `+1` on the left tibia and `-1` on the right —
so six legs at one physical angle carry commands that sum to 3000 rather than commands
that are equal. Selecting on raw command equality returns 2 poses. This label was
derived wrongly that way first and the 284 in the line above is what caught it.

Reproduce with `python -m tools.distinct_configurations --actions <path>`.

```
fixed theta3            -30.0000     status: surrogate
derived                 R = 180.7311     psi = -18.1543

rows                    1704
IK_OK                   1704
IK_W_REFLECTED             0
errors                     0

max  |d theta|          1.907349e-06 deg  =  1.412851e-05 command steps
mean |d theta|          6.189928e-07 deg
worst row               G0068 R1, d_theta1 = +9.537e-07, d_theta2 = -1.907e-06
```

**Nothing folds.** At `theta3 = -30.0000` the tibia opens the leg out rather than
tucking it, `R` is 180.7311 and the fold boundary sits far above any femur angle in
the corpus. Every row solves.

**This is the table that tests a shipped configuration**, and the shipped
configuration passes on the 284.

---

## 6. The reduction, quantified

D275 requires the fixed tibia to be published as a deliberate reduction rather than
absorbed. It is:

- **12 vendor `theta3` values against v1's one.**
- **One pose, G0117, lies outside any single choice** — its rear pair sits at
  135.0000 while the six-leg intersection ends at 121.5000. No fixed `theta3` covers
  the corpus.
- **110 poses of 394 have six tibiae that are not all equal** and are therefore not
  representable at all by a single fixed value.

**Excluding the 110 from Table 1 was refused and they are in it.** They are not a
random subset: they are the widest joint combinations in the corpus, which is where a
geometry defect would show.

**The refusals of §4.1 are not concentrated in them, and this was measured rather
than assumed:**

```
from the 110 non-uniform poses    168 refusals of  660 rows   25.4545 %
from the 284 uniform poses        898 refusals of 1704 rows   52.7000 %
                                 ----
                                 1066
```

**The uniform poses fold roughly twice as often.** A first draft of this section
asserted the opposite, on the reasoning that the widest combinations must contribute
the most refusals — plausible, and false. The tuck is driven by `theta3` being large,
and the corpus's large-`theta3` poses are mostly ones where all six tibiae agree.

**So the case for keeping the 110 is not that they produce the finding.** It is
D275's own: they are non-random, they are the widest combinations, and an exclusion
would have to be justified rather than convenient. §4.1 would have been found without
them.

`theta3_min_deg` / `theta3_max_deg` are **not set from this**. They are quarantined
under D260. `[-67.5000, +121.5000]` is an **upper bound on any fixed choice** and is
`INFERRED`.

---

## 7. What this does and does not establish

**Establishes** — the two-segment reduction `R`, `psi`, `Theta = theta2 + psi` is
self-consistent to the float32 floor across twelve tibia angles and 394 poses; the
channel map and the per-joint sign map reproduce all five of D275's published
figures; and the reachable set is a half-surface, not a surface.

**Does not establish** — that the angles are the robot's. The command offset (D262)
is Spider Hardware's, is not assumed zero, and cancels in a round trip, so nothing
here speaks to where the servos actually point. A round trip cannot detect an error
that is applied identically in both directions: **if `R` and `psi` were both wrong in
a self-consistent way, every residual above would still be 1e-6.** What would catch
that is a measurement, and it is A-day's.

`theta3_deg = -30.0000` is a `surrogate`. Table 2's cleanliness is a property of that
choice and does not transfer to whatever D260 eventually rules.

---

## 8. Files

| | |
|---|---|
| `tools/d275_fk_residuals.py` | the run |
| `tests/test_d275_residuals.py` | 5 tests pinning §2's five figures and §4.1's partition |
| `reports/d275_table1_per_pose_theta3.csv` | 2,364 rows |
| `reports/d275_table2_fixed_theta3.csv` | 1,704 rows |

Suite: **148 passing**, up from 143.
