"""`hex_derive` and `ik_fk_leg` in float, against sim/derive.py in double.

THE PREDICATE, STATED HERE BECAUSE D383 REQUIRES IT
---------------------------------------------------
**This file checks float32-against-double IMPLEMENTATION AGREEMENT at `N_ULP = 8`
ulps of the leg's reach.** Each C output and its double counterpart must differ
by no more than `N_ULP * FLOAT32_EPS * (L1 + L2 + L3)`.

**It does not check D147's four-decimal REPORTING agreement**, and passing here
says nothing about it. D147 is a convention for how one computed figure is
written down; agreement across a float32/double boundary is a different
predicate about a different quantity. D383 keeps the budget at 8 ulps on that
ground and rules that the two are not in conflict.

WHY THIS FILE EXISTS, AND WHY IT IS LATE
-----------------------------------------
`sim/derive.Derived`'s docstring has said since 27 August that its fields mirror
`hex_derived_t` *so that the 27 August agreement test can compare them field by
field*.  `README.md` has said since it was written that this check happens *once
`hex_config.c` lands*.  **`hex_config.c` landed on 26 August and this file did not
exist until 10 September.**  For sixteen days the repository shipped two
independent implementations of the same formulas and nothing compared them.

The design is deliberate: `sim/` is the reference and `core/` is what the firmware
calls, and neither is generated from the other.  That separation is only worth
anything if the two are checked, and until now the claim that they agreed was an
assumption wearing a README sentence.

THE TOLERANCE, AND HOW IT WAS ARRIVED AT
------------------------------------------
`core/` computes in `float`; `sim/` computes in double.  They cannot agree
exactly and asking them to is asking the wrong question.

**A purely RELATIVE tolerance is wrong here and was tried first.**  At
`theta2 = 20`, `Theta = theta2 + psi` is 1.85 degrees, so `z = -R*sin(Theta)` is
about 5.8 mm -- a small output produced from large intermediates.  Measured
against its own magnitude the disagreement there is 8.6 ulps and rising as the
output approaches zero, which reports a precision collapse that is not happening.
The error was accumulated on `R` (180.7 mm), not on `z`.

**So the budget is set against the leg's characteristic reach**, `L1 + L2 + L3`,
read from the config and never written here:

    |c_value - python_value|  <=  N_ULP * FLOAT32_EPS * REACH

That is the scale the intermediates actually live on.  Under it, the worst
disagreement over 1,221 poses (`theta1` -90..90, `theta2` -60..100, 5 degree
steps, three axes each -- 3,663 comparisons) is **1.02 ulps**, and the six
`hex_derived_t` scalars are all under 0.5.

`N_ULP = 8` is the operational budget.  The FK chain is roughly ten elementary
float operations; each rounding contributes up to half an ulp and `cosf`, `sinf`
and `sqrtf` are typically within one ulp each, so a naive worst case sits near
ten.  Eight is chosen to sit above every measured value with roughly eight times
headroom while staying below the naive bound, so it is a **tripwire and not a
proof**: if it fires, either an implementation changed or the analysis above is
wrong, and both are worth being told about.

WHAT THIS FILE HAS POWER OVER, MEASURED RATHER THAN CLAIMED
  Both controls below were executed on 10 September, and the first one FAILED.

  `-ffast-math`: **no power.** This docstring first asserted that building with it
  would turn these tests red.  It was tried: **all twelve passed.**  The chain is
  short enough that the reassociation the flag permits stays well inside eight
  ulps.  The claim is withdrawn rather than quietly removed, because a stated
  power that was never tested is the thing this whole file exists to argue
  against.

  A formula divergence: **power, and the tripwire is where it lives.**  Scaling
  `sim/derive`'s `psi_deg` by 1 + 2e-06 -- a change invisible at the four decimal
  places D147 prints -- moves the worst FK disagreement from 3.15e-05 mm to
  1.12e-04 mm.  **`test_the_measured_disagreement_sits_well_inside_the_budget`
  fails and the budget test does not.**  So the half-budget tripwire is not
  decoration: without it a 2 ppm divergence between the two implementations ships
  green.

WHAT IT HAS NO POWER OVER
  **whether either path is correct.**  Two implementations of the same wrong
  formula agree perfectly.  This is a comparison, not a validation, and the same
  caution `reports/D275_vendor_pose_validation.md` states about round-trip
  residuals applies here word for word: agreement is self-consistency.  Only
  measurement settles correctness, and that is A-day's.

WHY A BUDGET WIDER THAN ONE UNIT IN THE FOURTH DECIMAL PLACE IS NOT A DEFECT
  The budget is 2.18e-04 mm at full reach, which exceeds one unit in D147's fourth
  decimal place.  **This docstring previously called that a tension.  It is not
  one, and D383 says why:** the budget bounds float32-against-double arithmetic,
  and D147 governs how a reported figure is written.  A future disagreement in a
  fourth decimal place, inside this budget, is not a failure of this file, and
  this file is not the check for it.  The earlier paragraph also quoted a worst
  case of 2.79e-05 mm with no grid beside it; the figure with its grid is in
  MEASURED below.

MEASURED, 10 September, at `cd3f5c6` plus this file: 1,480 poses over `theta1`
-90..90 and `theta2` -60..135 in 5 degree steps, three axes each, **4,440
comparisons.  Worst disagreement 3.15e-05 mm against a 2.18e-04 mm budget, 14.4
per cent of it**, at `theta1 = -90`, `theta2 = 135` -- past the fold, which is the
least well-conditioned part of the grid and the reason the grid goes there.

NO SKIP.  If no compiler is found this fails, as `tests/test_binding.py` does and
for the same reason: a skip here would be CI green over the only check that the
two halves of this repository are the same thing.
"""

import ctypes
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from sim.constants import load  # noqa: E402
from sim.derive import derive  # noqa: E402
from tools.d275_fk_residuals import HexConfig, HexDerived, bind  # noqa: E402

FLOAT32_EPS = 2.0 ** -23
N_ULP = 8.0

# Characteristic scale for angular quantities. A degree measure lives on [-180, 180]
# whatever its own magnitude, for the same reason z lives on R rather than on z.
ANGLE_SCALE_DEG = 180.0

LENGTH_FIELDS = ("rigid_len_mm", "a_eff_nom_mm", "body_height_mm", "r_nom_mm")
ANGLE_FIELDS = ("psi_deg", "theta_nom_deg")


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    """Build core/ and bind it. FAILS rather than skips when there is no compiler."""
    so = tmp_path_factory.mktemp("agree") / (
        "libhex" + (".dll" if os.name == "nt" else ".so"))
    cc = os.environ.get("CC", "gcc")
    sources = sorted(str(p) for p in (ROOT / "core" / "src").glob("*.c"))
    assert sources, "core/src holds no .c files"
    cmd = [cc, "-std=c99", "-Wall", "-Wextra", "-pedantic", "-O2",
           "-shared", "-fPIC", "-I", str(ROOT / "core" / "include"),
           *sources, "-o", str(so), "-lm"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True)
    except FileNotFoundError:
        pytest.fail(
            "no C compiler on PATH as %r. This test does not skip: it is the only "
            "check that sim/ and core/ compute the same thing, and a skip here "
            "would be CI green over that." % cc)
    assert r.returncode == 0, "core/ failed to build:\n" + r.stderr
    return bind(ctypes.CDLL(str(so)))


@pytest.fixture(scope="module")
def constants():
    return load()


@pytest.fixture(scope="module")
def reach_mm(constants):
    """L1 + L2 + L3, read from the config. Never written into this file."""
    return (float(constants.value("coxa_length_mm"))
            + float(constants.value("femur_length_mm"))
            + float(constants.value("tibia_length_mm")))


@pytest.fixture(scope="module")
def cfg(constants):
    c = HexConfig()
    for name, _ctype in HexConfig._fields_:
        try:
            v = constants.value(name)
        except Exception:
            continue
        if isinstance(v, (int, float)):
            setattr(c, name, v)
    return c


@pytest.fixture(scope="module")
def c_derived(lib, cfg):
    d = HexDerived()
    rc = lib.hex_derive(ctypes.byref(cfg), ctypes.byref(d))
    assert rc == 0, "hex_derive rejected the shipped config, rc=%d" % rc
    return d


@pytest.fixture(scope="module")
def py_derived(constants):
    return derive(constants)


def budget(scale):
    return N_ULP * FLOAT32_EPS * scale


# ------------------------------------------------------- the six derived scalars

@pytest.mark.parametrize("field", LENGTH_FIELDS)
def test_derived_lengths_agree_within_the_float32_budget(
        field, c_derived, py_derived, reach_mm):
    c_value = getattr(c_derived, field)
    py_value = getattr(py_derived, field)
    limit = budget(reach_mm)
    assert abs(c_value - py_value) <= limit, (
        "%s: core %.10f, sim %.10f, differ by %.3e mm against a %.3e mm budget"
        % (field, c_value, py_value, abs(c_value - py_value), limit))


@pytest.mark.parametrize("field", ANGLE_FIELDS)
def test_derived_angles_agree_within_the_float32_budget(
        field, c_derived, py_derived):
    c_value = getattr(c_derived, field)
    py_value = getattr(py_derived, field)
    limit = budget(ANGLE_SCALE_DEG)
    assert abs(c_value - py_value) <= limit, (
        "%s: core %.10f, sim %.10f, differ by %.3e deg against a %.3e deg budget"
        % (field, c_value, py_value, abs(c_value - py_value), limit))


def test_every_hex_derived_field_is_covered(py_derived):
    """A comparison that silently omits a field passes for the wrong reason."""
    covered = set(LENGTH_FIELDS) | set(ANGLE_FIELDS)
    declared = set(py_derived.__slots__)
    assert covered == declared, (
        "hex_derived_t and this test disagree about which fields exist: "
        "uncovered %s, unknown %s"
        % (sorted(declared - covered), sorted(covered - declared)))


# ------------------------------------------------------------ forward kinematics

def fk_grid():
    """theta2 runs past the fold on purpose.

    r = L1 + R*cos(theta2 + psi) goes negative above theta2 = 121.5921 at the
    shipped theta3 -- reports/D275_vendor_pose_validation.md section 4.1. A grid
    that stopped at 100 would compare the two paths only on the well-conditioned
    side, which is the side where they cannot disagree. The first version of this
    grid did stop at 100, and the coverage test below is what said so.
    """
    for t1 in range(-90, 91, 5):
        for t2 in range(-60, 136, 5):
            yield float(t1), float(t2)


def python_fk(constants, d, theta1_deg, theta2_deg):
    """The leg-frame FK, in double, from the same formula ik_fk_leg implements."""
    theta = math.radians(theta2_deg + d.psi_deg)
    r = float(constants.value("coxa_length_mm")) + d.rigid_len_mm * math.cos(theta)
    t1 = math.radians(theta1_deg)
    return r * math.cos(t1), r * math.sin(t1), -d.rigid_len_mm * math.sin(theta)


def measure_fk(lib, cfg, c_derived, constants, py_derived):
    """Worst absolute disagreement over the grid, and where it sat."""
    F = ctypes.c_float
    x, y, z = F(), F(), F()
    worst, where = 0.0, None
    for t1, t2 in fk_grid():
        lib.ik_fk_leg(ctypes.byref(cfg), ctypes.byref(c_derived),
                      F(t1), F(t2), ctypes.byref(x), ctypes.byref(y), ctypes.byref(z))
        px, py_, pz = python_fk(constants, py_derived, t1, t2)
        for c_value, py_value, axis in ((x.value, px, "x"),
                                        (y.value, py_, "y"),
                                        (z.value, pz, "z")):
            delta = abs(c_value - py_value)
            if delta > worst:
                worst, where = delta, (t1, t2, axis, c_value, py_value)
    return worst, where


def test_forward_kinematics_agrees_across_the_grid(
        lib, cfg, c_derived, constants, py_derived, reach_mm):
    worst, where = measure_fk(lib, cfg, c_derived, constants, py_derived)
    limit = budget(reach_mm)
    assert worst <= limit, (
        "worst FK disagreement %.4e mm exceeds the %.4e mm budget, at "
        "theta1=%.1f theta2=%.1f axis %s: core %.10f, sim %.10f"
        % (worst, limit, where[0], where[1], where[2], where[3], where[4]))


def test_the_grid_is_not_empty_and_covers_the_fold(lib, cfg, c_derived,
                                                   constants, py_derived):
    """A generator that yields nothing passes every assertion above it.

    The fold at r = 0 is included on purpose: it is where the FK is least well
    conditioned in the leg frame, and a grid that stopped short of it would be
    comparing the two paths only where they cannot disagree.
    """
    points = list(fk_grid())
    assert len(points) >= 1000, "the grid collapsed to %d points" % len(points)
    signs = set()
    for t1, t2 in points:
        _, _, _ = python_fk(constants, py_derived, t1, t2)
        theta = math.radians(t2 + py_derived.psi_deg)
        r = float(constants.value("coxa_length_mm")) + py_derived.rigid_len_mm * math.cos(theta)
        signs.add(r > 0.0)
    assert signs == {True, False}, (
        "the grid stays on one side of r = 0; it covers %s only" % signs)


def test_the_measured_disagreement_sits_well_inside_the_budget(
        lib, cfg, c_derived, constants, py_derived, reach_mm):
    """A tripwire on drift, at half the budget.

    If this fires while the test above still passes, agreement is degrading
    without having broken yet -- which is exactly when it is cheap to look. The
    number to look at is in the failure message, not in this docstring.
    """
    worst, where = measure_fk(lib, cfg, c_derived, constants, py_derived)
    half = budget(reach_mm) / 2.0
    assert worst <= half, (
        "FK disagreement %.4e mm has passed half the %.4e mm budget at "
        "theta1=%.1f theta2=%.1f axis %s. Not yet a failure of the budget; "
        "reported because it used to sit at about an eighth of it."
        % (worst, budget(reach_mm), where[0], where[1], where[2]))


# ------------------------------------------------------------- does it have power

def test_the_comparison_would_catch_a_real_divergence(reach_mm):
    """What would this check also pass?

    A tolerance nobody has tested is a tolerance that might admit anything. A
    relative error of 1e-5 -- far too small to see in a printed table at four
    decimal places, and the size of a plausible transcription or unit slip -- must
    exceed the budget. If this ever stops holding, the budget has been widened to
    the point where it no longer discriminates.
    """
    limit = budget(reach_mm)
    plausible_error = reach_mm * 1e-5
    assert plausible_error > limit * 10.0, (
        "a 1e-5 relative error is %.3e mm against a %.3e mm budget -- the budget "
        "is too wide to catch the errors it exists for" % (plausible_error, limit))


def test_the_budget_is_stated_against_the_reach_and_not_hard_coded(reach_mm):
    """The scale comes from the config. A guard that hard-codes its own scale
    stops tracking the thing it guards the moment a link length changes."""
    assert 150.0 < reach_mm < 400.0, (
        "characteristic reach %.4f mm is outside anything this leg could be; "
        "the config was read but does not look like a leg" % reach_mm)
