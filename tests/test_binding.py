"""Tests for the ctypes binding, the key-to-field identity, and the sign map.

Each test here corresponds to a defect that was actually made, not to a
hypothetical one. The names say which.
"""

import ctypes
import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

from bindings.hexconfig import (
    HEX_CFG_ERR, LEG_GROUP, LEGS, N_JOINTS, HexConfig, HexDerived, K_ULP,
    agree, apply_sign, assert_layout, describe, fill, hex_coxa, hex_femur,
    load_library, project_joint_limits, project_theta3_envelope, sign_map, ulp32,
)
from sim.constants import ConstantError, load

ROOT = Path(__file__).resolve().parent.parent
CFLAGS = ["-std=c99", "-Wall", "-Wextra", "-pedantic", "-O2", "-fPIC", "-shared",
          "-ffp-contract=off"]


# --------------------------------------------------------------------------
# Fixtures
# --------------------------------------------------------------------------

@pytest.fixture(scope="session")
def lib(tmp_path_factory):
    """Build core/ with the mandated flags and load it.

    No -ffast-math, -Ofast or -funsafe-math-optimizations anywhere: they permit
    reassociation, which invalidates every error bound the agreement test rests
    on. If they are ever added the build must fail here, not pass quietly.
    """
    src = [str(ROOT / "core" / "src" / n) for n in ("hex_config.c", "ik_core.c")]
    if not all(os.path.exists(s) for s in src):
        pytest.skip("core/src not present")
    so = tmp_path_factory.mktemp("build") / ("libhex" + (".dll" if os.name == "nt" else ".so"))
    cc = os.environ.get("CC", "gcc")
    try:
        r = subprocess.run([cc] + CFLAGS + ["-I", str(ROOT / "core" / "include"),
                                            "-o", str(so)] + src + ["-lm"],
                           capture_output=True, text=True)
    except FileNotFoundError:
        # Deliberately NOT a skip. The binding is a deliverable; a skip here
        # would be CI green while the thing under test never ran, which is the
        # exact failure the 2 OS x 2 Python matrix exists to prevent.
        raise AssertionError(
            "no C compiler found as %r. Install one, or set CC.\n"
            "  Windows: MSYS2 UCRT64, then add C:\\msys64\\ucrt64\\bin to PATH\n"
            "  Linux/macOS: gcc or clang" % cc)
    assert r.returncode == 0, "core/ failed to build:\n" + r.stderr
    assert r.stderr.strip() == "", "core/ built with warnings:\n" + r.stderr
    return load_library(so)


@pytest.fixture(scope="session")
def constants():
    return load()


@pytest.fixture(scope="session")
def envelopes(constants):
    return constants.value("joint_envelopes_deg")


# --------------------------------------------------------------------------
# 1. Layout
# --------------------------------------------------------------------------

def test_layout_triples_match(lib):
    """sizeof is not enough: two layouts can share a size and differ at every offset."""
    assert assert_layout(lib) == len(HexConfig._fields_)


def test_field_introspection_is_out_of_range_safe(lib):
    n = lib.hex_config_field_count()
    minus_one = ctypes.c_size_t(-1).value
    for i in (-1, n, n + 100):
        assert lib.hex_config_field_name(i) is None
        assert lib.hex_config_field_offset(i) == minus_one
        assert lib.hex_config_field_size(i) == minus_one


def test_array_field_size_is_the_whole_array(lib):
    """An array member is ONE field and its size is the whole array."""
    sizes = {lib.hex_config_field_name(i).decode(): lib.hex_config_field_size(i)
             for i in range(lib.hex_config_field_count())}
    assert sizes["coxa_x_mm"] == 6 * ctypes.sizeof(ctypes.c_float)
    assert sizes["joint_min_deg"] == 12 * ctypes.sizeof(ctypes.c_float)


# --------------------------------------------------------------------------
# 2. The key-to-field identity
#
# jq ruled the JSON keys are renamed rather than a mapping table published,
# because nothing checks a mapping table: swap two entries and every offset
# assertion still passes while the invariant silently inverts.
# --------------------------------------------------------------------------

def _schema():
    return json.loads((ROOT / "config" / "hexapod.json").read_text(encoding="utf-8"))["_schema"]


def test_every_struct_field_is_reached_exactly_once(constants):
    keys = set(constants.names())
    derived = set(_schema()["derived_fields"])
    for name, _ in HexConfig._fields_:
        reached = (name in keys) + (name in derived)
        assert reached == 1, (
            "%s is reached %d times; it must come from exactly one of a key or a derivation"
            % (name, reached))


def test_every_key_is_declared(constants):
    struct = {n for n, _ in HexConfig._fields_}
    declared = set(_schema()["not_a_struct_field"])
    for key in constants.names():
        assert (key in struct) ^ (key in declared), (
            "%s must be either a struct field name or listed in "
            "_schema.not_a_struct_field with a reason" % key)


def test_no_derived_quantity_is_also_stored(constants):
    """Rule 2, which has caught three defects: pi/2, 93.0233 mm, 111.6279 mm."""
    for name in _schema()["derived_fields"]:
        assert name not in constants.names(), (
            "%s is declared derived AND stored as a key - two sources for one number" % name)


# --------------------------------------------------------------------------
# 3. The sign map
#
# MPTASK_05 proposed one sign per leg. That is right for femur and tibia and
# wrong for the coxa, and the wrong version passes every layout assertion.
# --------------------------------------------------------------------------

def test_coxa_sign_is_identity_on_both_sides():
    """theta1 is antisymmetric and the vendor mirror is antisymmetric, so they cancel."""
    assert sign_map("coxa", "left") == 1.0
    assert sign_map("coxa", "right") == 1.0


def test_femur_and_tibia_are_negated_on_the_right():
    for joint in ("femur", "tibia"):
        assert sign_map(joint, "left") == 1.0
        assert sign_map(joint, "right") == -1.0


def test_blanket_per_leg_sign_would_reflect_the_middle_coxa(envelopes):
    """The named wrong premise, kept as a test so it cannot come back.

    Under a per-LEG map the two middle legs sweep in opposite directions. The
    front legs cannot show this: their coxa row is symmetric about zero, so
    both maps agree there by accident.
    """
    lo_l, hi_l = envelopes["coxa_middle"]["left"]
    lo_r, hi_r = envelopes["coxa_middle"]["right"]

    correct = apply_sign(lo_r, hi_r, sign_map("coxa", "right"))
    blanket = apply_sign(lo_r, hi_r, -1.0)
    assert correct != blanket, "the middle coxa row must discriminate between the two maps"
    # correct: R2 mirrors L2 about zero, i.e. both legs sweep the same way in body terms
    assert correct == (-hi_l, -lo_l)
    # blanket: R2 becomes identical to L2 in LEG terms, i.e. opposite in body terms
    assert blanket == (lo_l, hi_l)

    front = envelopes["coxa_front"]["right"]
    assert apply_sign(*front, s=1.0) == apply_sign(*front, s=-1.0), (
        "the front coxa row is symmetric, so it cannot be used to check the sign map")


def test_left_and_right_agree_after_the_per_joint_map(envelopes):
    """Every group collapses to zero difference under the correct map, for the
    joints where the mirror does not cancel."""
    for group in ("femur_all", "tibia_front_middle", "tibia_rear"):
        joint = "femur" if group.startswith("femur") else "tibia"
        left = apply_sign(*envelopes[group]["left"], s=sign_map(joint, "left"))
        right = apply_sign(*envelopes[group]["right"], s=sign_map(joint, "right"))
        assert left == pytest.approx(right), group


# --------------------------------------------------------------------------
# 4. Projection
# --------------------------------------------------------------------------

def test_theta3_envelope_is_an_intersection_not_a_union(envelopes):
    lo, hi = project_theta3_envelope(envelopes)
    for leg in LEGS:
        _c, tibia, side = LEG_GROUP[leg]
        a, b = apply_sign(*envelopes[tibia][side], s=sign_map("tibia", side))
        assert a <= lo and hi <= b, "%s is not contained: [%s, %s]" % (leg, a, b)


def test_theta3_envelope_width_equals_the_tightest_leg(envelopes):
    """Necessary, not sufficient — but the raw mirrored form returns 135.0000,
    which is no leg's span at all, and that is what first exposed the defect."""
    lo, hi = project_theta3_envelope(envelopes)
    spans = {round(b - a, 4)
             for leg in LEGS
             for a, b in [apply_sign(*envelopes[LEG_GROUP[leg][1]][LEG_GROUP[leg][2]],
                                     s=sign_map("tibia", LEG_GROUP[leg][2]))]}
    assert round(hi - lo, 4) == min(spans)


def test_surrogate_theta3_lies_inside_the_envelope(constants, envelopes):
    lo, hi = project_theta3_envelope(envelopes)
    assert lo < constants.value("theta3_deg") < hi


def test_envelope_excludes_the_psi_branch_cut(envelopes):
    """psi jumps 360 deg at theta3 == 180 (mod 360) and no ULP bound covers that.
    The core does not check it — D258 authorised one error code and no more."""
    lo, hi = project_theta3_envelope(envelopes)
    assert not (lo <= 180.0 <= hi) and not (lo <= -180.0 <= hi)


def test_joint_limits_are_ordered(envelopes):
    """hex_config_validate does NOT catch min >= max on the twelve commanded
    joints, so the check lives here (COREDROP_04 §9b)."""
    mins, maxs = project_joint_limits(envelopes)
    assert len(mins) == len(maxs) == N_JOINTS
    for i in range(N_JOINTS):
        assert mins[i] < maxs[i], i


def test_interleaved_index_macros():
    assert [hex_coxa(i) for i in range(6)] == [0, 2, 4, 6, 8, 10]
    assert [hex_femur(i) for i in range(6)] == [1, 3, 5, 7, 9, 11]
    assert sorted([hex_coxa(i) for i in range(6)] + [hex_femur(i) for i in range(6)]) \
        == list(range(N_JOINTS))


# --------------------------------------------------------------------------
# 5. The filler refuses rather than defaults
# --------------------------------------------------------------------------

def test_strict_fill_refuses_on_unspecified():
    with pytest.raises(ConstantError):
        fill(strict=True)


def test_deferred_fields_are_nan_not_zero():
    """A zero would pass hex_config_validate's guards silently. NaN does not."""
    cfg, _surrogates, deferred = fill(strict=False)
    assert deferred, "nothing was deferred; this test no longer tests anything"
    for name in deferred:
        assert math.isnan(getattr(cfg, name)), name


def test_core_rejects_a_deferred_config(lib):
    cfg, _s, deferred = fill(strict=False)
    err = lib.hex_config_validate(ctypes.byref(cfg))
    assert err != 0, "a config with %s missing must not validate" % ", ".join(deferred)
    assert describe(err) == "E_RAMP"


def test_derived_config_is_usable_despite_the_deferral(lib):
    """hex_derive and FK never read the gait fields, so they still work."""
    cfg, _s, _d = fill(strict=False)
    d = HexDerived()
    assert lib.hex_derive(ctypes.byref(cfg), ctypes.byref(d)) == 0
    assert d.rigid_len_mm > 0.0
    assert d.psi_deg != 0.0, "a zero psi means theta3 collapsed the leg to two members"


def test_fill_reports_the_surrogates_it_used():
    _cfg, surrogates, _d = fill(strict=False)
    assert "theta3_deg" in surrogates, (
        "theta3_deg is a surrogate and every output fed by it must be stamped")


# --------------------------------------------------------------------------
# 6. Agreement, in ULPs. Not decimal places.
# --------------------------------------------------------------------------

def test_ulp32_matches_the_float_grid():
    assert ulp32(180.0) == pytest.approx(2.0 ** -16)
    assert ulp32(1.0) == pytest.approx(2.0 ** -23)
    assert ulp32(0.0) > 0.0


def test_derive_agrees_with_the_double_path(lib, constants):
    """The C core is float, sim/ is double. They agree to within K ULP at the
    field's scale. This is NOT a comparison at N decimal places: rounding two
    precisions to a shared decimal grid disagrees 5.58 % of the time at 4 dp
    and 0.06 % at 2 dp — the rate falls with the places and never reaches zero.
    """
    cfg, _s, _d = fill(strict=False)
    out = HexDerived()
    assert lib.hex_derive(ctypes.byref(cfg), ctypes.byref(out)) == 0

    L2 = constants.value("femur_length_mm")
    L3 = constants.value("tibia_length_mm")
    t3 = math.radians(constants.value("theta3_deg"))
    R = math.sqrt(L2 * L2 + L3 * L3 + 2 * L2 * L3 * math.cos(t3))
    psi = math.degrees(math.atan2(L3 * math.sin(t3), L2 + L3 * math.cos(t3)))
    theta0 = constants.value("theta2_nom_deg") + psi

    expected = {
        "rigid_len_mm": R,
        "psi_deg": psi,
        "theta_nom_deg": theta0,
        "a_eff_nom_mm": R * math.cos(math.radians(theta0)),
        "body_height_mm": R * math.sin(math.radians(theta0)),
        "r_nom_mm": constants.value("coxa_length_mm") + R * math.cos(math.radians(theta0)),
    }
    for field, want in expected.items():
        got = getattr(out, field)
        assert agree(got, want, field, out), (
            "%s: C float %.10f vs double %.10f, %.2f ULP at the scale (limit %d)"
            % (field, got, want, abs(got - want) / ulp32(max(abs(got), abs(want))), K_ULP))


def test_decimal_place_comparison_is_unsound():
    """The reason the test above is not written at 4 dp. Kept as a live check so
    that nobody 'simplifies' it back."""
    L2, L3 = 74.2, 112.6231
    bad = 0
    n = 0
    for i in range(-6000, 1):
        t = i / 100.0
        r = math.radians(t)
        dbl = math.sqrt(L2 * L2 + L3 * L3 + 2 * L2 * L3 * math.cos(r))
        flt = ctypes.c_float(dbl).value
        n += 1
        if round(dbl, 4) != round(flt, 4):
            bad += 1
    assert bad > 0, (
        "float and double agreed at 4 dp on every sample; if this ever passes, "
        "check that the comparison is still crossing precisions at all")


# --------------------------------------------------------------------------
# 7. Every error code is reachable and each names its own cause
# --------------------------------------------------------------------------

@pytest.mark.parametrize("field,value,expected", [
    ("femur_length_mm", -1.0, "E_MEMBER"),
    ("duty_factor", 1.0, "E_DUTY"),
    ("stride_mm", 0.0, "E_STRIDE"),
    ("swing_clearance_mm", 500.0, "E_CLEARANCE"),
    ("dtheta_peak_deg_s", 0.0, "E_RATE"),
    ("swing_peak_factor", 0.5, "E_PROFILE"),
    ("stride_mm", 900.0, "E_REACH"),
    ("margin_factor", 0.5, "E_MARGIN"),
    ("command_step_deg", 0.0, "E_RESOLUTION"),
    ("joint_accuracy_deg", -0.1, "E_RESOLUTION"),
    ("theta3_deg", 130.0, "E_THETA3"),
])
def test_each_error_code_fires_for_its_own_cause(lib, field, value, expected):
    cfg, _s, _d = fill(strict=False)
    cfg.stale_ramp_ms = 200.0          # lift the deferral so the later guards are reachable
    cfg.swing_eps_mm_s = 1.0
    assert describe(lib.hex_config_validate(ctypes.byref(cfg))) == "HEX_CFG_OK", \
        "baseline must validate before a single field is broken"
    setattr(cfg, field, value)
    assert describe(lib.hex_config_validate(ctypes.byref(cfg))) == expected


def test_no_error_code_is_dead(lib):
    """Every member of hex_cfg_err_t must be produced by something."""
    reached = {"HEX_CFG_OK"}
    for field, value in [("femur_length_mm", -1.0), ("duty_factor", 1.0), ("stride_mm", 0.0),
                         ("swing_clearance_mm", 500.0), ("dtheta_peak_deg_s", 0.0),
                         ("swing_peak_factor", 0.5), ("stride_mm", 900.0),
                         ("stale_ramp_ms", 1.0), ("margin_factor", 0.5),
                         ("command_step_deg", 0.0), ("theta3_deg", 130.0)]:
        cfg, _s, _d = fill(strict=False)
        cfg.stale_ramp_ms = 200.0
        cfg.swing_eps_mm_s = 1.0
        setattr(cfg, field, value)
        reached.add(describe(lib.hex_config_validate(ctypes.byref(cfg))))
    assert reached == set(HEX_CFG_ERR), "unreached: %s" % (set(HEX_CFG_ERR) - reached)
