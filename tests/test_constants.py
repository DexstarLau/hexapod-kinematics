"""Tests for the constant table and its loader.

These are the foundation of the hard-coded-constant guard of PROJECT_01 §5.3
and PROJECT_04 §4. That guard changes L2 / L3 / theta_3 / command_step_deg and
asserts the sweep outputs move. It can only work if there is exactly one place
those constants live. These tests protect that property.
"""

import json
import math
from pathlib import Path

import pytest

from sim import constants as C


# ---------------------------------------------------------------- schema

def test_table_loads():
    k = C.load()
    assert len(k.names()) > 0


def test_every_entry_has_the_required_fields():
    raw = json.loads(C.CONFIG_PATH.read_text(encoding="utf-8"))
    for name, entry in raw.items():
        if name.startswith("_"):
            continue
        for field in C.REQUIRED_FIELDS:
            assert field in entry, "{} is missing '{}'".format(name, field)


def test_every_status_is_recognised():
    k = C.load()
    for name in k.names():
        assert k.status(name) in C.VALID_STATUS


def test_lengths_and_angles_are_double_not_int():
    """D147: double precision throughout. An int here means someone typed 90 for 90.0."""
    k = C.load()
    for name in k.names():
        if not name.endswith(("_mm", "_deg", "_hz", "_s", "_v", "_kg")):
            continue
        if k.status(name) == "unspecified":
            continue
        value = k.value(name)
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            assert isinstance(value, float), "{} = {!r} is an int, not a double".format(name, value)


# ------------------------------------------------- the honest-failure guard

def test_unspecified_constants_raise_instead_of_returning_none():
    k = C.load()
    unspecified = k.with_status("unspecified")
    assert unspecified, "expected at least tibia_length_mm and theta3_deg to be blocked"
    for name in unspecified:
        with pytest.raises(C.ConstantError):
            k.value(name)


def test_d227_measured_geometry_is_applied():
    """The link lengths stopped being surrogates on 24 August. D227 from the D197
    STEP model. The old L1 = 50 seed was 19.0% wrong."""
    k = C.load()
    assert k.value("members_per_leg") == 3
    assert k.value("controlled_dof_per_leg") == 2
    assert k.value("coxa_length_mm") == 42.0
    assert k.value("femur_length_mm") == 74.2
    assert k.value("tibia_length_mm") == 112.6231
    assert round(sum([42.0, 74.2, 112.6231]), 4) == 228.8231
    for name in ("coxa_length_mm", "femur_length_mm", "tibia_length_mm",
                 "coxa_positions_mm", "beta_mount_deg"):
        assert k.status(name) == "measured"
    assert k.status("theta3_deg") == "surrogate"


def test_unknown_name_raises():
    k = C.load()
    with pytest.raises(C.ConstantError):
        k.value("femur_length")          # near-miss of femur_length_mm
    with pytest.raises(C.ConstantError):
        k.value("_schema")               # metadata is not a constant


# ---------------------------------------------------- values that must be right

def test_the_quantisation_constant_is_two_constants():
    """D240, as D325 amends the value. PROJECT_04 §2's 0.3000 is dead and so is the
    PCA9685 0.4392, in every form. The grid is 0.1350; the accuracy that BINDS is
    1.0000, the datasheet's 综合实际使用精度, not the 0.2400 理论精度 D240 had merged
    with it. D325."""
    k = C.load()
    assert k.value("command_step_deg") == 0.1350
    assert k.value("joint_accuracy_deg") == 1.0000
    assert round(k.value("joint_accuracy_deg") / k.value("command_step_deg"), 4) == 7.4074


def test_all_six_legs_are_present_in_every_per_leg_table():
    """D23.3: a single-leg derivation is an estimate, not a result."""
    k = C.load()
    expected = {"R1", "R2", "R3", "L1", "L2", "L3"}
    for name in ("coxa_positions_mm", "beta_mount_deg", "beta_neutral_deg"):
        assert set(k.value(name)) == expected, "{} does not cover all six legs".format(name)


# ------------------------------------------- guard against storing derived numbers

def test_no_derived_ratio_is_stored():
    """The half-sine peak/mean ratio is pi/2. It is a consequence of the profile,
    so it must be computed, never stored. Compute before asserting (PROJECT_01 §3 rule 5)."""
    k = C.load()
    assert k.value("swing_velocity_profile") == "half_sine"

    forbidden = math.pi / 2.0
    for name in k.names():
        if k.status(name) == "unspecified":
            continue
        value = k.value(name)
        if isinstance(value, float):
            assert not math.isclose(value, forbidden, rel_tol=1e-9), (
                "{} stores pi/2, which is derivable from swing_velocity_profile".format(name)
            )


# ---------------------------------------------- the generated table must not drift

def test_generated_table_matches_the_json():
    """docs/constants.md is generated. If someone edits it by hand, or edits the
    JSON without regenerating, this fires. CI runs the same check."""
    from sim.emit_constants_table import OUTPUT_PATH, render
    assert OUTPUT_PATH.read_text(encoding="utf-8") == render()


def test_generated_table_prints_four_decimal_places():
    """D147. Caught a real bug on day one: trailing zeros were being stripped,
    so 5.0 mm printed as '5' - a 1-significant-figure claim about a 4 dp value."""
    from sim.emit_constants_table import format_value
    assert format_value(5.0) == "5.0000"
    assert format_value(93.0) == "93.0000"
    assert format_value(0.3) == "0.3000"


# ---------------------------------------------------- surrogate machinery

def test_surrogates_are_tracked_and_stamped():
    """A result built on surrogates must be able to say so. PROJECT_01 rule 6."""
    k = C.load()
    assert k.stamp() == "no surrogate constants used"
    k.value("theta3_deg")
    assert "theta3_deg" in k.surrogates_read()
    assert k.stamp().startswith("SURROGATE VALUES USED")


def test_decided_constants_do_not_pollute_the_stamp():
    k = C.load()
    k.value("command_step_deg")
    k.value("update_rate_hz")
    assert k.stamp() == "no surrogate constants used"


def test_expectations_are_not_loadable_as_constants():
    """_expectations holds predicted OUTPUTS. Reading one as an input is the error
    this test exists to make impossible."""
    k = C.load()
    assert "_expectations" not in k.names()
    with pytest.raises(C.ConstantError):
        k.value("standing_height_mm")


def test_no_derived_torque_ceiling_is_stored():
    """a_eff_max was stored as 93.0 on 21 Aug. It is stall*margin/mass - a hyperbola,
    not a ceiling (PROJECT_02 §3). It must be computed."""
    k = C.load()
    assert "a_eff_max_mm" not in k.names()
    # D209 rules margin_factor = 2.5000 and PROJECT_05 §3 says "set a_eff_max_mm =
    # 111.6279". The margin is set; the ceiling is NOT stored. It is
    # tau_servo*10*legs/(mass*margin) and all three inputs are live - mass is
    # unmeasured and the margin reverts to 3.0000 automatically on two D190 triggers.
    # Checked as a figure in tests/test_torque.py instead. Filed to coordination.
    assert k.value("margin_factor") == 2.5


# ------------------------------------------------- D324, D325, D357: disputed

def test_d324_peak_rate_is_the_slower_of_two_vendor_figures():
    """D324 clauses 1 and 2. Value AND status, because either alone is half the
    decision: 250.0000 with a `decided` status would say the dispute is settled,
    and 375.0000 with a `disputed` status would say the wrong figure was chosen.

    The two figures come from 60/0.24 and 60/0.16, and the ratio between them is
    exactly 1.5000. That is computed here rather than stored anywhere -- it is a
    consequence of the two vendor readings, and a stored ratio is a derived
    quantity.
    """
    k = C.load()
    assert k.value("dtheta_peak_deg_s") == 250.0000
    assert k.status("dtheta_peak_deg_s") == "disputed"
    assert round(60.0 / 0.24, 4) == 250.0000
    assert round(60.0 / 0.16, 4) == 375.0000
    assert round(375.0000 / 250.0000, 4) == 1.5000


def test_d324_disputes_the_no_load_figure_without_deleting_it():
    """D324 clause 3: 375.0000 IS NOT VOID. Disputed and void are different
    things, and the difference is whether the number can still be read.

    A test that only checked the status would pass against an entry whose value
    had been nulled, so the value is asserted too.
    """
    k = C.load()
    assert k.status("servo_speed_no_load_deg_s") == "disputed"
    assert k.value("servo_speed_no_load_deg_s") == 375.0000
    assert "servo_speed_no_load_deg_s" not in k.with_status("unspecified")


def test_a_disputed_read_is_stamped_like_a_surrogate_read():
    """D357. The stamp is the mechanism that makes D324 clause 2's label
    executable, so it is asserted on the STAMP, not on the status field.

    Two things are pinned. The disputed clause appears only after a disputed
    constant has actually been handed out -- a stamp that always says it would
    say nothing. And the surrogate clause is unchanged character for character;
    D357 accepted that shape explicitly and two other tests assert that exact
    string.
    """
    k = C.load()
    assert k.stamp() == "no surrogate constants used"          # nothing read yet
    k.value("dtheta_peak_deg_s")
    assert k.disputed_read() == ["dtheta_peak_deg_s"]
    assert k.stamp() == ("no surrogate constants used"
                         " | DISPUTED VALUES USED: dtheta_peak_deg_s")
    k.value("theta3_deg")                                       # a surrogate
    assert k.stamp() == ("SURROGATE VALUES USED - NOT A MEASUREMENT: theta3_deg"
                         " | DISPUTED VALUES USED: dtheta_peak_deg_s")


def test_every_status_in_the_schema_is_known_to_the_loader():
    """Two vocabularies in two files that can drift apart. `_schema.status_meanings`
    documents the statuses for a human; VALID_STATUS is what `_validate` compares
    against, and a status in one and not the other is a load-time crash or a
    silently undocumented state.

    D357 exists because that drift was one `==` away from taking the whole
    repository down.
    """
    k = C.load()
    raw = json.loads(Path(C.CONFIG_PATH).read_text(encoding="utf-8"))
    documented = set(raw["_schema"]["status_meanings"])
    assert documented == set(C.VALID_STATUS), (
        "documented but unknown to the loader: %s\n"
        "known to the loader but undocumented: %s"
        % (sorted(documented - set(C.VALID_STATUS)),
           sorted(set(C.VALID_STATUS) - documented)))


def test_the_grid_never_binds_whatever_d190_measures():
    """D325's conclusion is a PROPERTY, not the number 7.4074.

    D190 will replace joint_accuracy_deg with a measurement and the ratio will
    move. What must not move is which of the two is the limit: the command grid
    binds only if the servo lands more finely than 0.1350 deg, which is four
    times better than the datasheet's theoretical figure and seven times better
    than its practical one. Asserting 7.4074 would pin today's arithmetic;
    asserting the inequality pins what D240 and D325 actually decided.
    """
    k = C.load()
    grid = k.value("command_step_deg")
    accuracy = k.value("joint_accuracy_deg")
    assert accuracy > grid, (
        "the command grid has become the binding limit -- D240's conclusion is "
        "reversed and the sweep model must change, not this test")
