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
    assert unspecified, "expected at least tibia_length_mm and theta_3_deg to be blocked"
    for name in unspecified:
        with pytest.raises(C.ConstantError):
            k.value(name)


def test_the_two_known_blockers_are_still_blocked():
    """Delete this test the day PROJECT_02 arrives and these get real values."""
    k = C.load()
    assert k.status("tibia_length_mm") == "unspecified"
    assert k.status("theta_3_deg") == "unspecified"


def test_unknown_name_raises():
    k = C.load()
    with pytest.raises(C.ConstantError):
        k.value("femur_length")          # near-miss of femur_length_mm
    with pytest.raises(C.ConstantError):
        k.value("_schema")               # metadata is not a constant


# ---------------------------------------------------- values that must be right

def test_command_step_is_the_post_project_04_value():
    """PROJECT_04 §2. Anything still holding 0.4392 is stale."""
    k = C.load()
    assert k.value("command_step_deg") == 0.3


def test_all_six_legs_are_present_in_every_per_leg_table():
    """D23.3: a single-leg derivation is an estimate, not a result."""
    k = C.load()
    expected = {"R1", "R2", "R3", "L1", "L2", "L3"}
    for name in ("coxa_positions_mm", "leg_neutral_direction_deg"):
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
