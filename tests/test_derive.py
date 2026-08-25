"""Acceptance tests for the fourteen-output derivation.

The important thing about this file: it checks against figures supplied by the
ALGORITHM WORKSTREAM and by PROJECT_02, not against this module's own output.
A test that only checks self-consistency verifies nothing.
"""

import math

import pytest

from sim import constants as C
from sim import derive as D


class FakeTable(object):
    """A constant table with explicit values, for reproducing a stated geometry."""

    def __init__(self, **kw):
        self._v = kw

    def value(self, name):
        return self._v[name]


LEGACY = FakeTable(
    coxa_length_mm=50.0, femur_length_mm=90.0, tibia_length_mm=0.0,
    theta_3_deg=0.0, theta_2_neutral_deg=40.0, swing_clearance_mm=15.0,
    payload_mass_kg=2.15, dtheta_peak_deg_s=375.0,
    swing_velocity_profile="half_sine", tripod_support_legs=3,
    stall_torque_kgcm=20.0, torque_margin=2.5, nominal_stride_mm=60.0,
)


# ------------------------------------------------- external acceptance figures

def test_legacy_a_eff_matches_the_algorithm_workstreams_figures():
    """§3 note on output 10: 72.6690 mm at the extreme against 68.9440 nominal,
    5.40 % more. If this file cannot reproduce those, the formulas are misread."""
    row, d, _ = D.sweep_point(LEGACY, stride_mm=60.0, duty_factor=0.5)
    assert round(d.a_eff_nom_mm, 4) == 68.9440
    assert round(row["a_eff_extreme_mm"], 4) == 72.6690
    excess = (row["a_eff_extreme_mm"] / d.a_eff_nom_mm - 1.0) * 100.0
    assert round(excess, 2) == 5.40


def test_legacy_height_and_bob_match_project_02():
    """PROJECT_02 §3 'was' column: 57.851 mm standing, 4.755 mm bob at 60 mm stride."""
    row, d, _ = D.sweep_point(LEGACY, stride_mm=60.0, duty_factor=0.5)
    assert round(d.body_height_mm, 3) == 57.851
    assert round(row["bob_mm"], 3) == 4.755


def test_the_three_member_leg_collapses_exactly_onto_two():
    """The identity in hex_config.h, checked directly rather than assumed:
    L2*cos(t2) + L3*cos(t2+t3) must equal R*cos(t2+psi) for arbitrary angles."""
    L2, L3, t3 = 74.0, 61.0, 37.0
    R = math.sqrt(L2**2 + L3**2 + 2*L2*L3*math.cos(math.radians(t3)))
    psi = math.degrees(math.atan2(L3*math.sin(math.radians(t3)),
                                  L2 + L3*math.cos(math.radians(t3))))
    for t2 in (-30.0, 0.0, 17.5, 40.0, 88.0):
        a, b = math.radians(t2), math.radians(t2 + t3)
        assert math.isclose(L2*math.cos(a) + L3*math.cos(b),
                            R*math.cos(math.radians(t2 + psi)), rel_tol=1e-12)
        assert math.isclose(L2*math.sin(a) + L3*math.sin(b),
                            R*math.sin(math.radians(t2 + psi)), rel_tol=1e-12)


def test_psi_is_zero_only_when_the_tibia_is_zero():
    """The named most-likely error is confusing theta2 with Theta. They coincide
    exactly when psi is zero, and psi is zero only when L3 is zero."""
    assert D.derive(LEGACY).psi_deg == 0.0
    with_tibia = FakeTable(coxa_length_mm=50.0, femur_length_mm=90.0,
                           tibia_length_mm=45.0, theta_3_deg=-30.0,
                           theta_2_neutral_deg=40.0)
    d = D.derive(with_tibia)
    assert abs(d.psi_deg) > 1.0
    assert d.theta_nom_deg != with_tibia.value("theta_2_neutral_deg")


# --------------------------------------------------------- structural checks

def test_swing_duration_does_not_depend_on_duty():
    """§3: 'If your sweep shows output 12 varying with duty, that is a bug in the
    runner, not a finding.'"""
    a, _, _ = D.sweep_point(LEGACY, 60.0, 0.50)
    b, _, _ = D.sweep_point(LEGACY, 60.0, 0.60)
    assert a["swing_duration_ms"] == b["swing_duration_ms"]
    assert a["cycle_duration_ms"] != b["cycle_duration_ms"]


def test_peak_factor_is_computed_not_stored():
    assert D.swing_peak_factor("half_sine") == math.pi / 2.0
    with pytest.raises(ValueError):
        D.swing_peak_factor("trapezoid")


def test_unreachable_stride_raises_rather_than_returning_nonsense():
    row_ok, _, _ = D.sweep_point(LEGACY, 60.0, 0.5)
    assert row_ok["theta_extreme_deg"] > 0
    with pytest.raises(D.UnreachableStride):
        D.sweep_point(LEGACY, 900.0, 0.5)


def test_real_table_produces_all_fourteen_outputs():
    k = C.load()
    row, _, _ = D.sweep_point(k, k.value("nominal_stride_mm"), k.value("duty_factor"))
    assert list(row) == D.OUTPUT_NAMES
    assert len(row) == 14
    assert all(isinstance(v, float) for v in row.values())


def test_results_from_the_real_table_are_stamped_as_surrogate():
    """Every geometry input is currently a stand-in. No emitted figure may look
    like a measurement."""
    k = C.load()
    D.sweep_point(k, 60.0, 0.5)
    assert k.stamp().startswith("SURROGATE VALUES USED")
