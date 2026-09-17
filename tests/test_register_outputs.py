"""The two register outputs MP1 lacked (D418 clause 2) and D104's naming (D426 clause 7),
checked against the register's own printed seed figures, not against this code."""

import pytest

from sim import constants as C
from sim import derive as D


class Table(object):
    def __init__(self, base, **kw):
        self._base, self._kw = base, kw

    def value(self, name):
        return self._kw[name] if name in self._kw else self._base.value(name)


def seed(theta2=40.0):
    """D97's seed geometry as PROJECT_22 states it: L1 50, L2 90, stride 60, clearance 15."""
    return Table(C.load(), coxa_length_mm=50.0, femur_length_mm=90.0, tibia_length_mm=0.0,
                 theta3_deg=0.0, swing_clearance_mm=15.0, theta2_nom_deg=theta2)


def test_d97_seed_row_at_40_degrees():
    row, _, _ = D.sweep_point(seed(), 60.0, 0.5)
    assert round(row["r_nom_mm"], 2) == 118.94
    assert round(row["femur_travel_swing_deg"], 2) == 15.44
    assert round(row["coxa_sweep_deg"], 2) == 28.31
    assert round(row["femur_coxa_ratio_swing_avg"], 2) == 1.83
    assert round(row["cobinding_clearance_mm"], 2) == 24.14


def test_d104_names_the_span_based_clearance():
    row, _, _ = D.sweep_point(seed(), 60.0, 0.5)
    assert round(row["cobinding_clearance_mm@span_named_d104"], 4) == 39.6181
    # D104 reports the more conservative of the two
    assert row["cobinding_clearance_mm"] < row["cobinding_clearance_mm@span_named_d104"]


def test_each_clearance_makes_its_own_equality_true():
    """Put the solved clearance back into the table: the two quantities it equates must
    then be equal. A closed form that only agrees with itself would fail here."""
    for theta2 in (40.0, 44.0):
        row, _, _ = D.sweep_point(seed(theta2), 60.0, 0.5)
        at_rep, _, _ = D.sweep_point(Table(seed(theta2), swing_clearance_mm=row["cobinding_clearance_mm"]), 60.0, 0.5)
        assert at_rep["femur_travel_swing_deg"] == pytest.approx(at_rep["coxa_sweep_deg"], abs=1e-9)
        named = row["cobinding_clearance_mm@span_named_d104"]
        at_named, _, _ = D.sweep_point(Table(seed(theta2), swing_clearance_mm=named), 60.0, 0.5)
        assert at_named["theta_span_deg"] == pytest.approx(at_named["coxa_sweep_deg"], abs=1e-9)


def test_ratio_is_coxa_over_femur_and_d425_flags_below_one():
    """D425 clause 4: at 70.0098 / 33.6056 mm the ratio is 0.9206 and rows 12-13 understate."""
    k = Table(C.load(), theta2_nom_deg=70.0098)
    row, _, trace = D.sweep_point(k, 33.6056, 0.5)
    assert round(row["femur_coxa_ratio_swing_avg"], 4) == 0.9206
    assert trace["swing_duration_understated"] is True
    row60, _, trace60 = D.sweep_point(k, 60.0, 0.5)
    assert trace60["swing_duration_understated"] is False


def test_no_root_is_emitted_never_a_number():
    assert D._cobinding_root(150.0, 180.0, 90.5) is None
    assert D._cobinding_root(150.0, 180.0, -90.5) is None
    assert D._cobinding_root(150.0, 180.0, 30.0) == pytest.approx(150.0 - 90.0)


def test_ratio_is_none_when_the_swing_has_no_lift():
    row, _, trace = D.sweep_point(Table(C.load(), swing_clearance_mm=0.5), 60.0, 0.5)
    assert row["femur_travel_swing_deg"] <= 0.0
    assert row["femur_coxa_ratio_swing_avg"] is None
    assert trace["swing_duration_understated"] is None


def test_the_dagger_mark_is_retired():
    """D418 clause 3."""
    assert not hasattr(D, "DEFINED_BY_ADOPTED_WITH_THE_SET")
    src = open(D.__file__, encoding="utf-8").read()
    assert "adopted with the set" not in src
