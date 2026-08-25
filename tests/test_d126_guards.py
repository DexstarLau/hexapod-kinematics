"""The three swing guards of D126, against the fourteen outputs.

    lift exists       output 7 > 0
    ordering holds    output 5 < output 4 < output 3
    span consistency  output 6 recomputed FROM THE EMITTED TRACE

Guard 3 is the one that has to be wired the tedious way. Comparing output 6
against output 3 minus output 4 tests nothing - it is the definition restated.
It has to be rebuilt from the trace's geometry.
"""

import math

import pytest

from sim import constants as C
from sim import derive as D

CASES = [(st, du) for st in (40.0, 60.0, 80.0) for du in (0.50, 0.55, 0.60)]


def rows():
    k = C.load()
    return [(st, du) + D.sweep_point(k, st, du) for st, du in CASES]


@pytest.mark.parametrize("stride,duty", CASES)
def test_lift_exists(stride, duty):
    k = C.load()
    row, _, _ = D.sweep_point(k, stride, duty)
    assert row["femur_travel_swing_deg"] > 0.0


@pytest.mark.parametrize("stride,duty", CASES)
def test_ordering_holds(stride, duty):
    k = C.load()
    row, _, _ = D.sweep_point(k, stride, duty)
    assert row["theta_midswing_deg"] < row["theta_extreme_deg"] < row["theta_nom_deg"]


@pytest.mark.parametrize("stride,duty", CASES)
def test_span_recomputed_from_the_trace(stride, duty):
    """Rebuild output 6 from the trace's raw geometry, not from outputs 3 and 4."""
    k = C.load()
    row, _, trace = D.sweep_point(k, stride, duty)

    R = trace["rigid_len_mm"]
    L1 = trace["coxa_length_mm"]
    theta_nom = trace["theta2_nom_deg"] + trace["psi_deg"]
    r_nom = L1 + R * math.cos(math.radians(theta_nom))
    s = trace["half_stride_mm"]
    theta_extreme = math.degrees(math.acos((math.sqrt(r_nom**2 + s**2) - L1) / R))

    assert math.isclose(theta_nom - theta_extreme, row["theta_span_deg"], rel_tol=1e-12)


def test_the_withdrawn_d125_guard_is_absent():
    """D125: 'theta_span_deg equals femur_travel_swing_deg' is NOT a failure
    condition. The two coincide at a swing clearance of 9.7482 mm, an entirely
    ordinary geometry, and in the collapse the guard was written to catch they are
    maximally unequal. Solved for, not pictured."""
    k = C.load()
    d = D.derive(k)
    R = d.rigid_len_mm

    def gap(clearance):
        row, _, _ = D.sweep_point(
            _Override(k, swing_clearance_mm=clearance), 60.0, 0.5)
        return row["theta_span_deg"] - row["femur_travel_swing_deg"]

    lo, hi = 1.0, min(d.body_height_mm - 1.0, R - 1.0)
    assert gap(lo) * gap(hi) < 0, "no crossing in range - cannot demonstrate the trap"
    for _ in range(200):
        mid = (lo + hi) / 2.0
        if gap(lo) * gap(mid) <= 0:
            hi = mid
        else:
            lo = mid
    crossing = (lo + hi) / 2.0
    assert 1.0 < crossing < d.body_height_mm, (
        "the two quantities coincide at swing clearance {:.4f} mm, which is an "
        "ordinary geometry - equality is not a failure".format(crossing))


class _Override(object):
    def __init__(self, base, **kw):
        self._base, self._kw = base, kw

    def value(self, name):
        return self._kw[name] if name in self._kw else self._base.value(name)
