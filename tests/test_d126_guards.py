"""The three swing guards of D126, against the fifteen outputs. Named, not numbered
(D415 clause 5); the numbers in older docstrings below are historical.

    lift exists       femur_travel_swing_deg > 0
    ordering holds    theta_midswing_deg < theta_extreme_deg < theta_nom_deg
    span consistency  theta_span_deg recomputed FROM THE EMITTED TRACE

Guard 3 is the one that has to be wired the tedious way. Comparing the span
against two emitted angles tests nothing - it is the definition restated. It has
to be rebuilt from the trace's geometry.

theta_span_deg follows D100: nominal to MID-SWING, over the whole cycle
(COREDROP_21 §2). Until 16 September 2026 this file tested the stance-only span
(nominal to extreme), which is what the code then computed.
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
    """Rebuild theta_span_deg from the trace's raw geometry, not from emitted angles.
    D100's expression: nominal minus mid-swing, the foot lifted by the clearance."""
    k = C.load()
    row, _, trace = D.sweep_point(k, stride, duty)

    R = trace["rigid_len_mm"]
    theta_nom = trace["theta2_nom_deg"] + trace["psi_deg"]
    height = R * math.sin(math.radians(theta_nom))
    theta_mid = math.degrees(math.asin((height - trace["swing_clearance_mm"]) / R))

    assert math.isclose(theta_nom - theta_mid, row["theta_span_deg"], rel_tol=1e-12)


@pytest.mark.parametrize("stride,duty", CASES)
def test_stance_span_is_kept_as_a_diagnostic_and_differs(stride, duty):
    """The stance-only span survives in the trace, labelled, and is not output 6."""
    k = C.load()
    row, _, trace = D.sweep_point(k, stride, duty)
    assert math.isclose(trace["theta_span_stance_deg"],
                        row["theta_nom_deg"] - row["theta_extreme_deg"], rel_tol=1e-12)
    assert abs(trace["theta_span_stance_deg"] - row["theta_span_deg"]) > 1.0


def test_span_predicate_is_the_angle_ordering_it_stands_for():
    """COREDROP_21 §2.3: D100's span is the cycle peak-to-peak only while
    theta_mid <= theta_extreme, which is clearance >= bob. The flag must say exactly
    that, on both sides of the edge."""
    k = C.load()
    for clearance in (1.0, 2.0, 5.0, 15.0):
        row, _, trace = D.sweep_point(_Override(k, swing_clearance_mm=clearance), 60.0, 0.5)
        ordering = row["theta_midswing_deg"] <= row["theta_extreme_deg"]
        assert trace["span_predicate_holds"] == ordering == (clearance >= row["bob_mm"])
    flags = {D.sweep_point(_Override(k, swing_clearance_mm=c), 60.0, 0.5)[2]["span_predicate_holds"]
             for c in (0.5, 15.0)}
    assert flags == {True, False}, "the edge was never crossed, so the test had no power"


def test_the_withdrawn_d125_guard_is_absent():
    """D125 withdrew a guard asserting theta_span_deg and femur_travel_swing_deg are
    never equal. D211 gives the closed form for where they coincide, so this is
    solved for rather than pictured - which is the rule that produced the withdrawal."""
    k = C.load()
    c = D.coincidence_clearance_mm(k)
    d = D.derive(k)
    assert 0.0 < c < d.body_height_mm, (
        "the two quantities coincide at swing clearance {:.4f} mm, an ordinary "
        "geometry - equality is not a failure".format(c))


def test_coincidence_closed_form_matches_a_direct_search():
    """The closed form and a bisection on span == travel must agree.

    D211's closed form c = h0 - R*sin(2*Theta_extreme - Theta_0) sets
    Theta_mid = 2*Theta_e - Theta_0. With theta_span_deg as D100 defines it,
    span == travel is Theta_0 - Theta_mid = 2(Theta_e - Theta_mid), which is the
    same equation. D125's prose and D211's closed form are one point.

    HISTORY. Until 16 September 2026 output 6 was the stance span, under which the
    closed form corresponded to travel == 2 * span and the prose to a different
    clearance, 7.22287354 mm. This file recorded that as a discrepancy and filed it.
    COREDROP_21 §2 rules output 6 to D100's expression, and the discrepancy closes.
    """
    k = C.load()
    closed = D.coincidence_clearance_mm(k)

    def gap(clearance):
        row, _, _ = D.sweep_point(_Override(k, swing_clearance_mm=clearance), 60.0, 0.5)
        return row["femur_travel_swing_deg"] - row["theta_span_deg"]

    lo, hi = 1.0, D.derive(k).body_height_mm - 1.0
    assert gap(lo) * gap(hi) < 0
    for _ in range(200):
        mid = (lo + hi) / 2.0
        if gap(lo) * gap(mid) <= 0:
            hi = mid
        else:
            lo = mid
    assert math.isclose(closed, (lo + hi) / 2.0, abs_tol=1e-9)


def test_no_intermediate_rounding_before_a_multiplication():
    """D211, binding. Rounding theta_extreme to 4 dp before the 2x multiplier gives
    9.7483 and at 3 dp gives 9.7486; the true value is 9.74822449. Three errors have
    been produced in this one quantity by exactly this mistake."""
    legacy = _Legacy()
    exact = D.coincidence_clearance_mm(legacy)
    assert round(exact, 8) == 9.74822449
    assert round(exact, 4) == 9.7482

    d = D.derive(legacy)
    R, L1 = d.rigid_len_mm, legacy.value("coxa_length_mm")
    s_half = legacy.value("stride_mm") / 2.0
    te = math.degrees(math.acos((math.sqrt(d.r_nom_mm ** 2 + s_half ** 2) - L1) / R))
    pre_rounded = d.body_height_mm - R * math.sin(
        math.radians(2.0 * round(te, 4) - d.theta_nom_deg))
    assert round(pre_rounded, 4) == 9.7483
    assert round(pre_rounded, 4) != round(exact, 4)


class _Legacy(object):
    """D211's seed set: L1 = 50, L2 = 90, theta2_nom = 40, half-stride 30, clearance 15."""
    _v = dict(coxa_length_mm=50.0, femur_length_mm=90.0, tibia_length_mm=0.0,
              theta3_deg=0.0, theta2_nom_deg=40.0, swing_clearance_mm=15.0,
              stride_mm=60.0, mass_kg=2.15, dtheta_peak_deg_s=200.0,
              swing_velocity_profile="half_sine", tripod_support_legs=3,
              tau_servo_kgcm=20.0, margin_factor=2.5,
              command_step_deg=0.1350, joint_accuracy_deg=1.0000)

    def value(self, name):
        return self._v[name]


def test_the_prose_and_the_closed_form_are_one_point_under_d100():
    """The discrepancy this file used to record, closed on the arithmetic rather than
    deleted. Under D100's span, span == travel IS the closed form. The stance-span
    reading is kept as the CONTROL: it must still cross somewhere else, or this test
    could not tell the two readings apart."""
    legacy = _Legacy()
    closed = D.coincidence_clearance_mm(legacy)

    def bisect(f):
        lo, hi = 1.0, 40.0
        for _ in range(200):
            mid = (lo + hi) / 2.0
            if f(lo) * f(mid) <= 0:
                hi = mid
            else:
                lo = mid
        return (lo + hi) / 2.0

    def row_at(c):
        return D.sweep_point(_Override(legacy, swing_clearance_mm=c), 60.0, 0.5)[0]

    def trace_at(c):
        return D.sweep_point(_Override(legacy, swing_clearance_mm=c), 60.0, 0.5)[2]

    span_eq_travel = bisect(lambda c: row_at(c)["theta_span_deg"]
                            - row_at(c)["femur_travel_swing_deg"])
    stance_span_eq_travel = bisect(lambda c: trace_at(c)["theta_span_stance_deg"]
                                   - row_at(c)["femur_travel_swing_deg"])

    assert round(span_eq_travel, 8) == 9.74822449
    assert round(closed, 8) == round(span_eq_travel, 8)
    assert round(stance_span_eq_travel, 8) == 7.22287354
    assert abs(stance_span_eq_travel - span_eq_travel) > 2.0


def test_d211_angle_set():
    """The machine-verified set that REPLACES the register's. Every figure is from
    D211, not from this module."""
    legacy = _Legacy()
    row, d, _ = D.sweep_point(legacy, 60.0, 0.5)
    assert round(row["theta_extreme_deg"], 4) == 36.1541
    assert round(row["theta_midswing_deg"], 4) == 28.4324
    assert round(row["coxa_sweep_deg"] / 2.0, 4) == 14.1559
    assert round(row["coxa_sweep_deg"], 4) == 28.3117
    assert round(row["femur_travel_swing_deg"], 4) == 15.4435
    assert round(row["bob_mm"], 4) == 4.7545


def test_d220_rate_figures():
    """D146 row 1 is 134.9164 mm/s under D220, not 134.93. D115: 3 m in 22.2360 s."""
    legacy = _Legacy()
    row, _, _ = D.sweep_point(legacy, 60.0, 0.5)
    _, _, trace = D.sweep_point(legacy, 60.0, 0.5)
    # body_speed_mm_s is a labelled convenience under D415 clause 3, not an output.
    assert round(trace["body_speed_mm_s"], 4) == 134.9164
    assert round(3000.0 / trace["body_speed_mm_s"], 4) == 22.2360


class _Override(object):
    def __init__(self, base, **kw):
        self._base, self._kw = base, kw

    def value(self, name):
        return self._kw[name] if name in self._kw else self._base.value(name)
