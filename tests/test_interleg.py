"""sim/interleg.py - D29's link-distance check.

The distance routine is checked against brute force, the cycle against the case D430
clause 4 names, and the boundary search against the distance it is supposed to invert.
"""

import math
import random

import pytest

from sim import constants as C
from sim import derive as D
from sim import interleg as I


class Table(object):
    def __init__(self, base, **kw):
        self._base, self._kw = base, kw

    def value(self, name):
        return self._kw[name] if name in self._kw else self._base.value(name)


def at(theta2):
    return Table(C.load(), theta2_nom_deg=theta2)


def test_segment_distance_against_brute_force():
    """Sampling both segments on a 200x200 grid can only ever be longer than the true
    minimum, so the routine must never exceed it - and must get close."""
    rng = random.Random(11)
    worst_over, worst_gap = 0.0, 0.0
    for _ in range(120):
        pts = [tuple(rng.uniform(-100, 100) for _ in range(3)) for _ in range(4)]
        brute = min(
            math.dist(tuple(pts[0][i] + t * (pts[1][i] - pts[0][i]) for i in range(3)),
                      tuple(pts[2][i] + u * (pts[3][i] - pts[2][i]) for i in range(3)))
            for t in [j / 200.0 for j in range(201)]
            for u in [j / 200.0 for j in range(201)])
        got = I.segment_distance(*pts)
        worst_over = max(worst_over, got - brute)
        worst_gap = max(worst_gap, brute - got)
    assert worst_over <= 1e-9
    assert worst_gap < 2.0


def test_segment_distance_on_cases_with_known_answers():
    assert I.segment_distance((0, 0, 0), (10, 0, 0), (0, 5, 0), (10, 5, 0)) == pytest.approx(5.0)
    assert I.segment_distance((0, 0, 0), (10, 0, 0), (5, -5, 3), (5, 5, 3)) == pytest.approx(3.0)
    assert I.segment_distance((0, 0, 0), (1, 0, 0), (5, 0, 0), (6, 0, 0)) == pytest.approx(4.0)
    # crossing segments touch: the clamp must not push them apart
    assert I.segment_distance((-5, 0, 0), (5, 0, 0), (0, -5, 0), (0, 5, 0)) == pytest.approx(0.0)


def test_the_closest_moment_is_the_hand_over_and_the_pair_is_swing_against_stance():
    """D430 clause 4's case: one swinging link against one standing link."""
    dist, where, phase = I.min_link_distance(at(80.0), 60.0)
    assert min(abs(phase - p) for p in (0.0, 0.5, 1.0)) < 1e-3
    left, right = where.split(" / ")
    poses = I.pose_at(at(80.0), D.derive(at(80.0)), 60.0, phase)
    swinging = {leg for leg in poses if leg in I.TRIPODS[1]}
    assert (left.split()[0] in swinging) != (right.split()[0] in swinging)
    assert 30.0 < dist < 60.0


def test_without_an_overshoot_the_cycle_minimum_is_at_a_hand_over_frame():
    """CORRECTED 22 September. This test's first docstring said the hand-over frame is 'a
    stance endpoint'. It is not: at phase 0 the lifting tripod is in the SWING model, which
    holds the mid-stance body height, so its femur reads 80.0000 where the stance extreme's
    is 78.7384 (see the test below). The assertion was always only this: the cycle minimum
    equals the minimum over the hand-over frames, both computed by this same model - it
    compared the model with itself (family F), and had no power over what it claimed."""
    k, d = at(80.0), D.derive(at(80.0))
    endpoints = min(I.link_distance_at(k, d, 60.0, p)[0] for p in (0.0, 0.5, 1.0))
    assert I.min_link_distance(k, 60.0)[0] == pytest.approx(endpoints, abs=1e-9)


@pytest.mark.parametrize("overshoot", [0.09, 2.20])
def test_the_overshoot_closes_the_gap_and_is_priced(overshoot):
    """D430 clause 4's 0.09-2.20 deg, priced in millimetres. The control is that zero
    overshoot gives a strictly larger distance."""
    k = at(80.0)
    plain = I.min_link_distance(k, 60.0)[0]
    with_overshoot = I.min_link_distance(k, 60.0, overshoot_deg=overshoot)[0]
    assert with_overshoot < plain
    assert plain - with_overshoot < 20.0


def test_distance_falls_as_the_stride_grows():
    k = at(80.0)
    got = [I.min_link_distance(k, s)[0] for s in (40.0, 60.0, 80.0, 100.0)]
    assert all(b < a for a, b in zip(got, got[1:]))


def test_the_width_boundary_inverts_the_distance():
    k = at(80.0)
    for width in (20.0, 40.0):
        stride = I.longest_stride_for_width(k, width, iterations=30)
        assert I.clearance_mm(k, stride, width) == pytest.approx(0.0, abs=1e-3)
        assert I.clearance_mm(k, stride + 0.5, width) < 0.0


def test_no_root_where_the_links_never_close_and_a_refusal_where_they_always_touch():
    assert I.longest_stride_for_width(at(80.0), 0.0) is None
    with pytest.raises(I.NoPose):
        I.longest_stride_for_width(at(80.0), 200.0)


def test_the_swing_side_carries_its_model_label():
    assert I.SWING_MODEL == "level-body, body-frame progress"


# ------------------------------------------------------------------ the solver and the budget

def test_bracketed_root_finds_a_root_known_in_closed_form():
    """cos(x) = x has the Dottie number as its only root; nothing in this repository
    produced it."""
    dottie = 0.7390851332151607
    f = lambda x: math.cos(x) - x
    passing, failing = I.bracketed_root(f, 0.0, 1.0, 1e-10)
    assert f(passing) > 0.0 >= f(failing)
    assert abs(passing - failing) <= 1e-10
    assert passing <= dottie <= failing or failing <= dottie <= passing
    with pytest.raises(ValueError):
        I.bracketed_root(f, 0.0, 0.5)             # no sign change: refuse, do not guess


@pytest.mark.parametrize("f,a,b", [(lambda x: x ** 3 - 2.0, 0.0, 4.0),
                                   (lambda x: math.exp(x) - 10.0, 0.0, 10.0)])
def test_the_illinois_step_is_what_beats_bisection_on_a_convex_function(f, a, b):
    """The root test above passes plain regula falsi too - its midpoint fallback still
    lands on the root, only slowly. What the Illinois step buys is SPEED, so that is what
    is tested: fewer evaluations than bisection needs for the same bracket and tolerance.
    Plain regula falsi takes 193 on the first case and does not converge on the second."""
    calls = [0]

    def counted(x):
        calls[0] += 1
        return f(x)

    I.bracketed_root(counted, a, b, 1e-10)
    assert calls[0] < math.ceil(math.log2((b - a) / 1e-10))


@pytest.mark.parametrize("width", [20.0, 40.81])
def test_bracketed_root_agrees_with_the_bisection_it_replaces(width):
    k = at(80.0)
    bisected = I.longest_stride_for_width(k, width)
    fast = I.bracketed_root(lambda s: I.clearance_mm(k, s, width), 1.0, 200.0, 1e-10)[0]
    assert fast == pytest.approx(bisected, abs=1e-8)


@pytest.mark.parametrize("theta2", [71.0, 80.0, 89.0])
def test_clearance_falls_as_the_overshoot_grows(theta2):
    """What the budget's solve rests on, checked on a grid rather than assumed."""
    k = at(theta2)
    got = [I.clearance_mm(k, 60.0, 40.81, overshoot_deg=0.25 * i) for i in range(17)]
    assert all(b < a for a, b in zip(got, got[1:]))


@pytest.mark.parametrize("theta2", [80.0, 89.0])
def test_overshoot_budget_lies_inside_a_brute_force_bracket(theta2):
    """A 0.01 deg scan can only bracket the root; the solve must land inside it."""
    k = at(theta2)
    budget = I.overshoot_budget_deg(k, 60.0, 40.81)
    grid = [0.01 * i for i in range(401)]
    first_fail = next(i for i, o in enumerate(grid) if I.clearance_mm(k, 60.0, 40.81, overshoot_deg=o) <= 0)
    assert grid[first_fail - 1] < budget <= grid[first_fail]
    assert I.clearance_mm(k, 60.0, 40.81, overshoot_deg=budget) > 0.0
    assert I.clearance_mm(k, 60.0, 40.81, overshoot_deg=budget + 1e-6) <= 0.0


def test_overshoot_budget_says_why_when_there_is_no_number():
    k = at(80.0)
    assert I.overshoot_budget_deg(k, 60.0, 52.0) == I.CONTACT_AT_ZERO
    assert I.overshoot_budget_deg(k, 30.0, 40.81) == "clear beyond 10.0000 deg"


def test_overshoot_budget_is_zero_where_d29_already_binds():
    k = at(80.0)
    stride = I.longest_stride_for_width(k, 40.81)
    assert 0.0 <= I.overshoot_budget_deg(k, stride, 40.81) < 1e-6


def test_the_overshoot_is_labelled_as_this_modules_quantity_not_p8s():
    assert "NOT P8" in I.OVERSHOOT_QUANTITY and "AT the contact instant" in I.OVERSHOOT_QUANTITY


def test_the_swing_models_lift_off_is_not_the_stance_extreme():
    """Recorded, not fixed: FINDING_23's 44.7711 mm is R2 in this module's swing model at
    lift-off against R3 at its true stance extreme. Both legs at their true extremes read
    45.0503 mm (sim/p8_path.py, and stance_pose_of here). The swing model holds the body
    at d.body_height_mm, 1.9162 mm above the ground at a stance extreme."""
    k, d = at(80.0), D.derive(at(80.0))
    lift_off = I.pose_at(k, d, 60.0, 0.0)["R2"]
    extreme = I.stance_pose_of(k, d, "R2", -30.0)
    assert round(lift_off[1] - extreme[1], 4) == round(61.8457 - 60.5841, 4)
    r3 = I.stance_pose_of(k, d, "R3", +30.0)
    def pair(a):
        la, lb = I.leg_links(k, d, "R2", *a), I.leg_links(k, d, "R3", *r3)
        return min(I.segment_distance(p[0], p[1], q[0], q[1])
                   for p in ((la[0], la[1]), (la[1], la[2])) for q in ((lb[0], lb[1]), (lb[1], lb[2])))
    assert round(pair(lift_off), 4) == 44.7711
    assert round(pair(extreme), 4) == 45.0503


def test_the_budget_at_a_solved_boundary_is_zero_on_either_side_of_zero():
    """At a stride solved to make the clearance zero its sign is noise; the budget there
    is 0.0 either way, and a refusal only where the links really overlap. Both sides are
    taken from the bracketed solve, and the test asserts that one really is below zero: a
    first version nudged a BISECTED boundary by 2.6e-13 mm, when the bisection itself only
    resolves 1.8e-10 mm - it never reached the other side, and control C16 showed it."""
    k = at(80.0)
    passing, failing = I.bracketed_root(lambda s: I.clearance_mm(k, s, 40.81), 1.0, 200.0, 1e-11)
    assert I.clearance_mm(k, passing, 40.81) > 0.0 >= I.clearance_mm(k, failing, 40.81) > -1e-9
    assert I.overshoot_budget_deg(k, passing, 40.81) == 0.0
    assert I.overshoot_budget_deg(k, failing, 40.81) == 0.0
    assert I.overshoot_budget_deg(k, passing + 0.01, 40.81) == I.CONTACT_AT_ZERO


# ------------------------------------------------------------------ the continuous swing model (D437 cl.8)

LEGS6 = ("R1", "R2", "R3", "L1", "L2", "L3")


def test_the_continuous_model_lifts_off_and_touches_down_at_the_stance_extremes():
    """Checked against stance_pose_of - built from each leg's own foothold, not from the
    swing model. The record's model misses by the 1.9162 mm bob (femur 80.0000 vs 78.7384)."""
    k, d = at(80.0), D.derive(at(80.0))
    for leg in LEGS6:
        for u, x in ((0.0, -30.0), (1.0, +30.0)):
            got = I.swing_pose_of(k, d, leg, u, 60.0, continuous=True)
            want = I.stance_pose_of(k, d, leg, x)
            assert got[0] == pytest.approx(want[0], abs=1e-9) and got[1] == pytest.approx(want[1], abs=1e-9)
    record = I.swing_pose_of(k, d, "R2", 0.0, 60.0)
    assert round(record[1] - d.psi_deg, 4) == 80.0000


def test_the_body_is_nominal_only_at_mid_stance():
    k, d = at(80.0), D.derive(at(80.0))
    assert I.body_height_at(k, d, 60.0, 0.5) == pytest.approx(d.body_height_mm, abs=1e-9)
    assert round(I.body_height_at(k, d, 60.0, 0.0), 4) == 157.4308
    assert round(d.body_height_mm - I.body_height_at(k, d, 60.0, 1.0), 4) == 1.9162


def test_on_the_continuous_model_the_cycle_minimum_at_80_60_is_pose_b():
    k = at(80.0)
    hand_over, _pair = I.handover_distance_mm(k, 60.0)
    assert round(hand_over, 4) == 45.0503
    assert I.min_link_distance(k, 60.0, continuous=True)[0] == pytest.approx(hand_over, abs=1e-9)
    assert round(I.min_link_distance(k, 60.0)[0], 4) == 44.7711        # the record, unchanged
