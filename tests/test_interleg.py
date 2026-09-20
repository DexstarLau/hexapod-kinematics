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


def test_without_an_overshoot_the_cycle_and_the_stance_endpoints_agree():
    """Recorded, because it is the limit of what this model can show. On the labelled
    swing model the lift is zero at both hand-overs, so the closest moment is a stance
    endpoint and a stance-only check reads the same number. What a stance-only check
    cannot see is the OVERSHOOT, and this model only has one when it is given one."""
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
