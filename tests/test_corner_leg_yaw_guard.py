"""The fifth guard: corner-leg yaw. PROJECT_06 §4.

D227 measures the front and rear coxae at radius 120.0409 mm and position angle
31.6374 deg while their beta_mount is 45.0000 - a 13.3626 deg difference. The
middle legs are radial to within 0.03 deg.

So a whole-body test that exercises R2 and L2 passes on a model that assumes legs
point radially outward from the body centre, and that model is wrong on four legs
of six. This guard is pointed at the corner legs specifically.

PROJECT_01 §5.3's warning, arriving as a real case: a guard that passes because it
was pointed at the easy case.
"""

import math

import pytest

from sim import constants as C
from sim import derive as D


def test_the_corner_legs_are_found_by_solving_not_by_listing():
    k = C.load()
    assert sorted(D.corner_legs(k)) == ["L1", "L3", "R1", "R3"]


def test_the_corner_offset_is_d227s_figure():
    k = C.load()
    for leg in ("R1", "R3", "L1", "L3"):
        x, y = k.value("coxa_positions_mm")[leg]
        offset = abs(k.value("beta_mount_deg")[leg] - math.degrees(math.atan2(y, x)))
        assert round(offset, 4) == 13.3626

    for leg in ("R2", "L2"):
        x, y = k.value("coxa_positions_mm")[leg]
        offset = abs(k.value("beta_mount_deg")[leg] - math.degrees(math.atan2(y, x)))
        assert offset < 0.03, "middle legs are radial; {} is not".format(leg)


class _Rotated(object):
    """beta_mount rotated by one degree on every leg."""

    def __init__(self, base, delta_deg):
        self._base = base
        self._beta = {leg: v + delta_deg
                      for leg, v in base.value("beta_mount_deg").items()}

    def value(self, name):
        return self._beta if name == "beta_mount_deg" else self._base.value(name)


def _feet(table):
    return {leg: D.foot_position_body(table, leg, 0.0, 40.0) for leg in D.LEG_ORDER}


def test_every_foot_moves_when_beta_mount_rotates():
    """The guard proper. If a foot does not move, the transform is not using
    beta_mount at all - which is the failure a radial model would produce."""
    k = C.load()
    before, after = _feet(k), _feet(_Rotated(k, 1.0))
    for leg in D.LEG_ORDER:
        moved = math.dist(before[leg], after[leg])
        assert moved > 1e-9, "{} did not move when beta_mount rotated".format(leg)


def test_rotating_beta_mount_moves_every_foot_by_the_same_distance():
    """Recorded because the first version of this guard asserted the opposite and
    was wrong. Every leg rotates about its OWN coxa position by the same delta, so
    the displacement MAGNITUDE is 2*reach*sin(delta/2) and is identical on all six.
    Magnitude cannot discriminate a radial model; direction can. The discriminating
    test is the next one."""
    k = C.load()
    before, after = _feet(k), _feet(_Rotated(k, 1.0))
    dist = [math.dist(before[leg], after[leg]) for leg in D.LEG_ORDER]
    assert max(dist) - min(dist) < 1e-9
    assert min(dist) > 1e-9


def test_the_radial_assumption_breaks_four_legs_and_leaves_two_intact():
    """THE GUARD. Replace beta_mount with each leg's own position angle - which is
    what a model assuming legs point radially outward from the body centre computes.

    The four corner feet move by more than a millimetre. The two middle feet do not
    move at all. So a whole-body test exercising only R2 and L2 passes on a model
    that is wrong on four legs of six."""
    k = C.load()
    radial = _Radial(k)

    moved = {leg: math.dist(D.foot_position_body(k, leg, 0.0, 40.0),
                            D.foot_position_body(radial, leg, 0.0, 40.0))
             for leg in D.LEG_ORDER}

    for leg in ("R1", "R3", "L1", "L3"):
        assert moved[leg] > 1.0, (
            "{} moved only {:.6f} mm under the radial assumption - the guard is not "
            "reaching the corner legs".format(leg, moved[leg]))
    for leg in ("R2", "L2"):
        assert moved[leg] < 1e-9, (
            "{} is a middle leg and is radial; it must be unaffected".format(leg))


class _Radial(object):
    """beta_mount replaced by each leg's own position angle: the wrong model."""

    def __init__(self, base):
        self._base = base
        self._beta = {}
        for leg in D.LEG_ORDER:
            x, y = base.value("coxa_positions_mm")[leg]
            self._beta[leg] = math.degrees(math.atan2(y, x))

    def value(self, name):
        return self._beta if name == "beta_mount_deg" else self._base.value(name)


@pytest.mark.parametrize("leg", ["R2", "L2"])
def test_a_middle_leg_only_test_would_pass_on_a_radial_model(leg):
    """Recorded so the reason for the guard cannot be lost. Replacing beta_mount
    with the leg's own position angle - the radial assumption - changes nothing on
    a middle leg and changes the corner legs by a measurable amount."""
    k = C.load()

    Radial = _Radial

    true_pos = D.foot_position_body(k, leg, 0.0, 40.0)
    radial_pos = D.foot_position_body(Radial(k), leg, 0.0, 40.0)
    assert math.dist(true_pos, radial_pos) < 1e-9

    corner_true = D.foot_position_body(k, "R1", 0.0, 40.0)
    corner_radial = D.foot_position_body(Radial(k), "R1", 0.0, 40.0)
    assert math.dist(corner_true, corner_radial) > 1.0
