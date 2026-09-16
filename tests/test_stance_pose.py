"""sim/stance_pose.py - Mini Project's own P1-prime (D414), checked four ways.

  1. against D414's recorded figures, which came from a different workstream's
     C harness - agreement between two derivations, not self-consistency;
  2. planting checked THROUGH A DIFFERENT FUNCTION: the solved joint angles go back
     through derive.foot_position_body and the body pose, and must land on the
     foothold. A solver that only drives its own residual to zero proves nothing;
  3. controls that give the checks power: a level body does NOT plant the feet, and
     at a vanishing stride the two swing measures differ by exactly the lift;
  4. what the per-tripod solve implies at the hand-over, recorded so it cannot be
     lost: the pose one tripod needs is not the pose the next one needs.
"""

import math

import pytest

from sim import constants as C
from sim import derive as D
from sim import stance_pose as S


class Table(object):
    def __init__(self, base, **kw):
        self._base, self._kw = base, kw

    def value(self, name):
        return self._kw[name] if name in self._kw else self._base.value(name)


def at(theta2, stride=60.0, **kw):
    return Table(C.load(), theta2_nom_deg=theta2, stride_mm=stride, **kw)


# D414 clauses 3, 4 and 6 (source: HANDOFF_C09_AI §§2-2.1). The C harness ran in
# float, so the last digit may differ by one; the tolerance is that and no wider.
@pytest.mark.parametrize("theta2,clearance", [(70.0098, -12.6688), (80.0, -2.4803)])
def test_d414_swing_clearance_at_shipped_stride(theta2, clearance):
    s = S.summarise(S.stance_cycle(at(theta2), 60.0))
    assert abs(s["min_swing_z_d414_mm"] - clearance) <= 2e-4


def test_d414_vertical_residual_at_the_lowest_posture():
    s = S.summarise(S.stance_cycle(at(70.0098), 60.0))
    assert round(s["height_pp_mm"], 4) == 2.5499


@pytest.mark.parametrize("theta2,stride", [(70.0098, 33.6056), (75.0, 41.6705), (80.0, 52.1483)])
def test_d414_longest_stride_reproduces(theta2, stride):
    found = S.longest_stride(at(theta2), "min_swing_z_d414_mm", iterations=40)
    assert abs(found - stride) <= 2e-4


def test_d414_clause5_pair():
    s = S.summarise(S.stance_cycle(at(80.0, 52.1483), 52.1483))
    assert round(s["theta2_max_deg"], 2) == 87.98
    assert round(s["height_pp_mm"], 4) == 1.3123


@pytest.mark.parametrize("theta2", [70.0098, 80.0, 89.0])
def test_feet_are_planted_through_an_independent_forward_kinematics(theta2):
    k = at(theta2)
    d = D.derive(k)
    cycle = S.stance_cycle(k, 60.0, frames=21)
    worst = 0.0
    for rec in cycle:
        pose = (rec["height_mm"], math.radians(rec["pitch_deg"]), math.radians(rec["roll_deg"]))
        for leg in S.TRIPODS[rec["tripod"]]:
            p_body = D.foot_position_body(k, leg, rec["theta1_" + leg], rec["theta2_" + leg])
            w = S.body_to_world(pose, rec["body_x_mm"], p_body)
            h = S.foothold_world(k, d, leg)
            worst = max(worst, math.dist(w, h))
    assert worst < 1e-6, "a stance foot moved {:.3e} mm".format(worst)


def test_control_a_level_body_does_not_plant_the_feet():
    """Without this, test_feet_are_planted could pass on a geometry where no pose
    change is ever needed."""
    k = at(70.0098)
    d = D.derive(k)
    level = (d.body_height_mm, 0.0, 0.0)
    worst = max(abs(S.surface_distance_mm(k, d, leg, S.world_to_body(level, -30.0, S.foothold_world(k, d, leg))))
                for leg in S.TRIPODS[0])
    assert worst > 1.0


def test_warm_start_and_cold_start_find_the_same_pose():
    """Newton is warm-started frame to frame. A second root would show up here."""
    k = at(70.0098)
    d = D.derive(k)
    cycle = S.stance_cycle(k, 60.0, frames=11)
    for rec in (cycle[3], cycle[8], cycle[14]):
        cold, _ = S.solve_pose(k, d, S.TRIPODS[rec["tripod"]], rec["body_x_mm"],
                               (d.body_height_mm, 0.0, 0.0))
        assert abs(cold[0] - rec["height_mm"]) < 1e-7
        assert abs(math.degrees(cold[1]) - rec["pitch_deg"]) < 1e-7
        assert abs(math.degrees(cold[2]) - rec["roll_deg"]) < 1e-7


def test_control_the_two_swing_measures_differ_by_the_lift_at_vanishing_stride():
    """At a stride too small to pitch the body, the D414 measure holds the full lift
    and the profile measure has zero lift at lift-off: they must read about 15 mm and
    about 0 mm. If both read the same, the measures are not the two readings named."""
    k = at(80.0, 0.001)
    s = S.summarise(S.stance_cycle(k, 0.001))
    clearance = k.value("swing_clearance_mm")
    assert abs(s["min_swing_z_d414_mm"] - clearance) < 1e-2
    assert abs(s["min_swing_z_profile_mm"]) < 1e-2


def test_handover_pose_jump_is_recorded_not_hidden():
    """FINDING_21. At the hand-over instant all six feet are down, and the pose that
    plants one tripod is not the pose that plants the other. Solved per tripod, the
    body pitch steps by its whole peak-to-peak in zero time. This test fails if that
    ever silently stops being true - e.g. if a transition is added and nobody says so."""
    s = S.summarise(S.stance_cycle(at(80.0), 60.0))
    assert s["handover_pitch_jump_deg"] > 1.0
    assert math.isclose(s["handover_pitch_jump_deg"], s["pitch_pp_deg"], rel_tol=1e-6)
    assert s["handover_height_jump_mm"] < 1e-6


def test_refuses_a_duty_factor_it_cannot_pair():
    with pytest.raises(ValueError):
        S.stance_cycle(at(80.0, duty_factor=0.55), 60.0)
