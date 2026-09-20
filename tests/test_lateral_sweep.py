"""tools/lateral_sweep.py - the search on D58's lateral footholds (D424 clause 7).

Checked against things this tool did not produce:
  * D424's own table (coordination's solver, not MP1's): bob, middle and corner coxa;
  * D425 clause 3's vertical totals and D426's six BODY-progress ratios;
  * each solved boundary re-evaluated through sim/derive.sweep_point and the joint
    windows - the closed forms are not trusted to check themselves;
  * a stride one micrometre past each boundary must fail a bar, or the boundary test
    could pass on a limit that was never binding;
  * the monotonicity the best-posture search rests on, on a grid.
"""

import csv
import math
import os

import pytest

from bindings.hexconfig import LEGS, hex_coxa, project_joint_limits
from sim import constants as C
from sim import derive as D
from tools import lateral_sweep as L

HERE = os.path.dirname(os.path.abspath(__file__))
COMMITTED = os.path.join(HERE, os.pardir, "reports", "d58_lateral_sweep.csv")
CURVE = os.path.join(HERE, os.pardir, "reports", "d427_stride_curve.csv")


def at(theta2):
    return L.at_posture(C.load(), theta2)


# D424's table: pair -> (bob / body-height p-p, middle coxa magnitude, corner excess at bn 0)
D424 = [
    ((80.0000, 52.1483), 1.4431, 11.5775, 9.3275),
    ((80.0000, 60.0000), 1.9162, 13.2629, 11.0129),
    ((77.1765, 42.7895), 1.0238, 9.0038, 6.7538),
    ((80.0000, 41.5304), 0.9121, 9.2661, 7.0161),
    ((85.0000, 38.9835), 0.7232, 9.7813, 7.5313),
    ((70.0098, 60.0000), 2.3276, 11.0495, 8.7995),
]


@pytest.mark.parametrize("pair,bob,middle,corner", D424)
def test_d424_table_reproduces(pair, bob, middle, corner):
    """D424's table, kept as a check even though D427 withdrew the envelope behind its
    corner column: the corner excess there is against D248's +/-47.25."""
    theta2, stride = pair
    row = L.evaluate(C.load(), theta2, stride, "check")
    assert round(row["bob_mm"], 4) == bob
    assert round(row["middle_coxa_from_mount_deg"], 4) == middle
    assert round(row["record: corner excess vs D248 @ beta_neutral 0"], 4) == corner


def test_d424_corner_signs():
    """D424: R1 and L3 run negative, R3 and L1 positive."""
    k = at(80.0)
    d = D.derive(k)
    for leg, sign in (("R1", -1), ("L3", -1), ("R3", 1), ("L1", 1)):
        lo, hi = L.coxa_range_deg(k, d, leg, 26.07415, beta_neutral_deg=0.0)
        assert math.copysign(1, 0.5 * (lo + hi)) == sign


D425 = [((70.0098, 33.6056), 2.6726), ((77.1765, 42.7895), 2.6474), ((80.0000, 41.5304), 2.4004),
        ((80.0000, 52.1483), 2.9315), ((80.0000, 60.0000), 3.4046), ((70.0098, 60.0000), 4.2759),
        ((85.0000, 38.9835), 1.9635)]


@pytest.mark.parametrize("pair,total", D425)
def test_d425_vertical_totals_reproduce(pair, total):
    assert round(L.vertical_total_mm(at(pair[0]), pair[1]), 4) == total


def test_d426_body_progress_ratios_reproduce():
    got = sorted(round(D.sweep_point(at(t), s, 0.5)[0]["femur_coxa_ratio_swing_avg"], 4)
                 for (t, s), _, _, _ in D424)
    assert got == sorted([1.7498, 1.4680, 1.1771, 1.1253, 1.0294, 1.8505])


# ---------------------------------------------------------------- the boundaries

def _bars(theta2, stride):
    """Each bar evaluated independently of the closed forms."""
    k = at(theta2)
    row = L.evaluate(C.load(), theta2, stride, "check")
    return {"torque": row["tau_x_margin_kgcm (D209, D420 cl.1)"] - k.value("tau_servo_kgcm"),
            "vertical": row["vertical_total_mm (D425 cl.3)"] - k.value("body_bob_budget_mm"),
            "coxa proxy": max(row["corner_coxa_from_mount_deg"] - L.HW_CORNER_DEG,
                              row["middle_coxa_from_mount_deg"] - L.HW_MIDDLE_DEG)}


# D427's re-derivation on Hardware's bounds, which coordination computed, not MP1.
D427 = [(71.6404, 70.4232), (75.0, 66.3273), (77.1765, 63.5875),
        (80.0, 59.9397), (85.0, 53.2465), (89.0, 47.7060)]


@pytest.mark.parametrize("theta2,stride", D427)
def test_d427_longest_strides_reproduce(theta2, stride):
    assert abs(L.stride_limits(at(theta2))[1] - stride) <= 2e-4


def test_d427_best_posture_reproduces():
    k = C.load()
    assert round(L.best_posture_deg(k, L.stride_zero_posture_deg(k), 89.9999), 4) == 71.6404


def test_the_shipped_stride_fails_the_corner_proxy_by_the_figure_d427_gives():
    """D427: at 80.0000 / 60.0000 the corner asks 58.2629 against 58.2500."""
    row = L.evaluate(C.load(), 80.0, 60.0, "check")
    assert round(row["corner_coxa_from_mount_deg"], 4) == 58.2629
    assert round(row["corner_coxa_margin_deg (stance only)"], 4) == -0.0129
    assert row["coxa_proxy_bar_passes"] is False


def test_finding_22_table_is_still_reachable_as_the_record():
    """D427 clause 2 withdrew D248's envelope as a bar. The table it produced must still
    reproduce, or the record is lost."""
    for theta2, stride in [(72.1958, 71.1233), (80.0, 61.1132), (89.0, 48.6400)]:
        assert abs(L.stride_limits(at(theta2), bars="d248")[1] - stride) <= 2e-4


def test_the_proxy_is_labelled_stance_only_and_the_rows_carry_d145():
    row = L.evaluate(C.load(), 80.0, 59.9397, "check")
    assert "STANCE ONLY" in row["coxa_bar_is_a_proxy"] and "0.09-2.20" in row["coxa_bar_is_a_proxy"]
    assert "LOWER BOUND" in row["coxa_bar_source"]
    assert "D145" in row["rows_12_13_profile"] and "1.0491-1.4148" in row["rows_12_13_profile"]


def test_d29_is_reported_beside_the_stride_and_its_cross_section_is_unsupplied():
    row = L.evaluate(C.load(), 80.0, 59.9397, "check")
    assert row["d29_minimum_at_handover"] is True
    assert "UNSUPPLIED" in row["d29_cross_section_mm"]
    for w in L.CROSS_SECTIONS_MM:
        assert row["d29_clearance_mm @ cross-section %.4f" % w] == pytest.approx(
            row["d29_min_link_distance_mm"] - w)


def test_the_stride_curve_is_a_curve_not_a_cap():
    """D428 clause 2, D64: the ceiling is whatever the bars give, uncapped by 60."""
    curve = L.stride_curve(C.load(), 70.5, 89.0)
    assert len(curve) > 20
    assert any(r["longest_stride_mm"] > 60.0 for r in curve)
    assert all(r["status"] == L.STATUS for r in curve)
    torque_rows = [r for r in curve if r["binding_bar"] == "torque"]
    assert torque_rows and all(r["theta2_nom_deg"] < 72.0 for r in torque_rows)


@pytest.mark.parametrize("theta2", [71.0, 71.6404, 75.0, 77.1765, 80.0, 85.0, 89.0])
def test_longest_stride_sits_exactly_on_its_binding_bar(theta2):
    _, longest, binding = L.stride_limits(at(theta2))
    at_limit = _bars(theta2, longest)
    for name, excess in at_limit.items():
        assert excess <= 1e-6, "%s fails at the solved boundary" % name
    assert any(abs(at_limit[b]) <= 1e-6 for b in binding)
    past = _bars(theta2, longest + 2e-3)
    assert any(v > 0 for v in past.values()), "nothing fails past the boundary: not binding"


def test_below_the_stride_zero_posture_no_stride_passes_torque():
    floor = L.stride_zero_posture_deg(C.load())
    assert round(floor, 4) == 70.0098
    assert L.half_stride_torque(at(floor + 1e-9)) == pytest.approx(0.0, abs=1e-3)
    assert L.half_stride_torque(at(floor - 0.01)) is None


def test_the_monotonicity_the_posture_search_rests_on():
    grid = [70.05 + 0.25 * i for i in range(int((89.9 - 70.05) / 0.25))]
    t = [L.half_stride_torque(at(p)) for p in grid]
    v = [L.half_stride_vertical(at(p)) for p in grid]
    c = [L.half_stride_coxa_proxy(at(p)) for p in grid]
    assert all(b > a for a, b in zip(t, t[1:]))
    assert all(b > a for a, b in zip(v, v[1:]))
    assert all(b < a for a, b in zip(c, c[1:]))


def test_the_solved_posture_is_a_maximum():
    k = C.load()
    best = L.best_posture_deg(k, L.stride_zero_posture_deg(k), 89.9999)
    peak = L.stride_limits(at(best))[1]
    for dp in (-0.5, -0.01, 0.01, 0.5):
        assert L.stride_limits(at(best + dp))[1] < peak


def test_clocking_brings_every_corner_inside_the_withdrawn_window():
    """D424 clause 6's arithmetic, kept as the record. PROJECT_25 §5 confirms the signs
    Mini Project read: c = -43.2000 for R1 and L3, +43.2000 for R3 and L1."""
    for r in L.run(C.load())[2]:
        assert r["record: corner excess vs D248 @ clocked 43.2"] == 0.0
        assert r["record: corner excess vs D248 @ beta_neutral 0"] > 5.0
    # SIGN CONVENTION. Here the command is the leg-frame yaw PLUS beta_neutral_deg, so
    # the clocking that brings R1 toward zero is +43.2000 in this file. PROJECT_25 §5
    # writes the same physical re-seat as c = -43.2000 for R1 and L3, +43.2000 for R3
    # and L1 - the opposite sign of the same rotation. What must hold either way is that
    # each corner's stance range ends up centred near 1.8000 degrees from zero.
    k, d = at(80.0), D.derive(at(80.0))
    for leg, sign in (("R1", 1.0), ("L3", 1.0), ("R3", -1.0), ("L1", -1.0)):
        assert math.copysign(1.0, L.clocked_beta_neutral(k, d, leg)) == sign
        lo, hi = L.coxa_range_deg(k, d, leg, 0.0, beta_neutral_deg=L.clocked_beta_neutral(k, d, leg))
        assert abs(abs(0.5 * (lo + hi)) - 1.8) < 1e-9


def test_every_row_is_interim_and_no_row_is_a_proposal():
    for r in L.run(C.load())[2]:
        assert r["status"] == "interim - swing and corner coxa open"
        assert "propos" not in r["case"] and "ruled" not in r["case"]


def test_committed_csv_is_reproduced(tmp_path):
    fresh = tmp_path / "lateral.csv"
    L.write_csv(L.run(C.load())[2], str(fresh))
    with open(COMMITTED, newline="") as a, open(str(fresh), newline="") as b:
        assert list(csv.reader(a)) == list(csv.reader(b))


def test_committed_stride_curve_is_reproduced(tmp_path):
    fresh = tmp_path / "curve.csv"
    L.write_csv(L.stride_curve(C.load(), 70.5, 89.0), str(fresh))
    with open(CURVE, newline="") as a, open(str(fresh), newline="") as b:
        assert list(csv.reader(a)) == list(csv.reader(b))
