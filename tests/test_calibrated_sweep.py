"""tools/calibrated_sweep.py - the stride curve on D431's bars (PROJECT_27 §1 items 1-2).

Checked against things this tool did not produce:
  * FINDING_23 §3.2's 44.7711 mm centre-line figure, and D431's subtraction on it;
  * coordination's three linearised consequences in D431 (64.17, 59.08, 1.80) - a
    straight-line estimate from a different method, which the solve must land near;
  * each solved boundary re-evaluated bar by bar through sim/derive.sweep_point, the
    record's closed forms and interleg.clearance_mm - not through stride_limits itself;
  * a stride two micrometres past each boundary must fail a bar;
  * the two shapes the knee and the no-maximum statement rest on, on a grid.
"""

import csv
import math
import os

import pytest

from sim import constants as C
from sim import derive as D
from sim import interleg as I
from tools import calibrated_sweep as S
from tools import lateral_sweep as L

HERE = os.path.dirname(os.path.abspath(__file__))
ROWS_CSV = os.path.join(HERE, os.pardir, "reports", "d431_rows.csv")
CURVE_CSV = os.path.join(HERE, os.pardir, "reports", "d431_stride_curve.csv")
W = S.CALIBRATED_WIDTH_MM


def at(theta2):
    return L.at_posture(C.load(), theta2)


def _csv_cells_that_differ(committed, fresh, limit=12):
    """Every differing cell as (row, column, committed, fresh) - so a failure on another
    machine names the cell. Both files are read as UTF-8, never the platform default."""
    with open(committed, newline="", encoding="utf-8") as fa, open(fresh, newline="", encoding="utf-8") as fb:
        a, b = list(csv.reader(fa)), list(csv.reader(fb))
    out = [("shape", "rows", len(a), len(b))] if len(a) != len(b) else []
    for i, (ra, rb) in enumerate(zip(a, b)):
        for j in range(max(len(ra), len(rb))):
            x = ra[j] if j < len(ra) else "<missing>"
            y = rb[j] if j < len(rb) else "<missing>"
            if x != y:
                out.append((i, (a[0][j] if j < len(a[0]) else j), x, y))
    return out[:limit]


@pytest.fixture(scope="module")
def swept():
    """run() and the curve, computed once for the module: about 40 s between them."""
    k = C.load()
    floor, knees, rows = S.run(k)
    curve = S.stride_curve(k, 70.5, 89.0)
    return floor, knees, rows, curve


# ---------------------------------------------------------------- the calibration

def test_the_calibration_is_reproduced_from_finding_23s_figure():
    """D431: 44.7711 mm centre-line (FINDING_23 §3.2) less Hardware's 3.96 mm solid to
    solid, at 80.0000 / 60.0000, R2 against R3, at the hand-over."""
    dist, where, phase = I.min_link_distance(at(80.0), 60.0)
    assert round(dist, 4) == 44.7711
    assert where == "R2 rigid / R3 rigid" and phase == 0.0
    assert round(44.7711 - 3.96, 2) == W
    assert round(dist - W, 2) == 3.96


def test_coordinations_linear_estimates_hold_to_their_own_rounding():
    """D431's arithmetic is linear interpolation on FINDING_23's tables, 'not a solve'.
    The solve must land within the two decimals coordination quoted."""
    k = at(80.0)
    assert abs(2.0 * S.half_stride_d29(k, W) - 64.17) < 0.01
    assert abs(I.longest_stride_for_width(k, W, overshoot_deg=2.20) - 59.08) < 0.01
    assert abs(I.overshoot_budget_deg(k, 60.0, W) - 1.80) < 0.01


def test_the_posed_bounds_sit_where_their_closed_forms_put_them():
    k = at(80.0)
    corner_s, middle_s = S.half_stride_posed_coxa(k)
    assert L.coxa_angles_from_mount_deg(k, corner_s)[1] == pytest.approx(S.POSED_CORNER_DEG, abs=1e-9)
    assert L.coxa_angles_from_mount_deg(k, middle_s)[0] == pytest.approx(S.POSED_MIDDLE_DEG, abs=1e-9)
    # D431: 'at 80.0000 / 60.0000 the corner now leaves 12.71' - 12.7171 to four places
    assert round(S.POSED_CORNER_DEG - L.coxa_angles_from_mount_deg(k, 30.0)[1], 4) == 12.7171


def test_the_d29_solve_agrees_with_the_bisection_in_interleg():
    for theta2 in (71.0, 80.0, 89.0):
        k = at(theta2)
        assert 2.0 * S.half_stride_d29(k, W) == pytest.approx(
            I.longest_stride_for_width(k, W, hi_mm=S.D29_HI_MM), abs=1e-8)


# ---------------------------------------------------------------- the boundaries

def _excess(theta2, stride):
    """Each bar evaluated independently of stride_limits: > 0 means it fails."""
    k = at(theta2)
    row = D.sweep_point(k, stride, k.value("duty_factor"))[0]
    middle, corner = L.coxa_angles_from_mount_deg(k, stride / 2.0)
    return {"torque": row["tau_femur_peak_kgcm"] * k.value("margin_factor") - k.value("tau_servo_kgcm"),
            "vertical": L.vertical_total_mm(k, stride) - k.value("body_bob_budget_mm"),
            "posed coxa": max(corner - S.POSED_CORNER_DEG, middle - S.POSED_MIDDLE_DEG),
            "D29": -I.clearance_mm(k, stride, W)}


@pytest.mark.parametrize("theta2", [71.0, 71.3531, 75.0, 80.0, 85.0, 89.0])
def test_longest_stride_sits_exactly_on_its_binding_bar(theta2):
    _, longest, binding = S.stride_limits(at(theta2))
    here = _excess(theta2, longest)
    for name, excess in here.items():
        assert excess <= 1e-6, "%s fails at the solved boundary" % name
    assert any(abs(here[b]) <= 1e-6 for b in binding)
    past = _excess(theta2, longest + 2e-3)
    assert any(v > 0 for v in past.values()), "nothing fails past the boundary: not binding"


def test_the_knee_is_where_torque_hands_over_to_d29(swept):
    knee = swept[1][W]
    assert S.stride_limits(at(knee - 0.01))[2] == ["torque"]
    assert S.stride_limits(at(knee + 0.01))[2] == ["D29"]
    assert swept[2][0]["binding_bar @ D29 width %.4f" % W] == "torque + D29"


def test_the_knee_gap_rises_through_zero_once():
    """What the knee's solve rests on: torque minus the rest, rising on a grid."""
    k = C.load()
    gaps = [S.knee_gap_mm(k, 70.5 + i, W) for i in range(19)]
    assert all(b > a for a, b in zip(gaps, gaps[1:]))
    assert gaps[0] < 0.0 < gaps[-1]


def test_d29s_limit_rises_slowly_so_there_is_no_interior_maximum(swept):
    """Why the record's best-posture search does not carry over: its bar shrank with
    posture, this one does not."""
    knee = swept[1][W]
    grid = [knee + (89.0 - knee) * i / 8.0 for i in range(9)]
    limits = [S.half_stride_d29(at(p), W) for p in grid]
    assert all(b > a for a, b in zip(limits, limits[1:]))
    assert 2.0 * (limits[-1] - limits[0]) < 1.0


# ---------------------------------------------------------------- the rows and the curve

def test_the_envelope_widths_are_never_looser_and_tighter_where_d29_binds(swept):
    """Where torque binds all three widths, the three strides are the same number (70.5000
    deg); a first version of this test asserted strict order everywhere and was wrong."""
    for r in swept[3]:
        a, b, c = (r["longest_stride_mm @ width %.4f" % w] for w in S.WIDTHS_MM)
        assert c <= b <= a
        if r["binding_bar @ width %.4f" % S.ENVELOPE_WIDTHS_MM[0]] == "D29":
            assert c < b
        if r["binding_bar @ width %.4f" % W] == "D29":
            assert b < a
        for w in S.ENVELOPE_WIDTHS_MM:
            assert r["overshoot_budget_deg @ shipped 60.0000, width %.4f" % w] == I.CONTACT_AT_ZERO


def test_every_minimum_is_the_hand_over_r2_against_r3(swept):
    for r in swept[2]:
        assert r["d29_minimum_at_handover"] is True
        assert r["d29_closest_links"] == "R2 rigid / R3 rigid"


def test_the_budget_at_the_shipped_stride_carries_its_predicate(swept):
    rows = {r["theta2_nom_deg"]: r for r in swept[3]}
    assert rows[70.5]["shipped_60.0000_passes_torque_vertical_posed_coxa"] is False
    assert rows[80.0]["shipped_60.0000_passes_torque_vertical_posed_coxa"] is True


def test_every_row_carries_its_labels_and_none_is_a_proposal(swept):
    for r in swept[2]:
        assert r["status"] == S.STATUS and "no pair" in r["status"]
        assert "calibrated at one configuration" in r["d29_width_label"]
        assert "wiring allowance owed" in r["d29_width_label"]
        for words in ("posed", "swing not included", "wiring not modelled", "provisional"):
            assert words in r["coxa_bar_source"]
        assert "NOT P8" in r["overshoot_quantity"]
        assert "propos" not in r["case"] and "ruled" not in r["case"]


def test_committed_rows_csv_is_reproduced(swept, tmp_path):
    fresh = tmp_path / "rows.csv"
    S.write_csv(swept[2], str(fresh))
    diffs = _csv_cells_that_differ(ROWS_CSV, str(fresh))
    assert not diffs, "cells that differ (row, column, committed, fresh): %r" % (diffs,)


def test_committed_curve_csv_is_reproduced(swept, tmp_path):
    fresh = tmp_path / "curve.csv"
    S.write_csv(swept[3], str(fresh))
    diffs = _csv_cells_that_differ(CURVE_CSV, str(fresh))
    assert not diffs, "cells that differ (row, column, committed, fresh): %r" % (diffs,)


# ---------------------------------------------------------------- platform stability

def test_a_row_on_the_boundary_prints_the_same_on_either_side_of_zero(swept):
    """A stride solved to make D29's clearance zero leaves it at +/- noise, and a printed
    cell must not depend on that sign. BOTH sides are taken from the bracketed solve at the
    knee, and the test asserts one really is below zero. A first version pushed the knee's
    stride by 2.6e-13 mm and assumed that crossed zero: here it did (+7e-15 mm), on jq's
    Windows it did not (+7.6e-11 mm, where torque sets the knee's stride) - the premise
    depended on the platform, and the test failed there."""
    k, knee = C.load(), swept[1][W]
    kk = at(knee)
    clear = lambda s: I.clearance_mm(kk, s, W)
    passing, failing = I.bracketed_root(clear, 1.0, S.D29_HI_MM, 1e-11)
    assert clear(passing) > 0.0 >= clear(failing) > -S.D29_ZERO_TOL_MM
    here = S.evaluate(k, knee, passing, "boundary")
    past = S.evaluate(k, knee, failing, "boundary")
    assert [S.fmt(v) for v in past.values()] == [S.fmt(v) for v in here.values()]


def test_every_committed_csv_is_ascii():
    """jq's Windows opens files as cp950 by default, where a UTF-8 section sign reads back
    as another character: that - not a tie - failed two CSVs there on 22 September."""
    import glob
    for path in sorted(glob.glob(os.path.join(HERE, os.pardir, "reports", "*.csv"))):
        with open(path, "rb") as fh:
            raw = fh.read()
        assert all(byte < 128 for byte in raw), "%s carries a non-ASCII byte" % os.path.basename(path)


def test_a_float_below_the_print_snap_prints_as_zero_with_no_sign():
    assert S.fmt(-7e-15) == S.fmt(7e-15) == S.fmt(0.0) == "0.0000"
    assert S.fmt(-1e-4) == "-0.0001" and S.fmt(True) == "yes"


# ---------------------------------------------------------------- D437 (PROJECT_29)

W37 = S.D437.widths[0]


@pytest.fixture(scope="module")
def swept_d437():
    k = C.load()
    floor, knees, rows = S.run(k, S.D437)
    curve = S.stride_curve(k, 70.5, 89.0, bars=S.D437)
    return floor, knees, rows, curve


def test_d437_widths_are_pose_c_less_hardwares_exact_gaps():
    """D437 cl.4: 46.5699 - 0.686 (worst pair) and 46.5699 - 1.192 (least conservative)."""
    from sim import p8_path as P
    pose_c = list(P.calibration_poses(P.at_posture(C.load())).values())[2][0]
    assert round(pose_c, 4) == 46.5699
    assert round(round(pose_c, 4) - 0.686, 4) == 45.8839 and round(45.8839, 2) == S.D437.widths[0]
    assert round(round(pose_c, 4) - 1.192, 4) == 45.3779 and round(45.3779, 2) == S.D437.widths[1]
    assert S.D437.widths[2:] == (51.0, 56.0) and S.D437.corner_deg == 71.06


def test_coordinations_hand_over_control_is_reproduced_two_ways():
    """PROJECT_29 sec.1: 59.1132 / 59.5821 / 59.6513 mm at 45.8839 / 45.4429 / 45.3779 mm,
    80.0000 deg, stance only. Reproduced stance only AND over the whole cycle on the
    continuous model - two routes that share only the link geometry."""
    k = at(80.0)
    for width, want in ((45.8839, 59.1132), (45.4429, 59.5821), (45.3779, 59.6513)):
        stance_only = S.handover_zero_stride_mm(k, width)
        assert round(stance_only, 4) == want
        whole = I.bracketed_root(lambda s: I.clearance_mm(k, s, width, continuous=True), 1.0, 200.0, 1e-10)[0]
        assert whole == pytest.approx(stance_only, abs=1e-8)


def test_d437_middle_bar_never_binds_and_the_corner_sits_on_its_closed_form():
    k = at(80.0)
    corner_s, middle_s = S.half_stride_posed_coxa(k, S.D437)
    assert middle_s == math.inf
    assert L.coxa_angles_from_mount_deg(k, corner_s)[1] == pytest.approx(71.06, abs=1e-9)


def _excess_d437(theta2, stride):
    k = at(theta2)
    row = D.sweep_point(k, stride, k.value("duty_factor"))[0]
    corner = L.coxa_angles_from_mount_deg(k, stride / 2.0)[1]
    return {"torque": row["tau_femur_peak_kgcm"] * k.value("margin_factor") - k.value("tau_servo_kgcm"),
            "vertical": L.vertical_total_mm(k, stride) - k.value("body_bob_budget_mm"),
            "posed coxa": corner - 71.06,
            "D29": -I.clearance_mm(k, stride, W37, continuous=True)}


@pytest.mark.parametrize("theta2", [71.0, 71.1521, 75.0, 80.0, 85.0, 89.0])
def test_d437_longest_stride_sits_exactly_on_its_binding_bar(theta2):
    _, longest, binding = S.stride_limits(at(theta2), bars=S.D437)
    here = _excess_d437(theta2, longest)
    assert all(v <= 1e-6 for v in here.values())
    assert any(abs(here[b]) <= 1e-6 for b in binding)
    assert any(v > 0 for v in _excess_d437(theta2, longest + 2e-3).values())


def test_d437_knee_hands_over_from_torque_to_d29(swept_d437):
    knee = swept_d437[1][W37]
    assert round(knee, 4) == 71.1521
    assert S.stride_limits(at(knee - 0.01), bars=S.D437)[2] == ["torque"]
    assert S.stride_limits(at(knee + 0.01), bars=S.D437)[2] == ["D29"]


def test_d437_rows_print_every_tie_as_a_tie(swept_d437):
    """No row may name one of four mirror-image pairs: which one 'wins' is the last bits."""
    for r in swept_d437[2]:
        assert "tie" in r["d29_closest_links"] and "rigid" not in r["d29_closest_links"]
    for r in swept_d437[3]:
        assert "tie" in r["d29_closest_links @ width %.4f" % W37]
    shipped = swept_d437[2][-1]
    assert round(shipped["d29_clearance_mm @ width 45.8800"], 4) == -0.8297
    assert shipped["d29_bar_passes @ width 45.8800 (clearance > -1e-9 mm)"] is False


def test_d437_no_printed_cell_moves_when_an_input_moves_by_a_few_ulps():
    class Nudge(object):
        def __init__(self, base, rel):
            self.base, self.rel = base, rel
        def value(self, name):
            v = self.base.value(name)
            return v * (1.0 + self.rel) if name == "coxa_length_mm" else v
    cells = lambda rows: [[S.fmt(v) for v in r.values()] for r in rows]
    base = S.stride_curve(C.load(), 88.0, 89.0, bars=S.D437)
    assert cells(S.stride_curve(Nudge(C.load(), 3e-15), 88.0, 89.0, bars=S.D437)) == cells(base)


def test_committed_d437_rows_csv_is_reproduced(swept_d437, tmp_path):
    fresh = tmp_path / "d437_rows.csv"
    S.write_csv(swept_d437[2], str(fresh))
    diffs = _csv_cells_that_differ(os.path.join(HERE, os.pardir, "reports", "d437_rows.csv"), str(fresh))
    assert not diffs, "cells that differ (row, column, committed, fresh): %r" % (diffs,)


def test_committed_d437_curve_csv_is_reproduced(swept_d437, tmp_path):
    fresh = tmp_path / "d437_curve.csv"
    S.write_csv(swept_d437[3], str(fresh))
    diffs = _csv_cells_that_differ(os.path.join(HERE, os.pardir, "reports", "d437_stride_curve.csv"), str(fresh))
    assert not diffs, "cells that differ (row, column, committed, fresh): %r" % (diffs,)


# ---------------------------------------------------------------- D439 (PROJECT_30)

W39 = S.D439.widths[0]


@pytest.fixture(scope="module")
def swept_d439():
    k = C.load()
    floor, knees, rows = S.run(k, S.D439)
    curve = S.stride_curve(k, 70.5, 89.0, bars=S.D439)
    return floor, knees, rows, curve


def test_d439_knee_hands_over_and_the_shipped_stride_touches(swept_d439):
    knee = swept_d439[1][W39]
    assert round(knee, 4) == 71.1567
    assert S.stride_limits(at(knee - 0.01), bars=S.D439)[2] == ["torque"]
    assert S.stride_limits(at(knee + 0.01), bars=S.D439)[2] == ["D29"]
    shipped = swept_d439[2][-1]
    assert round(shipped["d29_clearance_mm @ width 45.7700"], 4) == -0.7197
    assert shipped["d29_bar_passes @ width 45.7700 (clearance > -1e-9 mm)"] is False
    assert round(S.handover_zero_stride_mm(at(80.0), W39), 4) == 59.2343
    for r in swept_d439[2]:
        assert "tie" in r["d29_closest_links"] and "58.711" in r["d29_width_label"]


def test_committed_d439_csvs_are_reproduced(swept_d439, tmp_path):
    for rows, name in ((swept_d439[2], "d439_rows.csv"), (swept_d439[3], "d439_stride_curve.csv")):
        fresh = tmp_path / name
        S.write_csv(rows, str(fresh))
        diffs = _csv_cells_that_differ(os.path.join(HERE, os.pardir, "reports", name), str(fresh))
        assert not diffs, "%s: cells that differ (row, column, committed, fresh): %r" % (name, diffs)
