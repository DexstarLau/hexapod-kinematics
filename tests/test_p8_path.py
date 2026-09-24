"""sim/p8_path.py - P8's path (HANDOFF_C16_AI §2, D433) and D29 swept along it.

Checked against things this module did not produce: HANDOFF_C16_AI's own constants and
tables (T1, T2, T3), MP1's stance geometry (interleg.stance_pose_of) on all six legs, the
phase convention of interleg.pose_at, and a brute-force grid for the minimum.
"""

import csv
import math
import os

import pytest

from sim import constants as C
from sim import derive as D
from sim import interleg as I
from sim import p8_path as P

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(HERE, os.pardir, "reports", "d433_p8_d29.csv")
LEGS = ("R1", "R2", "R3", "L1", "L2", "L3")


def k80():
    return P.at_posture(C.load())


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


def path(eps):
    return P.P8Path(k80(), P.STRIDE_MM, eps)


def test_constants_reproduce_handoff_c16_ai_section_2():
    p = path(0.120)
    got = [round(v, 6) for v in (p.theta1_lo, p.rho_c, p.w0, p.c, p.a, p.rho_mid)]
    assert got == [-13.262856, 130.765224, -25.588260, 4.381576, 15.608204, 150.755004]
    assert [round(path(e).W, 6) for e in P.EPS_SET] == [27.319326, 33.632163, 43.528414]


def test_overshoot_and_tau_star_reproduce():
    got = [tuple(round(v, 6) for v in path(e).overshoot_deg()) for e in P.EPS_SET]
    assert got == [(0.129968, 0.007369), (0.988860, 0.055638), (1.895887, 0.105748)]


# T1 rows: phase, theta2, theta1 @ 0.015 / 0.120 / 0.246
T1 = [(0.00, 78.7384, -13.2629, -13.2629, -13.2629), (0.02, 78.4489, -12.5669, -14.1450, -14.2622),
      (0.05, 77.6978, -10.9277, -13.4417, -15.1505), (0.12, 74.9099, -7.1030, -8.7444, -11.3174),
      (0.25, 71.1589, 0.0000, 0.0000, 0.0000), (0.45, 77.6978, 10.9277, 13.4417, 15.1505),
      (0.50, 78.7384, 13.2629, 13.2629, 13.2629), (0.75, 80.0000, 0.0000, 0.0000, 0.0000),
      (0.99, 78.8366, -12.7500, -12.7500, -12.7500)]


@pytest.mark.parametrize("row", T1)
def test_t1_rows_reproduce(row):
    phase, theta2 = row[0], row[1]
    for eps, want in zip(P.EPS_SET, row[2:]):
        p = path(eps)
        assert round(p.r2(phase)[0], 4) == want
        assert round(p.theta2_deg(phase), 4) == theta2


# T3 at eps 0.246: leg, end-phase, theta1 from mount, theta2, body yaw
T3 = [("R1", 0.55287, -60.1587, -105.1587), ("R1", 0.94713, -29.8413, -74.8413),
      ("R2", 0.05287, -15.1587, -105.1587), ("R2", 0.44713, 15.1587, -74.8413),
      ("R3", 0.55287, 29.8413, -105.1587), ("R3", 0.94713, 60.1587, -74.8413),
      ("L1", 0.05287, 60.1587, 105.1587), ("L1", 0.44713, 29.8413, 74.8413),
      ("L2", 0.55287, 15.1587, 105.1587), ("L2", 0.94713, -15.1587, 74.8413),
      ("L3", 0.05287, -29.8413, 103.3928 + 1.7659), ("L3", 0.44713, -60.1587, 74.8413)]


@pytest.mark.parametrize("leg,phase,from_mount,yaw", T3)
def test_t3_extremes_reproduce_on_every_leg(leg, phase, from_mount, yaw):
    """Spider AI's error 22 was a left leg swept the wrong way; T3 carries all six."""
    p = path(0.246)
    # T3 quotes the phase to five places, and the femur moves fast there (77.6078 at the
    # rounded phase, 77.6077 at the extreme): evaluate at the EXACT extreme, found from
    # tau*, and check that it rounds to T3's phase - a first version read the table's
    # rounded phase and failed in the fourth decimal of theta2.
    tau_star = p.overshoot_deg()[1]
    swing_start = 0.5 if leg in P.TRIPOD_A else 0.0
    exact = swing_start + (tau_star if phase % 0.5 < 0.25 else 1.0 - tau_star) / 2.0
    assert round(exact, 5) == phase
    got_yaw, theta, swinging = p.leg(leg, exact)
    mount = k80().value("beta_mount_deg")[leg]
    assert swinging
    assert round(got_yaw, 4) == round(yaw, 4)
    assert round(got_yaw - mount, 4) == from_mount
    assert round(theta - D.derive(k80()).psi_deg, 4) == 77.6077


def test_stance_half_is_mp1s_own_stance_geometry_on_six_legs():
    """Not the map checked against itself: interleg.stance_pose_of builds each foothold
    from the leg's own coxa position."""
    k, d = k80(), D.derive(k80())
    p = path(0.120)
    for leg in LEGS:
        start = 0.0 if leg in P.TRIPOD_A else 0.5
        for i in range(41):
            phase = start + 0.4999 * i / 40.0
            yaw, theta, swinging = p.leg(leg, phase)
            assert not swinging
            u = (2.0 * phase) % 1.0
            y2, t2 = I.stance_pose_of(k, d, leg, P.STRIDE_MM / 2.0 - u * P.STRIDE_MM)
            assert abs((yaw - y2 + 180.0) % 360.0 - 180.0) < 1e-9 and abs(theta - t2) < 1e-9


def test_swing_meets_stance_at_both_contacts_on_six_legs():
    for eps in P.EPS_SET:
        p = path(eps)
        for leg in LEGS:
            lift = 0.5 if leg in P.TRIPOD_A else 0.0
            for a, b in ((lift, lift - 1e-12), (lift + 0.5 - 1e-12, lift + 0.5)):
                ya, ta, _ = p.leg(leg, a % 1.0)
                yb, tb, _ = p.leg(leg, b % 1.0)
                assert abs(ya - yb) < 1e-7 and abs(ta - tb) < 1e-7


def test_phase_convention_is_interleg_pose_ats():
    k, d = k80(), D.derive(k80())
    p = path(0.015)
    for phase in (0.1, 0.25, 0.6, 0.9):
        ref = I.pose_at(k, d, P.STRIDE_MM, phase)
        standing_ref = {leg for g, tri in enumerate(I.TRIPODS) for leg in tri
                        if (phase < 0.5) == (g == 0)}
        assert {leg for leg in ref if not p.leg(leg, phase)[2]} == standing_ref


def test_mid_swing_world_lift_is_15_mm():
    p, d = path(0.246), D.derive(k80())
    assert d.body_height_mm - p.R * math.sin(math.radians(p.r2(0.25)[1])) == pytest.approx(15.0, abs=1e-9)


# ---------------------------------------------------------------- the calibration control

def test_the_control_coordination_asked_for_does_not_reproduce_44_7711():
    """PROJECT_28 §1.1 asked for a control reproducing FINDING_23's 44.7711 mm 'from this
    path at the stance extreme'. It cannot: at the stance extreme the path gives 45.0503,
    as MP1's own stance geometry does. 44.7711 is interleg's SWING model at lift-off, whose
    foot is held at the mid-stance body height - femur 80.0000, not the extreme's 78.7384."""
    k, d = k80(), D.derive(k80())
    cal = list(P.calibration_poses(k).values())
    assert [round(c, 4) for c, _ in cal] == [44.7711, 45.0503, 46.5699]
    assert round(P._closest(k, d, path(0.015), 0.0, pairs=(("R2", "R3"),))[0], 4) == 45.0503
    r2 = I.pose_at(k, d, P.STRIDE_MM, 0.0)["R2"]
    assert round(r2[1] - d.psi_deg, 4) == 80.0000
    assert round(I.stance_pose_of(k, d, "R2", -30.0)[1] - d.psi_deg, 4) == 78.7384


# ---------------------------------------------------------------- the sweep

@pytest.mark.parametrize("eps", P.EPS_SET)
def test_the_refined_minimum_is_not_beaten_by_a_denser_brute_force_grid(eps):
    k, d, p = k80(), D.derive(k80()), path(eps)
    refined = P.min_link_distance(k, p)[0]
    brute = min(P._closest(k, d, p, i / 40000.0)[0] for i in range(40001))
    assert refined <= brute + 1e-9
    assert brute - refined < 1e-3


def test_the_four_pairs_share_one_minimum():
    """HANDOFF_C16_AI §3: the left pairs mirror the right, half a cycle apart."""
    k, p = k80(), path(0.246)
    got = [P.min_link_distance(k, p, frames=1001, pairs=(pr,))[0] for pr in I.ADJACENT_PAIRS]
    assert max(got) - min(got) < 1e-6


def test_the_minimum_is_a_swinging_link_against_a_standing_one_and_passes_every_calibration():
    cal, rows = P.sweep_rows(C.load())
    for r in (r for r in rows if r["scope"] == "all four pairs"):
        assert "(swing)" in r["closest_links"] and "(stance)" in r["closest_links"]
        passes = [v for key, v in r.items() if key.startswith("passes @")]
        assert passes == [True, True, True, False, False]


def test_committed_csv_is_reproduced(tmp_path):
    from tools.calibrated_sweep import write_csv
    fresh = tmp_path / "p8.csv"
    write_csv(P.sweep_rows(C.load())[1], str(fresh))
    diffs = _csv_cells_that_differ(CSV_PATH, str(fresh))
    assert not diffs, "cells that differ (row, column, committed, fresh): %r" % (diffs,)


# ---------------------------------------------------------------- platform stability

class _Nudge(object):
    """The same table with one input moved by a relative rel - a few ulps, the size of
    the difference between two platforms' maths libraries."""
    def __init__(self, base, rel):
        self._base, self._rel = base, rel

    def value(self, name):
        v = self._base.value(name)
        return v * (1.0 + self._rel) if name == "coxa_length_mm" else v


@pytest.mark.parametrize("rel", [3e-15, -1e-13])
def test_no_printed_cell_moves_when_an_input_moves_by_a_few_ulps(rel):
    """The Windows / Python 3.14 failure of 22 September: the four pairs tie to about
    1e-12 mm, and the first version printed whichever won - a nudge of 3e-15 flipped it."""
    from tools.calibrated_sweep import fmt
    cells = lambda rows: [[fmt(v) for v in r.values()] for r in rows]
    assert cells(P.sweep_rows(_Nudge(C.load(), rel))[1]) == cells(P.sweep_rows(C.load())[1])


def test_the_overall_row_names_the_tie_rather_than_a_winner():
    rows = P.sweep_rows(C.load())[1]
    for r in (r for r in rows if r["scope"] == "all four pairs"):
        assert r["closest_links"].startswith("R1 rigid (swing) / R2 rigid (stance)")
        assert r["closest_links"].endswith("tie within 1e-9 mm: R1-R2, R2-R3, L1-L2, L2-L3")


# ---------------------------------------------------------------- D437 (PROJECT_29)

def test_d437_widths_predict_contact_on_p8s_path_at_every_width():
    """D437 cl.5, coordination's re-derivation: -0.84 / -0.88 / -1.03 mm at 45.8839 mm.
    A prediction on a width assumed constant between poses (c) and (b), not a bar."""
    rows = P.sweep_rows(C.load(), P.WIDTHS_D437, P.STATUS_D437)[1]
    overall = [r for r in rows if r["scope"] == "all four pairs"]
    for r, coord in zip(overall, (-0.84, -0.88, -1.03)):
        assert abs((r["d29_min_centreline_mm"] - 45.8839) - coord) < 0.005
        assert [v for key, v in r.items() if key.startswith("passes @")] == [False, False, False, False]
        assert r["status"] == P.STATUS_D437 and "pose (b) pending" in r["status"]


def test_committed_d437_csv_is_reproduced(tmp_path):
    from tools.calibrated_sweep import write_csv
    fresh = tmp_path / "d437_p8.csv"
    write_csv(P.sweep_rows(C.load(), P.WIDTHS_D437, P.STATUS_D437)[1], str(fresh))
    diffs = _csv_cells_that_differ(os.path.join(HERE, os.pardir, "reports", "d437_p8_d29.csv"), str(fresh))
    assert not diffs, "cells that differ (row, column, committed, fresh): %r" % (diffs,)


# ---------------------------------------------------------------- D439 (PROJECT_30)

def test_p8_at_58_mm_reproduces_coordinations_closed_form_constants():
    """PROJECT_30 sec.2's table. The lift a is RE-SOLVED at 58 mm (16.114149), not carried
    from 60 mm (15.608204) - the literal D439 cl.5 found in Hardware's p8path.py."""
    k = k80()
    p = P.P8Path(k, 58.0, 0.120)
    got = [round(v, 6) for v in (p.theta1_lo, p.rho_c, p.w0, p.c, p.a, p.rho_mid)]
    assert got == [-12.835644, 130.539433, -24.820960, 4.101421, 16.114149, 150.755004]
    assert round(p.theta2_deg(0.0), 4) == 78.8205
    d = D.derive(k)
    assert d.body_height_mm - p.R * math.sin(math.radians(p.r2(0.25)[1])) == pytest.approx(15.0, abs=1e-9)
    want = [(26.440205, 0.126137, 0.007374, 57.9618), (32.556594, 0.959701, 0.055671, 58.7953),
            (42.144885, 1.839954, 0.105807, 59.6756)]
    for eps, (W, over, tau, corner) in zip(P.EPS_SET, want):
        q = P.P8Path(k, 58.0, eps)
        o, t = q.overshoot_deg()
        assert (round(q.W, 6), round(o, 6), round(t, 6)) == (W, over, tau)
        assert round(45.0 - q.theta1_lo + o, 4) == corner


def test_d439_width_is_the_gait_hand_over_pose_less_hardwares_gap():
    """D439 cl.2: 46.2626 - 0.489 = 45.7736, ruled 45.77; cl.3: at 60 mm the instrument
    predicts -0.72 on the worst pair's width and -0.09 on the least conservative."""
    from tools import calibrated_sweep as S
    k = k80()
    centre = I.handover_distance_mm(k, 58.711)[0]
    assert round(centre, 4) == 46.2626
    assert round(round(centre, 4) - 0.489, 4) == 45.7736 and round(45.7736, 2) == S.D439.widths[0]
    assert S.D439.widths[1:] == (45.14, 51.0, 56.0) and P.WIDTHS_D439[0][1] == S.D439.widths[0]
    at60 = I.handover_distance_mm(k, 60.0)[0]
    assert round(at60 - 45.7736, 2) == -0.72 and round(at60 - 45.14, 2) == -0.09


def test_d439_path_minimum_at_58_mm_and_the_four_pairs_tie():
    """Coordination's 46.9308 / 46.8907 / 46.7447 mm; and the tie that lets one closing
    pair stand for all four in the stride solve."""
    k = k80()
    for eps, want in zip(P.EPS_SET, (46.9308, 46.8907, 46.7447)):
        path = P.P8Path(k, 58.0, eps)
        per = [P.min_link_distance(k, path, frames=1001, pairs=(pr,))[0] for pr in I.ADJACENT_PAIRS]
        assert round(P.min_link_distance(k, path)[0], 4) == want
        assert max(per) - min(per) < 1e-6


def test_d439_strides_reproduce_coordinations_controls():
    """At 45.7736 mm the whole path keeps 0 at 59.228 / 59.184 / 59.028 mm and +0.5 mm at
    58.697 / 58.654 / 58.499 mm of stride (coord_rederive_2026-09-23)."""
    for eps, zero, half in zip(P.EPS_SET, (59.228, 59.184, 59.028), (58.697, 58.654, 58.499)):
        assert round(P.path_stride_for_clearance(C.load(), 45.7736, eps, 0.0), 3) == zero
        assert round(P.path_stride_for_clearance(C.load(), 45.7736, eps, 0.5), 3) == half


def test_committed_d439_csvs_are_reproduced(tmp_path):
    from tools.calibrated_sweep import write_csv
    for rows, name in ((P.sweep_strides(C.load()), "d439_p8_d29.csv"), (P.strides_rows(C.load()), "d439_p8_strides.csv")):
        fresh = tmp_path / name
        write_csv(rows, str(fresh))
        diffs = _csv_cells_that_differ(os.path.join(HERE, os.pardir, "reports", name), str(fresh))
        assert not diffs, "%s: cells that differ (row, column, committed, fresh): %r" % (name, diffs)
