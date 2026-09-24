"""The stride curve on D431's bars (PROJECT_27 §1, items 1-2): Hardware's POSED coxa
figures, and D29's link-distance check applied AS A BAR at the calibrated width.

    python -m tools.calibrated_sweep                          print the tables
    python -m tools.calibrated_sweep --csv PATH --curve-csv PATH

WHAT THIS REPLACES, AND WHAT IT LEAVES ALONE
--------------------------------------------
tools/lateral_sweep.py is FINDING_23's search and stays byte-reproducible as the record:
its coxa proxy is D427 clause 3's splayed figures, and it reports D29 parameterised
without applying it. D431 replaced both. This tool imports that module's closed forms
(torque, vertical, femur window, foothold geometry) unchanged and adds only what D431
changed. Nothing in the record's CSVs moves.

THE BARS
--------
  torque       D209 as D420 clause 1 rules it. Closed form, lateral_sweep.half_stride_torque.
  vertical     D425 clause 3. Closed form, lateral_sweep.half_stride_vertical.
  posed coxa   D431 clause 2: corner 70.9800 deg, middle 63.4400 deg from the mount -
               HANDOFF_C09_HW §5, STEP model, posed at 80.0000 / -30.0000, surface
               proximity, lower bound, swing not included, wiring not modelled,
               provisional. NOT written into joint_envelopes_deg. Measured at one posture
               and applied at every posture here - labelled so. On D58's footholds the
               corner angle is 45 + atan(s / r_nom), the middle atan(s / r_nom): each bound
               is a closed form in s.
  D29          D431 clause 3: clearance = centre-line distance (sim/interleg.py) - width,
               which must stay > 0 over the whole cycle. The width is 40.8100 mm =
               44.7711 - 3.96, CALIBRATED AT ONE CONFIGURATION (R2-R3, 80.0000 / 60.0000,
               stance extremes). The stride at which it reaches zero is bisected
               (interleg.longest_stride_for_width), not sampled. The same rows are
               reported at 52.0000 and 56.0000 mm, the parts' own envelopes, as the
               envelope bound. The swing side is the labelled level-body model at zero
               overshoot; its minimum is at the hand-over, where the swinging foot is
               still on the ground, so the bar is the stance-extremes figure - which is
               the configuration the calibration was measured in.
  femur        D248's window, a posture check, reported, never the stride bar.
  body         HANDOFF_C09_HW §5: no contact in +/-95 deg on any leg. Reported, not a bar.

The longest stride is 2 * min(torque, vertical, posed coxa, D29), and the bar that binds
is named.

THE KNEE, NOT A MAXIMUM
-----------------------
The record's best-posture search rests on the coxa limit SHRINKING as posture rises, so
that it meets the rising torque limit at an interior maximum. D29's limit does not
shrink: it is set by how far apart adjacent coxae sit along the body, which posture
barely changes, and it RISES slowly. There is then no interior maximum to solve for. What
is solved is the KNEE: the posture where torque stops binding,

    torque(p) = min(vertical, posed coxa, D29)(p),

by bisection to 1e-10 deg. Below it torque binds; above it the ceiling is D29's and
nearly flat. The two facts this rests on - torque minus the rest rises through zero
once, and D29's limit moves by under a millimetre above the knee - are checked on a grid
by test and stated, not proven.

THE OVERSHOOT BUDGET (item 2)
-----------------------------
Per row, interleg.overshoot_budget_deg at (a) the shipped stride 60.0000 mm and (b) the
row's own stride, at each width: the swing overshoot that uses the whole clearance. It is
in sim/interleg.py's parameterisation - the excursion placed AT the hand-over - and is
labelled NOT P8's quantity on every row (PROJECT_28 §1.2). Where D29 binds a row, (b) is
zero by construction.

Every row is interim; no pair is chosen here (D431 clause 9).
"""

import argparse
import csv
import math
import sys
from collections import OrderedDict

from bindings.hexconfig import LEGS, hex_coxa, project_joint_limits
from sim import constants as C
from sim import derive as D
from sim import interleg as I
from tools import lateral_sweep as L

STATUS = ("interim - width calibrated at one configuration; swing is the level-body "
          "model, not P8; no pair (D431 cl.9)")

# D431 clause 2 - HANDOFF_C09_HW §5 as PROJECT_27 carries it.
POSED_CORNER_DEG = 70.9800        # worst corner, L1; the others 71.41, 72.40, 73.62
POSED_MIDDLE_DEG = 63.4400        # R2, rear-going
# Every string that reaches a CSV is ASCII: jq's Windows opens files as cp950, where a UTF-8
# section sign reads back as another character (the failure of 22 September).
POSED_LABEL = ("Spider Hardware, STEP model, posed at 80.0000 / -30.0000, surface "
               "proximity, lower bound, swing not included, wiring not modelled, "
               "provisional (D431 cl.2); measured at 80.0000 and applied at every posture")
BODY_NOTE = "no contact in +/-95 deg on any leg (HANDOFF_C09_HW sec.5): reported, not a bar"
MIDDLE_GRAZE_NOTE = "<= 5 mm of clearance anywhere beyond 60 deg (Hardware's graze curve)"

# D431 clause 3. 44.7711 - 3.96 = 40.8111; the ruling quotes 40.81 because Hardware's
# figure carries two decimals. The ruled figure is used.
CALIBRATED_WIDTH_MM = 40.8100
ENVELOPE_WIDTHS_MM = (52.0000, 56.0000)
WIDTHS_MM = (CALIBRATED_WIDTH_MM,) + ENVELOPE_WIDTHS_MM
WIDTH_LABEL = ("calibrated at one configuration (R2-R3, 80.0000 / 60.0000 mm, stance "
               "extremes): FINDING_23's 44.7711 mm centre-line less Hardware's 3.96 mm "
               "solid-to-solid (D431 cl.3); wiring allowance owed, not assumed (cl.4)")
ENVELOPE_LABEL = "envelope bound: femur 56.00 mm, tibia 52.00 mm wide (D431 cl.3)"

SHIPPED_STRIDE_MM = 60.0
REPORTED_CASE = (80.0, SHIPPED_STRIDE_MM)
SAMPLE_POSTURES_DEG = (75.0, 77.1765, 80.0, 85.0, 89.0)
D29_HI_MM = 300.0
D29_FRAMES = 121
# The solvers' tolerance on a clearance. A clearance within it of zero is AT the boundary:
# its sign there is floating-point noise, and a printed cell must not depend on it.
D29_ZERO_TOL_MM = 1e-9
PRINT_SNAP = 5e-9          # a float this close to zero prints as 0.0000, never -0.0000
POSTURE_TOL_DEG = 1e-10


class Bars(object):
    """One ruled set of bars. D431 is FINDING_24's, kept so its CSVs reproduce; D437 is
    current. Every function below takes one; the default is the RECORD, so current work
    always names D437."""
    def __init__(self, **kw):
        self.__dict__.update(kw)


D431 = Bars(name="d431", status=STATUS, corner_deg=POSED_CORNER_DEG, middle_deg=POSED_MIDDLE_DEG,
            coxa_label=POSED_LABEL, middle_note=MIDDLE_GRAZE_NOTE, body_note=BODY_NOTE,
            widths=WIDTHS_MM, width_label=WIDTH_LABEL, envelope_label=ENVELOPE_LABEL,
            continuous=False)

# D437 (PROJECT_29) and D436 (HANDOFF_94) as PROJECT_29 carries them. ASCII only.
D437 = Bars(
    name="d437",
    status="interim - width calibrated at pose (c); pose (b) pending (D437 cl.8); no pair (D431 cl.9)",
    corner_deg=71.0600,
    middle_deg=95.0000,
    coxa_label=("Spider Hardware, STEP model, posed at 80.0000 / -30.0000, surface proximity, "
                "lower bound, swing not included, wiring not modelled, provisional (D436 cl.3); "
                "corner the worst of 71.06 / 71.91 / 71.41 / 71.38; measured at 80.0000 and "
                "applied at every posture"),
    middle_note=("no contact against a resting neighbour within +/-95 deg, at least 16.8 mm of "
                 "clearance beyond 60 deg (D436 cl.3): never binds - the middle coxa sits at "
                 "atan(s / r_nom) < 90 deg from its mount"),
    body_note="body sweep not re-run, INFERRED unchanged (D436 cl.3): reported, not a bar",
    widths=(45.8800, 45.3800, 51.0000, 56.0000),
    width_label=("calibrated at pose (c), 80.0000 deg, femurs 80.0000 deg, worst closing pair, "
                 "exact B-rep, wiring not modelled, one pose, provisional (D437 cl.4): "
                 "46.5699 - 0.686 = 45.8839, ruled 45.88; least conservative pair 46.5699 - "
                 "1.192 = 45.3779, reported as 45.38; pose (b) pending (D437 cl.6)"),
    envelope_label="envelope bound: femur 51.00 mm, tibia 56.00 mm wide (D436 cl.4)",
    continuous=True)
# D439 (HANDOFF_97, carried in PROJECT_30): D436's coxa bars, the width re-calibrated on the
# gait's own hand-over pose at 58.711 mm. ASCII only.
D439 = Bars(
    name="d439",
    status=("interim - width calibrated at 58.711 mm on the gait's hand-over pose; +/-0.15 mm near "
            "a boundary (D439 cl.3); no pair (D431 cl.9)"),
    corner_deg=D437.corner_deg, middle_deg=D437.middle_deg, coxa_label=D437.coxa_label,
    middle_note=D437.middle_note, body_note=D437.body_note,
    widths=(45.7700, 45.1400, 51.0000, 56.0000),
    width_label=("calibrated on the gait's hand-over pose at 80.0000 deg, stride 58.711 mm, worst "
                 "closing pair, exact B-rep, wiring not modelled, one stride, provisional (D439 cl.2): "
                 "46.2626 - 0.489 = 45.7736, ruled 45.77; least conservative pair 45.14; the "
                 "centre-line carries about +/-0.15 mm near a boundary (D439 cl.3); wiring allowance "
                 "owed and load-bearing (D439 cl.6)"),
    envelope_label=D437.envelope_label,
    continuous=True)
BAR_SETS = {"d431": D431, "d437": D437, "d439": D439}
TIE_MM = 1e-9


def _w(width_mm):
    return "%.4f" % width_mm


# --------------------------------------------------------------- per-bar limits

def half_stride_posed_coxa(k, bars=D431):
    """(corner, middle) half-stride limits from D431 clause 2's posed figures."""
    d = D.derive(k)
    corner = d.r_nom_mm * math.tan(math.radians(bars.corner_deg - 45.0))
    # A bound at or past 90 deg is no bound: on D58's footholds the middle coxa sits at
    # atan(s / r_nom) from its mount, below 90 deg for any stride. tan() past 90 deg is
    # NEGATIVE, and a first version fed D436's +/-95 deg through it (the curve raised).
    middle = (math.inf if bars.middle_deg >= 90.0
              else d.r_nom_mm * math.tan(math.radians(bars.middle_deg)))
    return corner, middle


def half_stride_d29(k, width_mm, lo_mm=1.0, hi_mm=D29_HI_MM, tol_mm=1e-10, continuous=False):
    """Half the longest stride whose D29 clearance at this width stays > 0.

    Solved on a bracket (interleg.bracketed_root) to tol_mm - the same boundary
    interleg.longest_stride_for_width bisects, in about a tenth of the evaluations, and
    checked against it by test. None means the links never close inside hi_mm: no root,
    never a number. A width already in contact at lo_mm raises, as the bisection does.
    """
    def clear(stride):
        return I.min_link_distance(k, stride, D29_FRAMES, continuous=continuous)[0] - width_mm

    if clear(hi_mm) > 0.0:
        return None
    if clear(lo_mm) <= 0.0:
        raise I.NoPose("links within {:.4f} mm even at {:.4f} mm stride".format(width_mm, lo_mm))
    return I.bracketed_root(clear, lo_mm, hi_mm, tol_mm)[0] / 2.0


def stride_limits(k, width_mm=None, bars=D431):
    """Half-stride limit by bar, the longest stride, and the bar(s) that bind."""
    width_mm = bars.widths[0] if width_mm is None else width_mm
    corner, middle = half_stride_posed_coxa(k, bars)
    lim = OrderedDict([("torque", L.half_stride_torque(k)),
                       ("vertical", L.half_stride_vertical(k)),
                       ("posed coxa", min(corner, middle)),
                       ("D29", half_stride_d29(k, width_mm, continuous=bars.continuous))])
    if lim["torque"] is None or lim["vertical"] is None:
        return lim, None, [n for n, v in lim.items() if v is None]
    live = {n: v for n, v in lim.items() if v is not None}
    s = min(live.values())
    binding = [n for n, v in live.items() if abs(v - s) <= 1e-9]
    return lim, 2.0 * s, binding


def shipped_passes_other_bars(k, bars=D431):
    """Whether the shipped 60.0000 mm clears torque, vertical and posed coxa at this
    posture - the predicate every 'budget at the shipped stride' figure needs beside it,
    because below the knee the shipped stride fails torque before D29 is even asked."""
    lim = stride_limits(k, bars=bars)[0]
    others = [lim[n] for n in ("torque", "vertical", "posed coxa")]
    return all(v is not None for v in others) and SHIPPED_STRIDE_MM / 2.0 <= min(others) + 1e-9


def _where(k, stride_mm, dist, where, phase, bars):
    """On the continuous model the cycle minimum IS the hand-over, pose (b), where the four
    closing pairs are mirror images and tie to about 1e-12 mm - which pair, and which of
    phases 0 and 0.5, 'wins' is decided by the last bits (L1-L2 here on first run). It is
    printed as the tie it is. The record's model is left exactly as FINDING_24 printed it."""
    if not bars.continuous:
        return where, phase
    if abs(dist - I.handover_distance_mm(k, stride_mm)[0]) <= TIE_MM:
        return "hand-over, pose (b): the four closing pairs tie", "0.0 or 0.5 (hand-over)"
    # Off the hand-over (at 89 deg the minimum moves into the swing) the four pairs still tie,
    # as mirror images - lift-off of one is touch-down of its mirror. The stable quantity is
    # how far from a hand-over it sits; which pair and which side are the last bits.
    off = min(abs(phase - h) for h in (0.0, 0.5, 1.0))
    return ("swing, %.4f of a cycle from a hand-over (tau %.4f after lift-off, or before "
            "touch-down): the four closing pairs tie as mirror images" % (off, 2.0 * off)), off


def handover_zero_stride_mm(k, width_mm, tol_mm=1e-10):
    """PROJECT_29 sec.4: the stride at which the hand-over clearance is zero - stance only,
    pose (b), no swing model. Coordination's control: 59.1132 mm at 45.8839 mm, 80 deg."""
    return I.bracketed_root(lambda s: I.handover_distance_mm(k, s)[0] - width_mm,
                            1.0, D29_HI_MM, tol_mm)[0]


# --------------------------------------------------------------- the knee

def knee_gap_mm(k, theta2_nom_deg, width_mm=None, bars=D431):
    """torque minus the smallest other bar, in half-stride mm. Negative: torque binds."""
    lim = stride_limits(L.at_posture(k, theta2_nom_deg), width_mm, bars)[0]
    if lim["torque"] is None:
        return -1.0
    others = [v for n, v in lim.items() if n != "torque" and v is not None]
    return lim["torque"] - min(others)


def knee_posture_deg(k, width_mm=None, hi_deg=89.9999, bars=D431):
    """The posture where torque hands over to the next bar, solved on a bracket.

    knee_gap_mm is negative while torque binds. bracketed_root wants f > 0 on the side
    it calls 'pass', so the gap is negated: the returned pair straddles the knee within
    POSTURE_TOL_DEG, and its midpoint is reported."""
    a, b = L.stride_zero_posture_deg(k) + 1e-9, hi_deg
    below, above = I.bracketed_root(lambda p: -knee_gap_mm(k, p, width_mm, bars), a, b,
                                    POSTURE_TOL_DEG)
    return 0.5 * (below + above)


# --------------------------------------------------------------- one row

def evaluate(k, theta2_nom_deg, stride_mm, label, bars=D431):
    """One row: every bar, D29 at each width, the overshoot budgets, outputs by name."""
    kk = L.at_posture(k, theta2_nom_deg)
    row, _, trace = D.sweep_point(kk, stride_mm, kk.value("duty_factor"))

    out = OrderedDict()
    out["case"] = label
    out["status"] = bars.status
    out["theta2_nom_deg"] = theta2_nom_deg
    out["stride_mm"] = stride_mm
    for w in bars.widths:
        lim, longest, binding = stride_limits(kk, w, bars)
        out["longest_stride_mm @ D29 width %s" % _w(w)] = longest
        out["binding_bar @ D29 width %s" % _w(w)] = " + ".join(binding)
        if w == bars.widths[0]:
            for name, v in lim.items():
                if name != "D29":
                    out["half_stride_limit_mm@" + name] = v
        out["half_stride_limit_mm@D29 width %s" % _w(w)] = lim["D29"]

    margin, servo = kk.value("margin_factor"), kk.value("tau_servo_kgcm")
    out["tau_x_margin_kgcm (D209, D420 cl.1)"] = row["tau_femur_peak_kgcm"] * margin
    out["torque_bar_passes"] = row["tau_femur_peak_kgcm"] * margin <= servo + 1e-9
    total = row["bob_mm"] + row["foot_dz_per_quantum_mm@joint_accuracy_deg"]
    out["vertical_total_mm (D425 cl.3)"] = total
    out["vertical_bar_passes"] = total <= kk.value("body_bob_budget_mm") + 1e-9
    f_lo, f_hi, _win, f_ok = L.femur_check(kk, stride_mm)
    out["femur_theta2_min_deg"], out["femur_theta2_max_deg"] = f_lo, f_hi
    out["femur_window_passes (D248, still in force)"] = f_ok

    middle, corner = L.coxa_angles_from_mount_deg(kk, stride_mm / 2.0)
    out["coxa_bar_source"] = bars.coxa_label
    out["corner_coxa_from_mount_deg"] = corner
    out["corner_coxa_bound_deg"] = bars.corner_deg
    out["corner_coxa_margin_deg (stance only)"] = bars.corner_deg - corner
    out["middle_coxa_from_mount_deg"] = middle
    out["middle_coxa_bound_deg"] = bars.middle_deg
    out["middle_coxa_note"] = bars.middle_note
    out["posed_coxa_bar_passes"] = (corner <= bars.corner_deg + 1e-9
                                    and middle <= bars.middle_deg + 1e-9)
    out["body_note"] = bars.body_note

    dist, where, phase = I.min_link_distance(kk, stride_mm, continuous=bars.continuous)
    out["d29_min_link_distance_mm (centre-line)"] = dist
    if bars.continuous:
        out["d29_handover_pose_b_mm (stance only)"] = I.handover_distance_mm(kk, stride_mm)[0]
    out["d29_closest_links"], out["d29_phase"] = _where(kk, stride_mm, dist, where, phase, bars)
    out["d29_minimum_at_handover"] = min(abs(phase - p) for p in (0.0, 0.5, 1.0)) < 1e-3
    out["d29_swing_model"] = (I.SWING_MODEL_CONTINUOUS if bars.continuous else I.SWING_MODEL) + ", zero overshoot"
    out["d29_width_label"] = bars.width_label
    out["d29_envelope_label"] = bars.envelope_label
    for w in bars.widths:
        out["d29_clearance_mm @ width %s" % _w(w)] = dist - w
        out["d29_bar_passes @ width %s (clearance > -1e-9 mm)" % _w(w)] = dist - w > -D29_ZERO_TOL_MM
    out["overshoot_quantity"] = I.OVERSHOOT_QUANTITY
    for w in bars.widths:
        out["overshoot_budget_deg @ this stride, width %s" % _w(w)] = \
            I.overshoot_budget_deg(kk, stride_mm, w, continuous=bars.continuous)
    out["shipped_60.0000_passes_torque_vertical_posed_coxa"] = shipped_passes_other_bars(kk, bars)
    for w in bars.widths:
        out["overshoot_budget_deg @ shipped 60.0000, width %s" % _w(w)] = \
            I.overshoot_budget_deg(kk, SHIPPED_STRIDE_MM, w, continuous=bars.continuous)

    for col in D.OUTPUT_COLUMNS:
        out[col] = row[col]
    out["swing_columns_model"] = D.SWING_MODEL_LABEL
    out["rows_12_13_profile"] = L.D145_LABEL
    out["swing_duration_understated (ratio < 1, D425 cl.6)"] = trace["swing_duration_understated"]
    return out


def stride_curve(k, lo_deg, hi_deg, step_deg=0.5, bars=D431):
    """The ceiling over posture at each width, D29's closest links at the calibrated
    width's longest stride, and the overshoot budgets. Uncapped by 60.0000 mm."""
    rows = []
    p = lo_deg
    while p <= hi_deg + 1e-9:
        kk = L.at_posture(k, p)
        r = OrderedDict([("theta2_nom_deg", p)])
        longest = None
        for w in bars.widths:
            _lim, s, binding = stride_limits(kk, w, bars)
            r["longest_stride_mm @ width %s" % _w(w)] = s
            r["binding_bar @ width %s" % _w(w)] = " + ".join(binding)
            if w == bars.widths[0]:
                longest = s
        if longest is not None:
            dist, where, phase = I.min_link_distance(kk, longest, continuous=bars.continuous)
            r["d29_closest_links @ width %s" % _w(bars.widths[0])], r["d29_phase"] = \
                _where(kk, longest, dist, where, phase, bars)
            r["overshoot_budget_deg @ own stride, width %s" % _w(bars.widths[0])] = \
                I.overshoot_budget_deg(kk, longest, bars.widths[0], continuous=bars.continuous)
        r["shipped_60.0000_passes_torque_vertical_posed_coxa"] = shipped_passes_other_bars(kk, bars)
        for w in bars.widths:
            r["overshoot_budget_deg @ shipped 60.0000, width %s" % _w(w)] = \
                I.overshoot_budget_deg(kk, SHIPPED_STRIDE_MM, w, continuous=bars.continuous)
        r["status"] = bars.status
        if longest is not None:
            rows.append(r)
        p += step_deg
    return rows


def run(k, bars=D431):
    floor = L.stride_zero_posture_deg(k)
    knees = OrderedDict((w, knee_posture_deg(k, w, bars=bars)) for w in bars.widths)
    postures = [("sample", p) for p in SAMPLE_POSTURES_DEG]
    postures.append(("knee @ width %s" % _w(bars.widths[0]), knees[bars.widths[0]]))
    rows = []
    for label, p in sorted(postures, key=lambda lp: lp[1]):
        longest = stride_limits(L.at_posture(k, p), bars=bars)[1]
        rows.append(evaluate(k, p, longest, "%s, longest stride" % label, bars))
    rows.append(evaluate(k, REPORTED_CASE[0], REPORTED_CASE[1], "reported case, shipped stride", bars))
    return floor, knees, rows


def fmt(v):
    """lateral_sweep.fmt, except that a float within PRINT_SNAP of zero prints 0.0000.

    WHY. On 22 September two committed CSVs failed to reproduce on Windows / Python 3.14
    while passing on Linux / 3.12: a clearance solved to zero came out +7e-15 on one and
    negative on the other. A printed cell must not depend on the last bits, so a value
    below any figure these tables resolve (4 decimals) is printed as the zero it is."""
    if isinstance(v, float) and not isinstance(v, bool) and abs(v) < PRINT_SNAP:
        v = 0.0
    return L.fmt(v)


def write_csv(rows, path):
    """UTF-8 always, never the platform default: on jq's Windows the default is cp950."""
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh, lineterminator="\n")
        w.writerow(list(rows[0]))
        for r in rows:
            w.writerow([fmt(v) for v in r.values()])


def main(argv=None):
    ap = argparse.ArgumentParser(description="The stride curve on D431's bars.")
    ap.add_argument("--csv")
    ap.add_argument("--curve-csv")
    ap.add_argument("--bars", choices=sorted(BAR_SETS), default="d439",
                    help="d439 current (default); d437 and d431 reproduce FINDING_25's and FINDING_24's records")
    args = ap.parse_args(argv)
    bars = BAR_SETS[args.bars]
    k = C.load()
    floor, knees, rows = run(k, bars)
    print("bars:", bars.name, "-", bars.status)
    for w in bars.widths:
        print("hand-over zero stride at 80.0000, width %s: %.4f mm (stance only, pose (b))"
              % (_w(w), handover_zero_stride_mm(L.at_posture(k, 80.0), w)))
    print(k.stamp())
    print("stride-zero torque limit: theta2_nom_deg %.4f" % floor)
    for w, p in knees.items():
        print("knee at D29 width %s mm:   theta2_nom_deg %.4f" % (_w(w), p))
    print()
    print("| quantity | " + " | ".join("%s / %s" % (fmt(r["theta2_nom_deg"]), fmt(r["stride_mm"]))
                                       for r in rows) + " |")
    print("|---|" + "---|" * len(rows))
    for key in list(rows[0])[2:]:
        print("| `%s` | " % key + " | ".join(fmt(r[key]) for r in rows) + " |")
    if args.csv:
        write_csv(rows, args.csv)
    if args.curve_csv:
        write_csv(stride_curve(k, math.ceil(floor * 2) / 2.0, 89.0, bars=bars), args.curve_csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
