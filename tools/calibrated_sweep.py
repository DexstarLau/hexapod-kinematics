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


def _w(width_mm):
    return "%.4f" % width_mm


# --------------------------------------------------------------- per-bar limits

def half_stride_posed_coxa(k):
    """(corner, middle) half-stride limits from D431 clause 2's posed figures."""
    d = D.derive(k)
    corner = d.r_nom_mm * math.tan(math.radians(POSED_CORNER_DEG - 45.0))
    middle = d.r_nom_mm * math.tan(math.radians(POSED_MIDDLE_DEG))
    return corner, middle


def half_stride_d29(k, width_mm, lo_mm=1.0, hi_mm=D29_HI_MM, tol_mm=1e-10):
    """Half the longest stride whose D29 clearance at this width stays > 0.

    Solved on a bracket (interleg.bracketed_root) to tol_mm - the same boundary
    interleg.longest_stride_for_width bisects, in about a tenth of the evaluations, and
    checked against it by test. None means the links never close inside hi_mm: no root,
    never a number. A width already in contact at lo_mm raises, as the bisection does.
    """
    def clear(stride):
        return I.min_link_distance(k, stride, D29_FRAMES)[0] - width_mm

    if clear(hi_mm) > 0.0:
        return None
    if clear(lo_mm) <= 0.0:
        raise I.NoPose("links within {:.4f} mm even at {:.4f} mm stride".format(width_mm, lo_mm))
    return I.bracketed_root(clear, lo_mm, hi_mm, tol_mm)[0] / 2.0


def stride_limits(k, width_mm=CALIBRATED_WIDTH_MM):
    """Half-stride limit by bar, the longest stride, and the bar(s) that bind."""
    corner, middle = half_stride_posed_coxa(k)
    lim = OrderedDict([("torque", L.half_stride_torque(k)),
                       ("vertical", L.half_stride_vertical(k)),
                       ("posed coxa", min(corner, middle)),
                       ("D29", half_stride_d29(k, width_mm))])
    if lim["torque"] is None or lim["vertical"] is None:
        return lim, None, [n for n, v in lim.items() if v is None]
    live = {n: v for n, v in lim.items() if v is not None}
    s = min(live.values())
    binding = [n for n, v in live.items() if abs(v - s) <= 1e-9]
    return lim, 2.0 * s, binding


def shipped_passes_other_bars(k):
    """Whether the shipped 60.0000 mm clears torque, vertical and posed coxa at this
    posture - the predicate every 'budget at the shipped stride' figure needs beside it,
    because below the knee the shipped stride fails torque before D29 is even asked."""
    lim = stride_limits(k)[0]
    others = [lim[n] for n in ("torque", "vertical", "posed coxa")]
    return all(v is not None for v in others) and SHIPPED_STRIDE_MM / 2.0 <= min(others) + 1e-9


# --------------------------------------------------------------- the knee

def knee_gap_mm(k, theta2_nom_deg, width_mm=CALIBRATED_WIDTH_MM):
    """torque minus the smallest other bar, in half-stride mm. Negative: torque binds."""
    lim = stride_limits(L.at_posture(k, theta2_nom_deg), width_mm)[0]
    if lim["torque"] is None:
        return -1.0
    others = [v for n, v in lim.items() if n != "torque" and v is not None]
    return lim["torque"] - min(others)


def knee_posture_deg(k, width_mm=CALIBRATED_WIDTH_MM, hi_deg=89.9999):
    """The posture where torque hands over to the next bar, solved on a bracket.

    knee_gap_mm is negative while torque binds. bracketed_root wants f > 0 on the side
    it calls 'pass', so the gap is negated: the returned pair straddles the knee within
    POSTURE_TOL_DEG, and its midpoint is reported."""
    a, b = L.stride_zero_posture_deg(k) + 1e-9, hi_deg
    below, above = I.bracketed_root(lambda p: -knee_gap_mm(k, p, width_mm), a, b,
                                    POSTURE_TOL_DEG)
    return 0.5 * (below + above)


# --------------------------------------------------------------- one row

def evaluate(k, theta2_nom_deg, stride_mm, label):
    """One row: every bar, D29 at each width, the overshoot budgets, outputs by name."""
    kk = L.at_posture(k, theta2_nom_deg)
    row, _, trace = D.sweep_point(kk, stride_mm, kk.value("duty_factor"))

    out = OrderedDict()
    out["case"] = label
    out["status"] = STATUS
    out["theta2_nom_deg"] = theta2_nom_deg
    out["stride_mm"] = stride_mm
    for w in WIDTHS_MM:
        lim, longest, binding = stride_limits(kk, w)
        out["longest_stride_mm @ D29 width %s" % _w(w)] = longest
        out["binding_bar @ D29 width %s" % _w(w)] = " + ".join(binding)
        if w == CALIBRATED_WIDTH_MM:
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
    out["coxa_bar_source"] = POSED_LABEL
    out["corner_coxa_from_mount_deg"] = corner
    out["corner_coxa_bound_deg"] = POSED_CORNER_DEG
    out["corner_coxa_margin_deg (stance only)"] = POSED_CORNER_DEG - corner
    out["middle_coxa_from_mount_deg"] = middle
    out["middle_coxa_bound_deg"] = POSED_MIDDLE_DEG
    out["middle_coxa_note"] = MIDDLE_GRAZE_NOTE
    out["posed_coxa_bar_passes"] = (corner <= POSED_CORNER_DEG + 1e-9
                                    and middle <= POSED_MIDDLE_DEG + 1e-9)
    out["body_note"] = BODY_NOTE

    dist, where, phase = I.min_link_distance(kk, stride_mm)
    out["d29_min_link_distance_mm (centre-line)"] = dist
    out["d29_closest_links"] = where
    out["d29_phase"] = phase
    out["d29_minimum_at_handover"] = min(abs(phase - p) for p in (0.0, 0.5, 1.0)) < 1e-3
    out["d29_swing_model"] = I.SWING_MODEL + ", zero overshoot"
    out["d29_width_label"] = WIDTH_LABEL
    out["d29_envelope_label"] = ENVELOPE_LABEL
    for w in WIDTHS_MM:
        out["d29_clearance_mm @ width %s" % _w(w)] = dist - w
        out["d29_bar_passes @ width %s (clearance > -1e-9 mm)" % _w(w)] = dist - w > -D29_ZERO_TOL_MM
    out["overshoot_quantity"] = I.OVERSHOOT_QUANTITY
    for w in WIDTHS_MM:
        out["overshoot_budget_deg @ this stride, width %s" % _w(w)] = \
            I.overshoot_budget_deg(kk, stride_mm, w)
    out["shipped_60.0000_passes_torque_vertical_posed_coxa"] = shipped_passes_other_bars(kk)
    for w in WIDTHS_MM:
        out["overshoot_budget_deg @ shipped 60.0000, width %s" % _w(w)] = \
            I.overshoot_budget_deg(kk, SHIPPED_STRIDE_MM, w)

    for col in D.OUTPUT_COLUMNS:
        out[col] = row[col]
    out["swing_columns_model"] = D.SWING_MODEL_LABEL
    out["rows_12_13_profile"] = L.D145_LABEL
    out["swing_duration_understated (ratio < 1, D425 cl.6)"] = trace["swing_duration_understated"]
    return out


def stride_curve(k, lo_deg, hi_deg, step_deg=0.5):
    """The ceiling over posture at each width, D29's closest links at the calibrated
    width's longest stride, and the overshoot budgets. Uncapped by 60.0000 mm."""
    rows = []
    p = lo_deg
    while p <= hi_deg + 1e-9:
        kk = L.at_posture(k, p)
        r = OrderedDict([("theta2_nom_deg", p)])
        longest = None
        for w in WIDTHS_MM:
            _lim, s, binding = stride_limits(kk, w)
            r["longest_stride_mm @ width %s" % _w(w)] = s
            r["binding_bar @ width %s" % _w(w)] = " + ".join(binding)
            if w == CALIBRATED_WIDTH_MM:
                longest = s
        if longest is not None:
            dist, where, phase = I.min_link_distance(kk, longest)
            r["d29_closest_links @ width %s" % _w(CALIBRATED_WIDTH_MM)] = where
            r["d29_phase"] = phase
            r["overshoot_budget_deg @ own stride, width %s" % _w(CALIBRATED_WIDTH_MM)] = \
                I.overshoot_budget_deg(kk, longest, CALIBRATED_WIDTH_MM)
        r["shipped_60.0000_passes_torque_vertical_posed_coxa"] = shipped_passes_other_bars(kk)
        for w in WIDTHS_MM:
            r["overshoot_budget_deg @ shipped 60.0000, width %s" % _w(w)] = \
                I.overshoot_budget_deg(kk, SHIPPED_STRIDE_MM, w)
        r["status"] = STATUS
        if longest is not None:
            rows.append(r)
        p += step_deg
    return rows


def run(k):
    floor = L.stride_zero_posture_deg(k)
    knees = OrderedDict((w, knee_posture_deg(k, w)) for w in WIDTHS_MM)
    postures = [("sample", p) for p in SAMPLE_POSTURES_DEG]
    postures.append(("knee @ width %s" % _w(CALIBRATED_WIDTH_MM), knees[CALIBRATED_WIDTH_MM]))
    rows = []
    for label, p in sorted(postures, key=lambda lp: lp[1]):
        longest = stride_limits(L.at_posture(k, p))[1]
        rows.append(evaluate(k, p, longest, "%s, longest stride" % label))
    rows.append(evaluate(k, REPORTED_CASE[0], REPORTED_CASE[1], "reported case, shipped stride"))
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
    args = ap.parse_args(argv)
    k = C.load()
    floor, knees, rows = run(k)
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
        write_csv(stride_curve(k, math.ceil(floor * 2) / 2.0, 89.0), args.curve_csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
