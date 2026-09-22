"""The (posture, stride) search on D58's LATERAL footholds (D424 clause 7, PROJECT_23 §4).

THE RECORD SINCE D431. This module is FINDING_23's search and stays byte-reproducible:
D431 replaced its coxa proxy with Hardware's posed figures and made D29 a bar at a
calibrated width. The search on those bars is tools/calibrated_sweep.py, which imports
the closed forms here unchanged.

    python -m tools.lateral_sweep                     print the tables
    python -m tools.lateral_sweep --csv PATH          also write the CSV they come from

THE FOOTHOLDS, AND WHY THE LEVEL-BODY ROWS DESCRIBE THIS GAIT
------------------------------------------------------------
D424: every stance foot sits BESIDE its coxa at lateral offset y0 = r_nom_mm, and moves
along x from +stride/2 to -stride/2. Every foot then keeps the same reach function,
sqrt(r_nom^2 + x^2), so one body height satisfies all six legs, the body stays level,
and both tripods agree at the hand-over. That reach is exactly the hypotenuse
sim/derive.py already uses, which is why derive's level-body rows are this gait's rows
(bob_mm equals the body-height peak-to-peak) - D424's own reading, re-checked here by
test against its six pairs. The swing columns rest on body-frame progress
(D426 clauses 1-2) and carry that label.

THE BARS, AND HOW EACH STRIDE LIMIT IS SOLVED (not sampled - family A)
---------------------------------------------------------------------
For a posture, each bar gives the largest half-stride s it admits:

  torque     D209 as D420 clause 1 rules it:
                 tau_femur_peak_kgcm * margin_factor <= tau_servo_kgcm
             tau = (mass_kg / tripod_support_legs) * a_eff_extreme / 10, and
             a_eff_extreme = sqrt(r_nom^2 + s^2) - L1, so in closed form
                 s <= sqrt((L1 + a_eff_max)^2 - r_nom^2)
             with a_eff_max = derive.a_eff_max_mm (never stored). Below the posture
             where r_nom = L1 + a_eff_max no stride passes: that is the stride-zero
             limit, not the bar (D420 clause 1).
  vertical   D425 clause 3:  bob_mm + foot_dz_per_quantum_mm@joint_accuracy_deg
                             <= body_bob_budget_mm
             bob = h - R*sin(Theta_ext), so sin(Theta_ext) >= (h - budget + dz)/R,
             which bounds the reach and hence s, in closed form.
  coxa       D427 clause 3's Hardware figures, as a PROXY for D29's check and labelled
  proxy      so: middle 52.1700 deg, corner 58.2500 deg from the mount, splayed-pose
             LOWER BOUNDS, provisional, not written into joint_envelopes_deg. On D58's
             footholds the corner angle is 45 + atan(s / r_nom), so the bound is a
             closed form in s. STANCE ONLY - it omits P8's swing overshoot, 0.09-2.20
             deg (D430 clause 4).
  D29        the real bar: minimum distance between adjacent leg LINKS across the whole
             cycle, hand-over included, at a configurable cross-section (sim/interleg.py).
             The cross-section is Hardware's and is UNSUPPLIED, so it is reported
             parameterised and is not applied as a bar.
  D248       withdrawn as a coxa bar by D427 clause 2. Kept reachable as bars="d248",
             which reproduces FINDING_22's table.
  femur      theta2 over the D99 points - nominal, stride extreme, mid-swing - inside
             the projected window on every leg. It does not depend on stride at all
             here (the largest theta2 is the nominal, the smallest the mid-swing), so it
             is a posture check, reported, and never the binding stride bar.

The longest stride is 2 * min(torque, vertical, middle coxa), and the binding bar is named.

THE BEST POSTURE, SOLVED
------------------------
Raising the posture shortens r_nom. The torque and vertical limits then GROW and the
middle-coxa limit SHRINKS, so the longest stride over posture sits where the smaller
of the two growing limits meets the shrinking one:  min(torque, vertical) = coxa.
That crossing is solved by bisection on posture. The three monotonicities it relies
on are checked on a grid by test and stated, not proven.

(A first version of this tool assumed the vertical limit shrinks with posture and
solved torque = min(vertical, coxa), landing on 71.6427 deg - a point where the longest
stride was still rising. Caught by reading its own table before any figure left.)

WHAT IS REPORTED, NOT APPLIED
-----------------------------
  corner coxa excess, two columns (D424 clauses 6-7): at beta_neutral_deg 0.0000, and
  at D31's 43.2000 clocking by leg. The clocking SIGN is this tool's reading - the
  sign that moves each corner's stance range toward zero, which reproduces D424's
  statement that the clocked ranges fall inside the window. HANDOFF_C12_AI §2.1, which
  signs it, is not held here.
  tau * 3.0000: a sensitivity column, labelled with the mass at which margin 2.5000
  gives the same ceiling (D420 clause 2), computed from the table, not typed.

Every row is labelled "interim - swing and corner coxa open" (PROJECT_23 §4 item 3).
No pair is chosen or proposed here.
"""

import argparse
import csv
import math
import sys
from collections import OrderedDict

from bindings.hexconfig import LEGS, hex_coxa, hex_femur, project_joint_limits
from sim import constants as C
from sim import derive as D
from sim import interleg as I
from sim import stance_pose as S

STATUS = "interim - swing and corner coxa open"

# D427 clause 3, carried in PROJECT_25 §3 item 2. NOT written into joint_envelopes_deg,
# which is D248's file: these are a separate bar with their own name and source.
HW_BAR_LABEL = ("Spider Hardware, STEP model, splayed pose, surface proximity, "
                "LOWER BOUND, provisional (D427 clause 3)")
HW_MIDDLE_DEG = 52.1700          # middle coxa, rear-going, worst leg of the pair
HW_CORNER_DEG = 58.2500          # corner coxa toward its neighbour, worst leg
PROXY_LABEL = ("coxa-angle PROXY for D29's link-distance check, STANCE ONLY; it omits "
               "the swing overshoot, 0.09-2.20 deg at the pairs Spider AI measured "
               "(D430 clause 4, HANDOFF_87)")
D145_LABEL = ("rows swing_duration_ms and cycle_duration_ms assume D145's half-sine; "
              "P8's peak/mean is 1.0491-1.4148 against pi/2, flagged not adopted "
              "(D430 clause 5, HANDOFF_87)")
# D29's second check - the physical link cross-section - is Hardware's and is UNSUPPLIED,
# so the link-distance bar is run parameterised at these widths and at none of them ruled.
CROSS_SECTIONS_MM = (10.0, 20.0, 30.0, 40.0, 50.0)
# D430 clause 4's overshoot range, priced in millimetres of link clearance. The
# MAGNITUDE is D430's; the shape is sim/interleg.py's parameterisation, labelled there.
OVERSHOOT_DEG = (0.09, 2.20)
D29_FRAMES = 121
SAMPLE_POSTURES_DEG = (75.0, 77.1765, 80.0, 85.0, 89.0)
REPORTED_CASE = (80.0, None)            # (posture, stride); None -> the table's stride_mm
CLOCK_DEG = 43.2                        # D31: three teeth on a 25T spline
MIDDLE_LEGS = ("R2", "L2")
CORNER_LEGS = ("R1", "R3", "L1", "L3")
TOL_MM = 1e-10


class _Override(object):
    def __init__(self, base, **kw):
        self._base, self._kw = base, kw

    def value(self, name):
        return self._kw[name] if name in self._kw else self._base.value(name)


def at_posture(k, theta2_nom_deg):
    return _Override(k, theta2_nom_deg=theta2_nom_deg)


# --------------------------------------------------------------- foothold geometry

def lateral_foot_body(k, d, leg, x_mm):
    """D424's foothold: beside the coxa at y0 = r_nom outward, offset x along the body."""
    px, py = k.value("coxa_positions_mm")[leg]
    side = 1.0 if py > 0 else -1.0
    return (px + x_mm, py + side * d.r_nom_mm, -d.body_height_mm)


def coxa_range_deg(k, d, leg, half_stride_mm, beta_neutral_deg=None):
    """(min, max) commanded coxa angle over the stance, x from -s to +s.

    beta_neutral_deg None uses the table's value; a number replaces it for that leg."""
    table = k if beta_neutral_deg is None else _Override(
        k, beta_neutral_deg=dict(k.value("beta_neutral_deg"), **{leg: beta_neutral_deg}))
    a = S.joint_angles(table, d, leg, lateral_foot_body(k, d, leg, +half_stride_mm))[0]
    b = S.joint_angles(table, d, leg, lateral_foot_body(k, d, leg, -half_stride_mm))[0]
    return min(a, b), max(a, b)


def clocked_beta_neutral(k, d, leg):
    """This tool's reading of D31's sign: move the stance range's centre toward zero."""
    lo, hi = coxa_range_deg(k, d, leg, 0.0, beta_neutral_deg=0.0)
    centre = 0.5 * (lo + hi)
    return -math.copysign(CLOCK_DEG, centre)


def window_excess_deg(lo, hi, win):
    return max(0.0, win[0] - lo, hi - win[1])


# --------------------------------------------------------------- per-bar limits

def half_stride_torque(k):
    d = D.derive(k)
    reach_max = k.value("coxa_length_mm") + D.a_eff_max_mm(k)
    if reach_max < d.r_nom_mm:
        return None                      # below the stride-zero limit: no stride passes
    return math.sqrt(reach_max ** 2 - d.r_nom_mm ** 2)


def vertical_total_mm(k, stride_mm):
    row, _, _ = D.sweep_point(k, stride_mm, k.value("duty_factor"))
    return row["bob_mm"] + row["foot_dz_per_quantum_mm@joint_accuracy_deg"]


def half_stride_vertical(k):
    d = D.derive(k)
    R, L1 = d.rigid_len_mm, k.value("coxa_length_mm")
    dz = d.a_eff_nom_mm * k.value("joint_accuracy_deg") * math.pi / 180.0
    budget = k.value("body_bob_budget_mm")
    if dz >= budget:
        return None
    sin_min = (d.body_height_mm - budget + dz) / R
    if sin_min <= 0.0:
        cos_max = 1.0                     # bob cannot exhaust the budget before reach runs out
    else:
        cos_max = math.sqrt(1.0 - sin_min ** 2)
    reach = L1 + R * cos_max
    return math.sqrt(max(0.0, reach ** 2 - d.r_nom_mm ** 2))


def half_stride_coxa_proxy(k):
    """Hardware's coxa figures as the proxy bar (D427 clauses 3-4).

    On D58's footholds the corner leg's angle from its mount is 45 + atan(s / r_nom) and
    the middle leg's is atan(s / r_nom), so each bound is a closed form in s. The corner
    bound is the tighter one at every posture reachable here; both are computed and the
    smaller is returned.
    """
    d = D.derive(k)
    corner = d.r_nom_mm * math.tan(math.radians(HW_CORNER_DEG - 45.0))
    middle = d.r_nom_mm * math.tan(math.radians(HW_MIDDLE_DEG))
    return min(corner, middle)


def half_stride_middle_coxa_d248(k):
    """THE RECORD: D248's +/-13.5000 envelope, the bar FINDING_22 was computed on.
    D427 clause 2 withdrew it as a bar; it is kept so that table still reproduces."""
    d = D.derive(k)
    mins, maxs = project_joint_limits(k.value("joint_envelopes_deg"))
    limits = []
    for leg in MIDDLE_LEGS:
        i = LEGS.index(leg)
        win = (mins[hex_coxa(i)], maxs[hex_coxa(i)])

        def inside(s):
            lo, hi = coxa_range_deg(k, d, leg, s)
            return win[0] <= lo and hi <= win[1]

        if not inside(0.0):
            return 0.0
        lo_s, hi_s = 0.0, d.r_nom_mm * 10.0
        if inside(hi_s):
            limits.append(hi_s)
            continue
        while hi_s - lo_s > TOL_MM:
            mid = 0.5 * (lo_s + hi_s)
            lo_s, hi_s = (mid, hi_s) if inside(mid) else (lo_s, mid)
        limits.append(lo_s)
    return min(limits)


def femur_check(k, stride_mm):
    row, _, trace = D.sweep_point(k, stride_mm, k.value("duty_factor"))
    psi = trace["psi_deg"]
    angles = [row["theta_nom_deg"] - psi, row["theta_extreme_deg"] - psi,
              row["theta_midswing_deg"] - psi]
    mins, maxs = project_joint_limits(k.value("joint_envelopes_deg"))
    lo = max(mins[hex_femur(i)] for i in range(len(LEGS)))
    hi = min(maxs[hex_femur(i)] for i in range(len(LEGS)))
    return min(angles), max(angles), (lo, hi), lo <= min(angles) and max(angles) <= hi


def stride_limits(k, bars="hardware"):
    """Half-stride limits by bar, the longest stride, and the bar that binds.

    bars="hardware"  torque, vertical and Hardware's coxa proxy (D427, D428 clause 3)
    bars="d248"      torque, vertical and D248's withdrawn envelope - FINDING_22's table
    """
    coxa = half_stride_coxa_proxy(k) if bars == "hardware" else half_stride_middle_coxa_d248(k)
    lim = OrderedDict([("torque", half_stride_torque(k)),
                       ("vertical", half_stride_vertical(k)),
                       ("coxa proxy" if bars == "hardware" else "middle coxa", coxa)])
    live = {n: v for n, v in lim.items() if v is not None}
    if len(live) < len(lim):
        return lim, None, [n for n, v in lim.items() if v is None]
    s = min(live.values())
    binding = [n for n, v in live.items() if abs(v - s) <= 1e-9]
    return lim, 2.0 * s, binding


# --------------------------------------------------------------- posture search

def stride_zero_posture_deg(k):
    """The posture at which the torque bar admits exactly zero stride."""
    d = D.derive(k)
    a_max = D.a_eff_max_mm(k)
    return math.degrees(math.acos(a_max / d.rigid_len_mm)) - d.psi_deg


def best_posture_deg(k, lo_deg, hi_deg, bars="hardware"):
    """Bisection for the posture where min(torque, vertical) meets the coxa limit."""
    def gap(t):
        kk = at_posture(k, t)
        growing = [half_stride_torque(kk), half_stride_vertical(kk)]
        if any(v is None for v in growing):
            return -1.0
        coxa = (half_stride_coxa_proxy(kk) if bars == "hardware"
                else half_stride_middle_coxa_d248(kk))
        return min(growing) - coxa

    a, b = lo_deg + 1e-9, hi_deg
    if gap(a) > 0 or gap(b) < 0:
        raise ValueError("no crossing between %.4f and %.4f deg" % (lo_deg, hi_deg))
    for _ in range(200):
        m = 0.5 * (a + b)
        a, b = (m, b) if gap(m) < 0 else (a, m)
        if b - a < 1e-10:
            break
    return 0.5 * (a + b)


# --------------------------------------------------------------- one row

def coxa_angles_from_mount_deg(k, half_stride_mm):
    """(middle, corner) stance angle from the mount on D58's footholds, in degrees."""
    d = D.derive(k)
    a = math.degrees(math.atan(half_stride_mm / d.r_nom_mm))
    return a, 45.0 + a


def evaluate(k, theta2_nom_deg, stride_mm, label, bars="hardware"):
    """One row: the bars, D29's check, and the outputs by name."""
    kk = at_posture(k, theta2_nom_deg)
    d = D.derive(kk)
    limits, longest, binding = stride_limits(kk, bars)
    row, _, trace = D.sweep_point(kk, stride_mm, kk.value("duty_factor"))
    s = stride_mm / 2.0
    mins, maxs = project_joint_limits(kk.value("joint_envelopes_deg"))

    out = OrderedDict()
    out["case"] = label
    out["status"] = STATUS
    out["theta2_nom_deg"] = theta2_nom_deg
    out["stride_mm"] = stride_mm
    out["longest_stride_mm"] = longest
    out["binding_bar"] = " + ".join(binding)
    for name, v in limits.items():
        out["half_stride_limit_mm@" + name] = v

    mass = kk.value("mass_kg")
    margin, servo = kk.value("margin_factor"), kk.value("tau_servo_kgcm")
    out["tau_x_margin_kgcm (D209, D420 cl.1)"] = row["tau_femur_peak_kgcm"] * margin
    out["torque_bar_passes"] = row["tau_femur_peak_kgcm"] * margin <= servo + 1e-9
    out["tau_x_3.0000_kgcm (sensitivity: margin 2.5000 at %.4f kg)" % (mass * 3.0 / margin)] = \
        row["tau_femur_peak_kgcm"] * 3.0
    total = row["bob_mm"] + row["foot_dz_per_quantum_mm@joint_accuracy_deg"]
    out["vertical_total_mm (D425 cl.3)"] = total
    out["vertical_bar_passes"] = total <= kk.value("body_bob_budget_mm") + 1e-9

    f_lo, f_hi, _win, f_ok = femur_check(kk, stride_mm)
    out["femur_theta2_min_deg"], out["femur_theta2_max_deg"] = f_lo, f_hi
    out["femur_window_passes (D248, still in force)"] = f_ok

    middle, corner = coxa_angles_from_mount_deg(kk, s)
    out["coxa_bar_source"] = HW_BAR_LABEL
    out["coxa_bar_is_a_proxy"] = PROXY_LABEL
    out["middle_coxa_from_mount_deg"] = middle
    out["middle_coxa_bound_deg"] = HW_MIDDLE_DEG
    out["corner_coxa_from_mount_deg"] = corner
    out["corner_coxa_bound_deg"] = HW_CORNER_DEG
    out["corner_coxa_margin_deg (stance only)"] = HW_CORNER_DEG - corner
    out["coxa_proxy_bar_passes"] = corner <= HW_CORNER_DEG + 1e-9 and middle <= HW_MIDDLE_DEG + 1e-9

    # D248's envelope, withdrawn as a bar by D427 clause 2, kept as the record that
    # FINDING_22 was computed against.
    for column, clocked in (("record: corner excess vs D248 @ beta_neutral 0", False),
                            ("record: corner excess vs D248 @ clocked 43.2", True)):
        worst = 0.0
        for leg in CORNER_LEGS:
            i = LEGS.index(leg)
            bn = clocked_beta_neutral(kk, d, leg) if clocked else 0.0
            lo, hi = coxa_range_deg(kk, d, leg, s, beta_neutral_deg=bn)
            worst = max(worst, window_excess_deg(lo, hi, (mins[hex_coxa(i)], maxs[hex_coxa(i)])))
        out[column] = worst

    # D29's own check, over the WHOLE cycle including the hand-over (D430 clause 4).
    dist, where, phase = I.min_link_distance(kk, stride_mm, D29_FRAMES)
    out["d29_min_link_distance_mm"] = dist
    out["d29_closest_links"] = where
    out["d29_phase"] = phase
    out["d29_minimum_at_handover"] = min(abs(phase - p) for p in (0.0, 0.5, 1.0)) < 1e-3
    out["d29_swing_model"] = I.SWING_MODEL
    out["d29_cross_section_mm"] = "UNSUPPLIED (Spider Hardware, D29 second check)"
    for w in CROSS_SECTIONS_MM:
        out["d29_clearance_mm @ cross-section %.4f" % w] = dist - w
    for o in OVERSHOOT_DEG:
        out["d29_min_link_distance_mm @ swing overshoot %.4f deg (D430 cl.4 magnitude, "
            "shape parameterised)" % o] = I.min_link_distance(
                kk, stride_mm, D29_FRAMES, overshoot_deg=o)[0]

    for col in D.OUTPUT_COLUMNS:
        out[col] = row[col]
    out["swing_columns_model"] = D.SWING_MODEL_LABEL
    out["rows_12_13_profile"] = D145_LABEL
    out["swing_duration_understated (ratio < 1, D425 cl.6)"] = trace["swing_duration_understated"]
    out["theta_span_stance_deg (diagnostic, not an output)"] = trace["theta_span_stance_deg"]
    out["body_speed_mm_s (derived convenience, not an output)"] = trace["body_speed_mm_s"]
    return out


def stride_curve(k, lo_deg, hi_deg, step_deg=0.5, bars="hardware"):
    """The stride ceiling as a curve, not a cap (D428 clause 2, D64): posture, longest
    stride, the bar that binds, and D29's distance there. Uncapped by 60.0000 mm."""
    rows = []
    p = lo_deg
    while p <= hi_deg + 1e-9:
        kk = at_posture(k, p)
        _lim, longest, binding = stride_limits(kk, bars)
        if longest is not None:
            dist, where, phase = I.min_link_distance(kk, longest, D29_FRAMES)
            rows.append(OrderedDict([
                ("theta2_nom_deg", p), ("longest_stride_mm", longest),
                ("binding_bar", " + ".join(binding)),
                ("d29_min_link_distance_mm", dist), ("d29_closest_links", where),
                ("d29_phase", phase), ("status", STATUS)]))
        p += step_deg
    return rows


def run(k, bars="hardware"):
    floor = stride_zero_posture_deg(k)
    best = best_posture_deg(k, floor, 89.9999, bars)
    rows = []
    postures = [("sample", p) for p in SAMPLE_POSTURES_DEG] + [("solved best posture", best)]
    for label, p in sorted(postures, key=lambda lp: lp[1]):
        longest = stride_limits(at_posture(k, p), bars)[1]
        rows.append(evaluate(k, p, longest, "%s, longest stride" % label, bars))
    posture, stride = REPORTED_CASE
    rows.append(evaluate(k, posture, stride if stride is not None else k.value("stride_mm"),
                         "reported case, shipped stride", bars))
    return floor, best, rows


def fmt(v):
    if v is None:
        return "no root"
    if isinstance(v, bool):
        return "yes" if v else "NO"
    if isinstance(v, float):
        return "{:.4f}".format(v)
    return str(v)


def write_csv(rows, path):
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh, lineterminator="\n")
        w.writerow(list(rows[0]))
        for r in rows:
            w.writerow([fmt(v) for v in r.values()])


def main(argv=None):
    ap = argparse.ArgumentParser(description="Search on D58's lateral footholds.")
    ap.add_argument("--csv")
    ap.add_argument("--curve-csv", help="the stride ceiling as a curve over posture")
    args = ap.parse_args(argv)
    k = C.load()
    floor, best, rows = run(k)
    print(k.stamp())
    print("stride-zero torque limit: theta2_nom_deg %.4f" % floor)
    print("solved best posture:      theta2_nom_deg %.4f" % best)
    print()
    print("| quantity | " + " | ".join("%s / %s" % (fmt(r["theta2_nom_deg"]), fmt(r["stride_mm"])) for r in rows) + " |")
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
