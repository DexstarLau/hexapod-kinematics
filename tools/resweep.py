"""The D197 re-sweep (D411 clause 3, D414, D415): run it, and write what it found.

    python -m tools.resweep                  print the tables
    python -m tools.resweep --csv PATH       also write the CSV the tables come from

WHAT IS SEARCHED
----------------
D414 moves the search from slip to swing clearance. For each posture the sweep
finds the longest stride at which sim/stance_pose.py's D414 swing measure stays
at or above the floor, caps it at the shipped stride_mm, and evaluates the
fifteen outputs (D415) at that (posture, stride) pair. Nothing is chosen here:
coordination rules the pair (D411 clause 3).

WHICH POSTURES, AND WHY THOSE
-----------------------------
    the D209 floor at margin_factor 2.5000     derived here, never typed in
    the D209 floor at margin_factor 3.0000     D415 clause 7's second column
    75.0000, 80.0000, 85.0000                  the points D414 already reports,
                                               so the two derivations can be
                                               compared cell by cell

The D209 comparand is a_eff_nom_mm, the quantity the register's 70.0098 floor is
derived from. a_eff_extreme_mm (output 10) is printed beside it and is not used
as the comparand.

No row at theta2_nom_deg = 40.0000 is produced (D260).

EVERY FIGURE CARRIES ITS INPUTS' STATUS
---------------------------------------
The stamp line from sim/constants.py is printed with the tables: theta3_deg is a
surrogate, mass_kg is provisional, dtheta_peak_deg_s is disputed. A figure from
this file is a figure about those inputs.
"""

import argparse
import csv
import math
import sys
from collections import OrderedDict

from bindings.hexconfig import LEGS, hex_coxa, hex_femur, project_joint_limits
from sim import constants as C
from sim import derive as D
from sim import stance_pose as S

EXTRA_POSTURES_DEG = (75.0, 80.0, 85.0)
D11_BODY_BUDGET_MM = 5.0000     # D11, compared against, never used in a computation
BISECTION_ITERATIONS = 40


class _Override(object):
    """A constant table with named values replaced. Surrogate stamping passes through."""

    def __init__(self, base, **kw):
        self._base, self._kw = base, kw

    def value(self, name):
        return self._kw[name] if name in self._kw else self._base.value(name)

    def entry(self, name):
        return self._base.entry(name)


def posture_floor_deg(k, margin_factor):
    """theta2_nom_deg at which a_eff_nom_mm meets D209's ceiling. Derived, not stored."""
    t = _Override(k, margin_factor=margin_factor)
    ceiling = D.a_eff_max_mm(t)
    d = D.derive(k)
    return math.degrees(math.acos(ceiling / d.rigid_len_mm)) - d.psi_deg


def evaluate(k, theta2_nom_deg, stride_mm):
    """One sweep row: fifteen outputs by name, then the constraint columns."""
    t = _Override(k, theta2_nom_deg=theta2_nom_deg, stride_mm=stride_mm)
    row, d, trace = D.sweep_point(t, stride_mm, k.value("duty_factor"))
    summary = S.summarise(S.stance_cycle(t, stride_mm))
    mins, maxs = project_joint_limits(k.value("joint_envelopes_deg"))

    out = OrderedDict()
    out["theta2_nom_deg"] = theta2_nom_deg
    out["stride_mm"] = stride_mm
    out.update(row)
    out["theta_span_stance_deg (diagnostic, not an output)"] = trace["theta_span_stance_deg"]
    out["body_speed_mm_s (derived convenience, not an output)"] = trace["body_speed_mm_s"]
    out["span_predicate_clearance_ge_bob"] = trace["span_predicate_holds"]
    out["a_eff_nom_mm"] = d.a_eff_nom_mm
    out["a_eff_ceiling_mm @ margin 2.5000"] = D.a_eff_max_mm(_Override(t, margin_factor=2.5))
    out["a_eff_ceiling_mm @ margin 3.0000"] = D.a_eff_max_mm(_Override(t, margin_factor=3.0))
    out["pitch_pp_deg"] = summary["pitch_pp_deg"]
    out["roll_pp_deg"] = summary["roll_pp_deg"]
    out["body_height_pp_mm"] = summary["height_pp_mm"]
    out["worst_surface_mm"] = summary["worst_surface_mm"]
    out["min_swing_z_d414_mm"] = summary["min_swing_z_d414_mm"]
    out["min_swing_z_profile_mm"] = summary["min_swing_z_profile_mm"]
    out["handover_pitch_jump_deg"] = summary["handover_pitch_jump_deg"]
    out["handover_roll_jump_deg"] = summary["handover_roll_jump_deg"]
    out["theta2_max_deg"] = summary["theta2_max_deg"]
    out["femur_limit_max_deg"] = min(maxs[hex_femur(i)] for i in range(len(LEGS)))
    outside = joints_outside_windows(summary, mins, maxs)
    out["joints_inside_windows"] = not outside
    out["joints_outside_windows"] = ", ".join(outside) if outside else "none"
    return out


def joints_outside_windows(summary, mins, maxs):
    """Every joint, on every leg, whose range over the cycle leaves its projected window.

    The windows are MP1's own projection of joint_envelopes_deg (D247-D249, D258,
    D365), which are `provisional`, with no D262 command offset applied. A joint
    outside is reported against that projection and nothing firmer.
    """
    bad = []
    for i, leg in enumerate(LEGS):
        for joint, idx, lo_key, hi_key in (("coxa", hex_coxa(i), "theta1_min_deg_", "theta1_max_deg_"),
                                           ("femur", hex_femur(i), "theta2_min_deg_", "theta2_max_deg_")):
            if summary[lo_key + leg] < mins[idx] or summary[hi_key + leg] > maxs[idx]:
                bad.append("{} {}".format(leg, joint))
    return bad


def longest_stride_within_windows(k, theta2_nom_deg, hi_mm):
    """Longest stride keeping BOTH the D414 swing measure >= 0 and every joint inside
    its projected window. Bisection assumes both fail monotonically as stride grows;
    the endpoints are checked, and a lower bound that already fails raises."""
    mins, maxs = project_joint_limits(k.value("joint_envelopes_deg"))

    def ok(stride):
        t = _Override(k, theta2_nom_deg=theta2_nom_deg, stride_mm=stride)
        try:
            s = S.summarise(S.stance_cycle(t, stride))
        except S.NoPose:
            return False
        return s["min_swing_z_d414_mm"] >= 0.0 and not joints_outside_windows(s, mins, maxs)

    if ok(hi_mm):
        return hi_mm
    lo, hi = 1.0, hi_mm
    if not ok(lo):
        raise S.NoPose("no stride >= 1 mm meets both bars at {:.4f} deg".format(theta2_nom_deg))
    for _ in range(BISECTION_ITERATIONS):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
    return lo


def run(k):
    shipped = k.value("stride_mm")
    postures = [("D209 floor @ margin 2.5000", posture_floor_deg(k, 2.5)),
                ("D209 floor @ margin 3.0000", posture_floor_deg(k, 3.0))]
    postures += [("{:.4f}".format(p), p) for p in EXTRA_POSTURES_DEG]
    postures.sort(key=lambda lp: lp[1])

    rows = []
    for label, posture in postures:
        t = _Override(k, theta2_nom_deg=posture)
        stride = S.longest_stride(t, "min_swing_z_d414_mm", hi_mm=shipped,
                                  iterations=BISECTION_ITERATIONS)
        windowed = longest_stride_within_windows(k, posture, shipped)
        for kind, st in (("longest stride, D414 measure", stride),
                         ("longest stride, D414 measure and joint windows", windowed),
                         ("shipped stride", shipped)):
            r = OrderedDict([("posture", label), ("stride_kind", kind)])
            r.update(evaluate(k, posture, st))
            rows.append(r)
    return rows


def fmt(v):
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


def print_tables(rows, k):
    print(k.stamp())
    print()
    for kind in ("longest stride, D414 measure",
                 "longest stride, D414 measure and joint windows", "shipped stride"):
        cols = [r for r in rows if r["stride_kind"] == kind]
        print("### " + kind)
        print()
        print("| quantity | " + " | ".join(
            "{} / {} mm".format(fmt(r["theta2_nom_deg"]), fmt(r["stride_mm"])) for r in cols) + " |")
        print("|---|" + "---|" * len(cols))
        for key in list(cols[0])[4:]:
            print("| `{}` | ".format(key) + " | ".join(fmt(r[key]) for r in cols) + " |")
        print()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--csv", help="write the full table to this path")
    args = ap.parse_args(argv)
    k = C.load()
    rows = run(k)
    print_tables(rows, k)
    if args.csv:
        write_csv(rows, args.csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
