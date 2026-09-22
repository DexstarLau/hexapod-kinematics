"""P8's swing path in closed form (HANDOFF_C16_AI §2, accepted as measurement by D433), and
D29's link-distance check swept along it (D431 clauses 3 and 7, PROJECT_28 §1).

WHAT THIS IS
------------
Spider AI's path at 80.0000 / 60.0000 mm on D58's lateral footholds, for swing_ramp_eps
0.015, 0.120 and 0.246. It is re-derived here from HANDOFF_C16_AI §2's formulas and from
config/hexapod.json - NOT copied from its tables, which are the check values the tests
hold it to (T1, T2, T3). Every constant is computed, never typed.

THE PATH (reference leg R2, tripod B; tau = 2*phase over its swing, [0, 0.5))
    swing   theta1(tau) = theta1_lo + w0*tau + (W - w0)*IB(tau; eps)
            rho(tau)    = rho_c + c*sin(pi*tau) + a*(1 - cos(2*pi*tau))/2
            Theta       = acos((rho - L1) / R)          the rigid member's angle, as interleg
    stance  x_f = s/2 - s*u, u = 2*phase - 1
            theta1 = atan2(x_f, r_nom),  rho = hypot(r_nom, x_f)
    IB, W, w0, c, a exactly as HANDOFF_C16_AI §2 defines them.

THE PER-LEG MAP, IN interleg's TERMS
------------------------------------
HANDOFF_C16_AI §2: theta1_k = theta1_mid(k) + side(k)*theta1_R2(phase + delta), delta 0.5 for
tripod A, and body yaw = beta_mount + theta1. Substituting theta1_mid = (-90 right, +90 left)
- beta_mount, the mount cancels:

    body yaw = -90 + theta1_R2(phase + delta)     right legs
    body yaw = +90 - theta1_R2(phase + delta)     left legs

which is the 'yaw' sim/interleg.py's leg_links takes. Theta is the same on both sides. The
test checks this against MP1's own stance geometry (interleg.stance_pose_of) on all six
legs, and against HANDOFF_C16_AI's T3 body-yaw column - not against the map itself.

THE PHASE CONVENTION is sim/interleg.py's pose_at(): tripod A = R1, R3, L2 stands for
phase < 0.5 (HANDOFF_C16_AI §1 states it is identical; the tests check it at the extremes).

WHAT IT HAS NO POWER OVER
-------------------------
Any posture or stride but 80.0000 / 60.0000, any eps but the three, the machine, wiring,
the -0.5000 mm coxa-to-femur drop, and theta3 (a surrogate, -30.0000 throughout, which the
rigid member R already carries).
"""

import math
from collections import OrderedDict

from sim import derive as D
from sim import interleg as I

EPS_SET = (0.015, 0.120, 0.246)
TRIPOD_A = I.TRIPODS[0]
POSTURE_DEG, STRIDE_MM = 80.0, 60.0
SOURCE = ("P8's path, HANDOFF_C16_AI sec.2 (Spider AI, bc063acde6953a52..., D433), re-derived in "
          "closed form from config/hexapod.json at 80.0000 / 60.0000 mm")


def _smooth_integral(x):
    """I(x) = 2.5x^4 - 3x^5 + x^6, the integral of the quintic smoothstep."""
    return 2.5 * x ** 4 - 3.0 * x ** 5 + x ** 6


def _ib(tau, eps):
    if tau <= eps:
        return eps * _smooth_integral(tau / eps)
    if tau < 1.0 - eps:
        return eps / 2.0 + (tau - eps)
    return 1.0 - eps - eps * _smooth_integral((1.0 - tau) / eps)


class P8Path(object):
    """The path for one table k (posture already set), one stride and one eps."""

    def __init__(self, k, stride_mm, eps):
        if not 0.0 < eps <= 0.5:
            raise ValueError("swing_ramp_eps %r is outside (0, 0.5]" % eps)
        d = D.derive(k)
        self.k, self.d, self.stride, self.eps = k, d, stride_mm, eps
        self.L1, self.R, self.r_nom = k.value("coxa_length_mm"), d.rigid_len_mm, d.r_nom_mm
        half = stride_mm / 2.0
        th_lo = -math.atan(half / self.r_nom)
        self.theta1_lo = math.degrees(th_lo)
        self.theta1_td = -self.theta1_lo
        self.rho_c = math.hypot(self.r_nom, half)
        self.w0 = math.degrees(-stride_mm * math.cos(th_lo) / self.rho_c)       # deg per tau
        self.c = stride_mm * abs(math.sin(th_lo)) / math.pi
        sin_mid = (d.body_height_mm - k.value("swing_clearance_mm")) / self.R
        self.rho_mid = self.L1 + self.R * math.sqrt(1.0 - sin_mid ** 2)
        self.a = self.rho_mid - self.rho_c - self.c
        self.W = self.w0 + (self.theta1_td - self.theta1_lo - self.w0) / (1.0 - eps)

    # ---------------------------------------------------------------- R2, the reference leg
    def r2(self, phase):
        """(theta1 from the mount, Theta, swinging) for R2 at this phase, degrees."""
        phase = phase % 1.0
        if phase < 0.5:
            tau = 2.0 * phase
            theta1 = self.theta1_lo + self.w0 * tau + (self.W - self.w0) * _ib(tau, self.eps)
            rho = (self.rho_c + self.c * math.sin(math.pi * tau)
                   + self.a * (1.0 - math.cos(2.0 * math.pi * tau)) / 2.0)
            swinging = True
        else:
            u = 2.0 * phase - 1.0
            x_f = self.stride / 2.0 - self.stride * u
            theta1 = math.degrees(math.atan2(x_f, self.r_nom))
            rho = math.hypot(self.r_nom, x_f)
            swinging = False
        cos_t = (rho - self.L1) / self.R
        if not -1.0 <= cos_t <= 1.0:
            raise I.NoPose("reach {:.4f} mm is outside the leg".format(rho))
        return theta1, math.degrees(math.acos(cos_t)), swinging

    def theta2_deg(self, phase):
        """The femur command, kinematic: Theta - psi (HANDOFF_C16_AI §1)."""
        return self.r2(phase)[1] - self.d.psi_deg

    # ---------------------------------------------------------------- every leg
    def leg(self, leg, phase):
        """(body yaw, Theta, swinging) - the pair sim/interleg.py's leg_links takes."""
        delta = 0.5 if leg in TRIPOD_A else 0.0
        theta1, theta, swinging = self.r2(phase + delta)
        right = self.k.value("coxa_positions_mm")[leg][1] < 0.0
        return (-90.0 + theta1 if right else 90.0 - theta1), theta, swinging

    def poses(self, phase):
        return OrderedDict((leg, self.leg(leg, phase)) for pair in I.TRIPODS for leg in pair)

    def overshoot_deg(self, tol=1e-12):
        """The coxa's excursion beyond its stance range: theta1_lo minus the minimum of the
        first ramp, found where theta1' = 0 by bisection on the ramp's derivative."""
        def rate(tau):
            h = 1e-7
            return (self.r2((tau + h) / 2.0)[0] - self.r2((tau - h) / 2.0)[0]) / (2 * h)
        lo, hi = 1e-9, self.eps
        if rate(hi) <= 0.0:
            raise ValueError("no extreme inside the first ramp")
        while hi - lo > tol:
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if rate(mid) < 0.0 else (lo, mid)
        tau_star = 0.5 * (lo + hi)
        return self.theta1_lo - self.r2(tau_star / 2.0)[0], tau_star


# -------------------------------------------------------------------- D29 along the path

def _closest(k, d, path, phase, pairs=I.ADJACENT_PAIRS):
    """(distance, description, phase, pair) at one phase; the description says which link
    of which leg, and whether that leg is swinging or standing."""
    poses = path.poses(phase)
    best = (float("inf"), None, phase, None)
    names = ("coxa", "rigid")
    for left, right in pairs:
        a = I.leg_links(k, d, left, poses[left][0], poses[left][1])
        b = I.leg_links(k, d, right, poses[right][0], poses[right][1])
        for ia, sa in enumerate(((a[0], a[1]), (a[1], a[2]))):
            for ib, sb in enumerate(((b[0], b[1]), (b[1], b[2]))):
                dist = I.segment_distance(sa[0], sa[1], sb[0], sb[1])
                if dist < best[0]:
                    tag = lambda leg: "swing" if poses[leg][2] else "stance"
                    best = (dist, "%s %s (%s) / %s %s (%s)" % (
                        left, names[ia], tag(left), right, names[ib], tag(right)),
                        phase, (left, right))
    return best


def min_link_distance(k, path, frames=4001, refine=60, pairs=I.ADJACENT_PAIRS):
    """Minimum over the whole cycle: a 4,001-frame grid (phase step 0.00025, finer than
    tau* = 0.0074 of the swing, i.e. phase 0.0037), then golden-section around the best
    frame. Checked by test against a denser brute-force grid."""
    d = D.derive(k)
    grid = [i / float(frames - 1) for i in range(frames)]
    best = min((_closest(k, d, path, p, pairs) for p in grid), key=lambda r: r[0])
    step = 1.0 / (frames - 1)
    lo, hi = best[2] - step, best[2] + step
    for _ in range(refine):
        m1, m2 = lo + 0.382 * (hi - lo), lo + 0.618 * (hi - lo)
        r1 = _closest(k, d, path, m1, pairs)
        r2 = _closest(k, d, path, m2, pairs)
        if r1[0] < r2[0]:
            hi, best = m2, min(best, r1, key=lambda r: r[0])
        else:
            lo, best = m1, min(best, r2, key=lambda r: r[0])
    phase = best[2] % 1.0
    return best[0], best[1], (0.0 if phase > 1.0 - 5e-9 else phase), best[3]


# -------------------------------------------------------------------- the calibration poses

TIE_MM = 1e-9                   # pairs within this of the minimum TIE: none is 'the' closest
HARDWARE_GAP_MM = 3.96          # HANDOFF_C09_HW §3 as D431 quotes it; that file is NOT HELD here
RULED_WIDTH_MM = 40.8100        # D431 clause 3
ENVELOPE_WIDTHS_MM = (52.0000, 56.0000)
STATUS = ("interim - P8 at 80.0000 / 60.0000 only; centre-line less a width; which width is "
          "open (see the three calibration poses); no pair (D431 cl.9)")


class _At(object):
    def __init__(self, base, **kw):
        self._base, self._kw = base, kw

    def value(self, name):
        return self._kw[name] if name in self._kw else self._base.value(name)


def at_posture(k, theta2_nom_deg=POSTURE_DEG):
    return _At(k, theta2_nom_deg=theta2_nom_deg)


def _pair_distance(k, d, a_pose, b_pose, a="R2", b="R3"):
    la, lb = I.leg_links(k, d, a, *a_pose), I.leg_links(k, d, b, *b_pose)
    return min(I.segment_distance(sa[0], sa[1], sb[0], sb[1])
               for sa in ((la[0], la[1]), (la[1], la[2])) for sb in ((lb[0], lb[1]), (lb[1], lb[2])))


def calibration_poses(k):
    """R2 against R3 at 80.0000 / 60.0000, in the three poses the 3.96 mm could belong to.

    (a) FINDING_23's 44.7711: interleg.pose_at at phase 0 - R2 in the swing model at lift-off,
        which holds the MID-STANCE body height, so its femur reads 80.0000; R3 at its true
        stance extreme, femur 78.7384. Not 'stance extremes', as FINDING_23 labelled it.
    (b) both legs at their true stance extremes - P8 at phase 0, and interleg.stance_pose_of.
    (c) Hardware's pose as D431 quotes it: coxae at +/-13.2629, femur 80.0000 and tibia
        -30.0000 on both. HANDOFF_C09_HW is not held, so which femur angle each leg was
        posed at is NOT verified here.
    Returns OrderedDict label -> (centre-line mm, width = centre-line - 3.96)."""
    d = D.derive(k)
    out = OrderedDict()
    pa = I.pose_at(k, d, STRIDE_MM, 0.0)
    ext2 = I.stance_pose_of(k, d, "R2", -STRIDE_MM / 2.0)
    ext3 = I.stance_pose_of(k, d, "R3", +STRIDE_MM / 2.0)
    nominal = k.value("theta2_nom_deg") + d.psi_deg
    for label, a_pose, b_pose in (
            ("(a) FINDING_23: R2 swing model at lift-off / R3 stance extreme", pa["R2"], pa["R3"]),
            ("(b) both at their true stance extremes", ext2, ext3),
            ("(c) Hardware's pose as quoted: coxae at the extremes, both femurs nominal",
             (ext2[0], nominal), (ext3[0], nominal))):
        c = _pair_distance(k, d, a_pose, b_pose)
        out[label] = (c, c - HARDWARE_GAP_MM)
    return out


def sweep_rows(k0):
    """D29 along P8, per eps: overall and per pair, at the ruled width, at the widths poses
    (b) and (c) imply, and at the two envelopes. Beside it, the parameterised overshoot of
    sim/interleg.py at the SAME scalar - the comparison PROJECT_28 §1.2 asks about."""
    k = at_posture(k0)
    d = D.derive(k)
    cal = calibration_poses(k)
    widths = OrderedDict([("ruled 40.8100 (D431 cl.3, on pose a)", RULED_WIDTH_MM)])
    for label, (_c, w) in list(cal.items())[1:]:
        widths["%.4f from pose %s" % (w, label[1])] = w
    for w in ENVELOPE_WIDTHS_MM:
        widths["envelope %.4f" % w] = w
    rows = []
    for eps in EPS_SET:
        path = P8Path(k, STRIDE_MM, eps)
        over, tau_star = path.overshoot_deg()
        param = I.min_link_distance(k, STRIDE_MM, overshoot_deg=over)[0]
        at_contact = _closest(k, d, path, 0.0)[0]
        # Per pair first. The four adjacent pairs are mirror images of each other and tie
        # to about 1e-12 mm, so WHICH pair holds the overall minimum is decided by the last
        # bits - a first version printed the winner, and Windows picked a different one.
        # The overall row now names every pair within TIE_MM, and the first of them in
        # ADJACENT_PAIRS order: a fixed choice, not a floating-point one.
        per_pair = [(("%s-%s" % p), min_link_distance(k, path, pairs=(p,))) for p in I.ADJACENT_PAIRS]
        low = min(res[0] for _, res in per_pair)
        tied = [(name, res) for name, res in per_pair if res[0] <= low + TIE_MM]
        first = tied[0][1]
        overall = (low, first[1] + ("; tie within 1e-9 mm: " + ", ".join(n for n, _ in tied)
                                    if len(tied) > 1 else ""), first[2], first[3])
        for scope, result in [("all four pairs", overall)] + per_pair:
            dist, desc, phase, pair = result
            r = OrderedDict()
            r["swing_ramp_eps"] = eps
            r["p8_overshoot_deg (beyond stance, at tau*)"] = over
            r["tau_star (of the swing)"] = tau_star
            r["scope"] = scope
            r["d29_min_centreline_mm"] = dist
            r["phase"] = phase
            r["closest_links"] = desc
            r["centreline_at_the_hand_over_mm (phase 0)"] = at_contact
            r["cost_of_the_swing_mm (hand-over less minimum)"] = at_contact - dist
            for name, w in widths.items():
                r["clearance_mm @ %s" % name] = dist - w
            for name, w in widths.items():
                r["passes @ %s" % name] = dist - w > 0.0
            r["interleg_parameterised_min_mm @ the same scalar overshoot"] = param
            r["source"] = SOURCE
            r["status"] = STATUS
            rows.append(r)
    return cal, rows


def main(argv=None):
    import argparse
    import sys
    from sim import constants as C
    from tools.calibrated_sweep import fmt, write_csv
    ap = argparse.ArgumentParser(description="D29 along P8's path (D431 cl.7, D433).")
    ap.add_argument("--csv")
    args = ap.parse_args(argv)
    cal, rows = sweep_rows(C.load())
    for label, (c, w) in cal.items():
        print("%-78s centre-line %.4f mm, width %.4f mm" % (label, c, w))
    for r in rows:
        print(" | ".join(fmt(v) for k_, v in r.items() if k_ not in ("source", "status")))
    if args.csv:
        write_csv(rows, args.csv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
