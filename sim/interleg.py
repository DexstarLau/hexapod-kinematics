"""D29's inter-leg clearance check: the distance between adjacent leg LINKS, across the
whole gait cycle, at a configurable cross-section width.

WHAT D29 ASKS FOR, AND WHY A COXA ANGLE IS NOT IT
-------------------------------------------------
D29: under D15's lateral configuration, adjacent legs on the same side are parallel and
stroke in antiphase, so they close on each other by the full stride. It adopts two
checks: minimum distance between adjacent leg LINKS across the gait cycle at a
configurable cross-section width, and a hardware report of the physical cross-section.

Hardware's coxa figures (D427 clause 3) are a PROXY for the first. They are measured in
one pose, they are lower bounds, and they say nothing about where the links actually are
at any other moment of the cycle. This module is the check itself.

THE MODEL
---------
Each leg is two straight links in the BODY frame:

    P0  the coxa axis, at coxa_positions_mm[leg]
    P1  the femur axis: P0 + L1 along the leg's own yaw, horizontal
    P2  the foot:       P1 + R*cos(Theta) along that yaw, R*sin(Theta) down

Theta is the rigid member's angle (theta2 + psi), exactly as sim/derive.py uses it, so
the two members collapse onto one segment by the same identity. Body attitude does not
enter: the body is rigid, and rotating it rotates every link together.

    stance   the foot is D58's lateral foothold, beside its coxa at y0 = r_nom_mm,
             moving along x from +stride/2 to -stride/2. Theta follows the reach.
    swing    the foot follows the level-body, body-frame progress model that MP1's sweep
             columns are labelled with (D426 clauses 1-2): half-sine stride fraction,
             lift = swing_clearance_mm * sin(pi*u), and the 2-DOF leg solved the way
             IK_PROJ_RADIAL does - hold the height, clamp the reach.

    THE SWING SIDE IS THAT MODEL, NOT D430's P8. P8 is adopted as the engine's swing
    direction but is not held here as a trajectory. Every swing-phase figure this module
    prints carries that label, and the check is re-run when the profile is delivered.

THE HAND-OVER IS IN THE SAMPLE
------------------------------
At the phase boundary one tripod plants at its front extreme while the other lifts from
its rear extreme, so the closing pair is one SWINGING link against one STANDING link -
the case D430 clause 4 names, and the case a stance-only proxy cannot see. The frame
grid includes both ends of both half-cycles, and the minimum is refined around the
sampled best frame rather than read off the grid.

WHICH PAIRS
-----------
The four adjacent same-side pairs, which is what D29's sentence is about: R1-R2, R2-R3,
L1-L2, L2-L3. Legs across the body are not adjacent and are not checked here; if that
ever matters it is a separate finding, not a silent extra pair.

WIDTH
-----
clearance = centre-line distance - cross_section_mm. The cross-section is Hardware's to
supply (D427 clause 4, D29's second check). Until it arrives nothing is assumed: the
callers pass a width, and a width of 0.0 means the raw centre-line distance.
"""

import math
from collections import OrderedDict

from sim import derive as D

ADJACENT_PAIRS = (("R1", "R2"), ("R2", "R3"), ("L1", "L2"), ("L2", "L3"))
TRIPODS = (("R1", "R3", "L2"), ("R2", "L1", "L3"))
SWING_MODEL = D.SWING_MODEL_LABEL


class NoPose(Exception):
    """A phase the leg cannot stand at. Never a clamped angle."""


def _half_sine(u):
    return 0.5 * (1.0 - math.cos(math.pi * u))


def leg_links(k, d, leg, yaw_deg, theta_deg):
    """(P0, P1, P2): the coxa link and the rigid member, in the body frame."""
    px, py = k.value("coxa_positions_mm")[leg]
    yaw = math.radians(yaw_deg)
    cx, cy = math.cos(yaw), math.sin(yaw)
    L1 = k.value("coxa_length_mm")
    R = d.rigid_len_mm
    p0 = (px, py, 0.0)
    p1 = (px + L1 * cx, py + L1 * cy, 0.0)
    horizontal = R * math.cos(math.radians(theta_deg))
    p2 = (p1[0] + horizontal * cx, p1[1] + horizontal * cy,
          -R * math.sin(math.radians(theta_deg)))
    return p0, p1, p2


def stance_pose_of(k, d, leg, x_mm):
    """(yaw, Theta) for a foot on D58's lateral foothold, offset x along the body."""
    px, py = k.value("coxa_positions_mm")[leg]
    side = 1.0 if py > 0 else -1.0
    fx, fy = px + x_mm, py + side * d.r_nom_mm
    yaw = math.degrees(math.atan2(fy - py, fx - px))
    reach = math.hypot(fx - px, fy - py)
    cos_theta = (reach - k.value("coxa_length_mm")) / d.rigid_len_mm
    if not -1.0 <= cos_theta <= 1.0:
        raise NoPose("stance reach {:.4f} mm is outside the leg".format(reach))
    return yaw, math.degrees(math.acos(cos_theta))


def swing_pose_of(k, d, leg, u, stride_mm, overshoot_deg=0.0):
    """(yaw, Theta) on the labelled swing model, solved as IK_PROJ_RADIAL does.

    overshoot_deg prices D430 clause 4's overshoot WITHOUT claiming P8's shape: the coxa
    passes beyond its stance range by this much at BOTH ends, which is the structural
    consequence D430 derives, and the offset used here is -overshoot*cos(pi*u) - beyond
    the rear extreme at lift-off, beyond the front extreme at touchdown, zero at
    mid-swing. The MAGNITUDE is D430's (0.09-2.20 deg at the pairs Spider AI measured);
    the shape is this module's parameterisation and is labelled so wherever it is used.
    """
    px, py = k.value("coxa_positions_mm")[leg]
    side = 1.0 if py > 0 else -1.0
    x = (-0.5 + _half_sine(u)) * stride_mm
    lift = k.value("swing_clearance_mm") * math.sin(math.pi * u)
    fx, fy = px + x, py + side * d.r_nom_mm
    yaw = math.degrees(math.atan2(fy - py, fx - px))
    # yaw grows with x on the right side and falls on the left, so this sign is the one
    # that carries the foot further BEYOND the rear extreme at lift-off and beyond the
    # front extreme at touchdown. A first version had it the other way and made the legs
    # look further apart; caught by the control that says an overshoot must close the gap.
    yaw += side * overshoot_deg * math.cos(math.pi * u)
    sin_theta = (d.body_height_mm - lift) / d.rigid_len_mm
    if not -1.0 <= sin_theta <= 1.0:
        raise NoPose("swing lift {:.4f} mm is outside the leg".format(lift))
    return yaw, math.degrees(math.asin(sin_theta))


def pose_at(k, d, stride_mm, phase, overshoot_deg=0.0):
    """Every leg's (yaw, Theta) at one phase in [0, 1). Group A stances first."""
    s = stride_mm / 2.0
    out = OrderedDict()
    for g, stance in enumerate(TRIPODS):
        u = (phase * 2.0) % 1.0
        standing = (phase < 0.5) == (g == 0)
        for leg in stance:
            if standing:
                out[leg] = stance_pose_of(k, d, leg, s - u * stride_mm)
            else:
                out[leg] = swing_pose_of(k, d, leg, u, stride_mm, overshoot_deg)
    return out


def segment_distance(a0, a1, b0, b1):
    """Shortest distance between two line SEGMENTS.

    The standard clamped solve: minimise |(a0 + sc*u) - (b0 + tc*v)| over sc, tc in
    [0, 1]. The unconstrained solution is clamped to the square and, when a clamp binds,
    the other parameter is re-solved on that edge - clamping both at once is the error
    that makes two crossing segments look far apart. Checked against a brute-force grid
    by test.
    """
    def sub(p, q):
        return (p[0] - q[0], p[1] - q[1], p[2] - q[2])

    def dot(p, q):
        return p[0] * q[0] + p[1] * q[1] + p[2] * q[2]

    u, v, w = sub(a1, a0), sub(b1, b0), sub(a0, b0)
    a, b, c, dd, e = dot(u, u), dot(u, v), dot(v, v), dot(u, w), dot(v, w)
    den = a * c - b * b
    if den > 1e-12:
        sc = (b * e - c * dd) / den
    else:
        sc = 0.0                                   # parallel: start at one end
    sc = min(1.0, max(0.0, sc))
    tc = (b * sc + e) / c if c > 1e-12 else 0.0
    if tc < 0.0 or tc > 1.0:                       # the t clamp binds: re-solve s on it
        tc = min(1.0, max(0.0, tc))
        sc = (b * tc - dd) / a if a > 1e-12 else 0.0
        sc = min(1.0, max(0.0, sc))
    dx = (w[0] + sc * u[0] - tc * v[0], w[1] + sc * u[1] - tc * v[1],
          w[2] + sc * u[2] - tc * v[2])
    return math.sqrt(dot(dx, dx))


def link_distance_at(k, d, stride_mm, phase, overshoot_deg=0.0):
    """(distance, pair, which links, phase) - the closest adjacent links at one phase."""
    poses = pose_at(k, d, stride_mm, phase, overshoot_deg)
    best = (float("inf"), None, None)
    for left, right in ADJACENT_PAIRS:
        a = leg_links(k, d, left, *poses[left])
        b = leg_links(k, d, right, *poses[right])
        for ia, (a0, a1) in enumerate(((a[0], a[1]), (a[1], a[2]))):
            for ib, (b0, b1) in enumerate(((b[0], b[1]), (b[1], b[2]))):
                dist = segment_distance(a0, a1, b0, b1)
                if dist < best[0]:
                    names = ("coxa", "rigid")
                    best = (dist, "%s %s / %s %s" % (left, names[ia], right, names[ib]),
                            phase)
    return best


def min_link_distance(k, stride_mm, frames=121, refine=40, overshoot_deg=0.0):
    """Minimum over the whole cycle, refined around the best frame rather than read off
    the grid. Returns (distance_mm, which pair and links, phase)."""
    d = D.derive(k)
    grid = [i / float(frames - 1) for i in range(frames)]
    best = min((link_distance_at(k, d, stride_mm, p, overshoot_deg) for p in grid),
               key=lambda r: r[0])
    step = 1.0 / (frames - 1)
    lo, hi = max(0.0, best[2] - step), min(1.0, best[2] + step)
    for _ in range(refine):                       # golden-section on the local minimum
        m1, m2 = lo + 0.382 * (hi - lo), lo + 0.618 * (hi - lo)
        r1 = link_distance_at(k, d, stride_mm, m1, overshoot_deg)
        r2 = link_distance_at(k, d, stride_mm, m2, overshoot_deg)
        if r1[0] < r2[0]:
            hi, best = m2, min(best, r1, key=lambda r: r[0])
        else:
            lo, best = m1, min(best, r2, key=lambda r: r[0])
    return best


def clearance_mm(k, stride_mm, cross_section_mm, frames=121, overshoot_deg=0.0):
    """D29's figure: centre-line distance minus the link cross-section."""
    return min_link_distance(k, stride_mm, frames, overshoot_deg=overshoot_deg)[0] - cross_section_mm


def longest_stride_for_width(k, cross_section_mm, lo_mm=1.0, hi_mm=200.0,
                             frames=121, iterations=40, overshoot_deg=0.0):
    """The stride at which D29's clearance reaches zero for this cross-section.

    The distance falls as the stride grows - adjacent legs stroke in antiphase - so the
    boundary is bisected, not sampled. Returns None where the clearance never runs out
    inside the range searched, and raises where even lo_mm is already in contact. No
    stride is invented at either end.
    """
    def clear(stride):
        return clearance_mm(k, stride, cross_section_mm, frames, overshoot_deg)

    if clear(hi_mm) > 0.0:
        return None          # no contact anywhere in the range searched: no root
    if clear(lo_mm) <= 0.0:
        raise NoPose("links are within {:.4f} mm of each other even at {:.4f} mm stride"
                     .format(cross_section_mm, lo_mm))
    lo, hi = lo_mm, hi_mm
    for _ in range(iterations):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if clear(mid) > 0.0 else (lo, mid)
    return lo
