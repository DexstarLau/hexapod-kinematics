"""The P1-prime stance pose, in double: Mini Project's own implementation of D414.

WHAT THIS IS, AND WHAT IT IS NOT
--------------------------------
D414 adopts P1-prime as the stance solution: with the commanded x, y and yaw
spent, the body keeps three freedoms - height, pitch and roll - and three stance
legs impose exactly three scalar constraints, so the pose that keeps all three
stance feet planted is solved per frame.

This module is the analysis path for the D197 re-sweep (D411 clause 3, D415). It
is NOT gait_core, it is not in core/, and nothing here is an engine. It exists so
that the (posture, stride) search is run on committed code in this repository
(D366) rather than on figures carried from another workstream's harness.

WHY ONE CONSTRAINT PER LEG, NOT THREE
-------------------------------------
With theta3 fixed, the leg is two members: a coxa yaw and a femur pitch. Its foot
can be anywhere on the surface

    (rho - L1)^2 + lz^2 = R^2          rho: horizontal distance from the coxa axis
                                        lz:  height relative to the femur axis

because theta1 spins that circle freely about the vertical axis. A planted foot is
reachable exactly when it lies on that surface - one scalar equation. Three
stance legs, three equations, three unknowns.

THE TWO SWING MEASURES
----------------------
P1-prime pitches the body. The swing legs are not re-planned here (D414 clause 7
routes re-planning to Spider AI), so how low a swing foot goes depends on what
the swing leg is assumed to be doing. Two readings are computed and both are
reported, by name, never merged:

    swing_z_d414_mm      the swing foot held at its nominal horizontal position
                         with the FULL swing_clearance_mm lift, at every stance
                         frame. This is the reading behind D414's recorded
                         figures (-12.6688 mm at 70.0098 deg; 33.6056 mm stride).

    swing_z_profile_mm   the swing foot on the swing trajectory the shipped
                         gait_core v1 commands - half-sine stride fraction,
                         lift = swing_clearance_mm * sin(pi * u) - planned in
                         the level body frame and solved by the 2-DOF leg with
                         radial projection (hold height, clamp reach), which is
                         what IK_PROJ_RADIAL does. Its lift is ZERO at lift-off
                         and touch-down, where the body pitch is largest.

Floor is world z = 0. A negative value is a foot below the floor.

CONVENTIONS
-----------
Body frame X forward, Y left, Z up, origin at the coxa centroid (D227), exactly
as sim/derive.py. Body attitude is Ry(pitch) * Rx(roll). Footholds are the
nominal feet at mid-stance with the body level at body_height_mm; the body moves
in pure +x from -stride/2 to +stride/2 over one stance.

Tripods mirror core/src/gait_core.c v1's assignment (R1, R3, L2 / R2, L1, L3),
which that file records as its own design choice under D107 - no decision fixes
it. Read, not transcribed: it is two tuples of leg names.
"""

import math
from collections import OrderedDict

from sim import derive as D

TRIPODS = (("R1", "R3", "L2"), ("R2", "L1", "L3"))

# Newton stops when every stance foot is within this distance of its reachable
# surface. It is a solver tolerance, not a measurement of a robot.
SURFACE_TOL_MM = 1e-9
MAX_ITERATIONS = 60


class NoPose(Exception):
    """No body pose keeps all three stance feet on their reachable surfaces."""


def rotation(pitch_rad, roll_rad):
    """Ry(pitch) * Rx(roll), as rows."""
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    return ((cp, sp * sr, sp * cr),
            (0.0, cr, -sr),
            (-sp, cp * sr, cp * cr))


def _apply(m, v):
    return tuple(m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2] for i in range(3))


def _apply_transpose(m, v):
    return tuple(m[0][i] * v[0] + m[1][i] * v[1] + m[2][i] * v[2] for i in range(3))


def nominal_foot_body(k, d, leg):
    """Nominal foot in the BODY frame: r_nom_mm along beta_mount, body_height_mm down."""
    pos = k.value("coxa_positions_mm")[leg]
    beta = math.radians(k.value("beta_mount_deg")[leg])
    return (pos[0] + d.r_nom_mm * math.cos(beta),
            pos[1] + d.r_nom_mm * math.sin(beta),
            -d.body_height_mm)


def foothold_world(k, d, leg):
    """The planted foot: the nominal foot at mid-stance, level body, on the floor."""
    x, y, _ = nominal_foot_body(k, d, leg)
    return (x, y, 0.0)


def world_to_body(pose, body_x_mm, p_world):
    height, pitch, roll = pose
    m = rotation(pitch, roll)
    return _apply_transpose(m, (p_world[0] - body_x_mm, p_world[1], p_world[2] - height))


def body_to_world(pose, body_x_mm, p_body):
    height, pitch, roll = pose
    w = _apply(rotation(pitch, roll), p_body)
    return (w[0] + body_x_mm, w[1], w[2] + height)


def surface_distance_mm(k, d, leg, p_body):
    """Signed distance of a body-frame point from the leg's reachable surface."""
    lx, ly, lz = D.body_to_leg(k, leg, *p_body)
    return math.hypot(math.hypot(lx, ly) - k.value("coxa_length_mm"), lz) - d.rigid_len_mm


def joint_angles(k, d, leg, p_body):
    """(theta1_deg, theta2_deg), kinematic, for a body-frame point ON the surface.

    theta1 is the commanded coxa angle: the leg-frame yaw plus beta_neutral_deg,
    the inverse of derive.foot_position_body. theta2 subtracts psi (the trap named
    at the top of sim/derive.py).
    """
    lx, ly, lz = D.body_to_leg(k, leg, *p_body)
    theta1 = math.degrees(math.atan2(ly, lx)) + k.value("beta_neutral_deg")[leg]
    big_theta = math.degrees(math.atan2(-lz, math.hypot(lx, ly) - k.value("coxa_length_mm")))
    return theta1, big_theta - d.psi_deg


def _solve3(a, b):
    """3x3 linear solve, Gaussian elimination with partial pivoting."""
    m = [list(a[i]) + [b[i]] for i in range(3)]
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-14:
            raise NoPose("singular Jacobian")
        m[col], m[piv] = m[piv], m[col]
        for r in range(col + 1, 3):
            f = m[r][col] / m[col][col]
            for c in range(col, 4):
                m[r][c] -= f * m[col][c]
    x = [0.0, 0.0, 0.0]
    for r in (2, 1, 0):
        x[r] = (m[r][3] - sum(m[r][c] * x[c] for c in range(r + 1, 3))) / m[r][r]
    return x


def solve_pose(k, d, legs, body_x_mm, guess):
    """Newton on (height, pitch, roll). Returns (pose, worst |distance| mm).

    Raises NoPose rather than returning a best effort: a pose that does not plant
    the feet is not a stance pose, and supplying one would be a default for a
    missing value.
    """
    holds = [foothold_world(k, d, leg) for leg in legs]

    def residual(pose):
        return [surface_distance_mm(k, d, leg, world_to_body(pose, body_x_mm, h))
                for leg, h in zip(legs, holds)]

    pose = list(guess)
    for _ in range(MAX_ITERATIONS):
        f = residual(pose)
        worst = max(abs(v) for v in f)
        if worst < SURFACE_TOL_MM:
            return tuple(pose), worst
        step = (1e-6, 1e-8, 1e-8)
        jac = [[0.0] * 3 for _ in range(3)]
        for j in range(3):
            probe = list(pose)
            probe[j] += step[j]
            fp = residual(probe)
            for i in range(3):
                jac[i][j] = (fp[i] - f[i]) / step[j]
        dx = _solve3(jac, [-v for v in f])
        pose = [pose[i] + dx[i] for i in range(3)]
    raise NoPose("Newton did not converge in {} iterations at body_x {:.4f} mm".format(
        MAX_ITERATIONS, body_x_mm))


def _half_sine_fraction(u):
    return 0.5 * (1.0 - math.cos(math.pi * u))


def swing_foot_profile_body(k, d, leg, u, stride_mm):
    """Body-frame swing foot on gait_core v1's commanded swing, after the 2-DOF solve.

    Target: nominal + (-1/2 + half_sine(u)) * stride along +x, lifted by
    swing_clearance_mm * sin(pi u), planned in the LEVEL body frame. The leg can
    only reach its surface, so the target is projected the way IK_PROJ_RADIAL
    does: hold the height, clamp the radial reach.
    """
    nx, ny, nz = nominal_foot_body(k, d, leg)
    frac = -0.5 + _half_sine_fraction(u)
    lift = k.value("swing_clearance_mm") * math.sin(math.pi * u)
    lx, ly, lz = D.body_to_leg(k, leg, nx + frac * stride_mm, ny, nz + lift)
    R = d.rigid_len_mm
    if abs(lz) > R:
        raise NoPose("swing target height {:.4f} mm is beyond R {:.4f} mm".format(lz, R))
    reach = k.value("coxa_length_mm") + math.sqrt(R * R - lz * lz)
    yaw = math.atan2(ly, lx)
    lx, ly = reach * math.cos(yaw), reach * math.sin(yaw)
    pos = k.value("coxa_positions_mm")[leg]
    beta = math.radians(k.value("beta_mount_deg")[leg])
    c, s = math.cos(beta), math.sin(beta)
    return (pos[0] + c * lx - s * ly, pos[1] + s * lx + c * ly, lz)


def stance_cycle(k, stride_mm, frames=101):
    """Both tripods through one stance each. Returns a list of per-frame records.

    Frame i of tripod g is stance progress u = i / (frames - 1) for the three legs
    of TRIPODS[g], and swing progress u for the other three. That pairing is only
    true at duty_factor 0.5, so any other duty factor is refused, not assumed.
    """
    if k.value("duty_factor") != 0.5:
        raise ValueError("stance_cycle pairs stance and swing frames one-to-one, "
                         "which holds only at duty_factor 0.5; got {}".format(
                             k.value("duty_factor")))
    if frames < 3:
        raise ValueError("need at least 3 frames per stance")
    d = D.derive(k)
    clearance = k.value("swing_clearance_mm")
    out = []
    for g, stance in enumerate(TRIPODS):
        swing = TRIPODS[1 - g]
        guess = (d.body_height_mm, 0.0, 0.0)          # level start for each tripod
        for i in range(frames):
            u = i / float(frames - 1)
            body_x = (u - 0.5) * stride_mm
            pose, worst = solve_pose(k, d, stance, body_x, guess)
            guess = pose
            rec = OrderedDict(tripod=g, frame=i, u=u, body_x_mm=body_x,
                              height_mm=pose[0], pitch_deg=math.degrees(pose[1]),
                              roll_deg=math.degrees(pose[2]), worst_surface_mm=worst)
            for leg in stance:
                pb = world_to_body(pose, body_x, foothold_world(k, d, leg))
                rec["theta1_" + leg], rec["theta2_" + leg] = joint_angles(k, d, leg, pb)
            for leg in swing:
                nx, ny, nz = nominal_foot_body(k, d, leg)
                rec["swing_z_d414_" + leg] = body_to_world(
                    pose, body_x, (nx, ny, nz + clearance))[2]
                pb = swing_foot_profile_body(k, d, leg, u, stride_mm)
                rec["swing_z_profile_" + leg] = body_to_world(pose, body_x, pb)[2]
                rec["theta1_" + leg], rec["theta2_" + leg] = joint_angles(k, d, leg, pb)
            out.append(rec)
    return out


def summarise(cycle):
    """Peak-to-peak attitude and height, worst planting error, both swing minima,
    joint ranges per leg, and the pose jump at each tripod hand-over."""
    def col(prefix):
        return [v for r in cycle for key, v in r.items() if key.startswith(prefix)]

    pitch = [r["pitch_deg"] for r in cycle]
    roll = [r["roll_deg"] for r in cycle]
    height = [r["height_mm"] for r in cycle]
    s = OrderedDict()
    s["pitch_pp_deg"] = max(pitch) - min(pitch)
    s["roll_pp_deg"] = max(roll) - min(roll)
    s["height_pp_mm"] = max(height) - min(height)
    s["worst_surface_mm"] = max(r["worst_surface_mm"] for r in cycle)
    s["min_swing_z_d414_mm"] = min(col("swing_z_d414_"))
    s["min_swing_z_profile_mm"] = min(col("swing_z_profile_"))
    t2 = col("theta2_")
    s["theta2_min_deg"], s["theta2_max_deg"] = min(t2), max(t2)
    for leg in D.LEG_ORDER:
        t1 = [r["theta1_" + leg] for r in cycle]
        s["theta1_min_deg_" + leg], s["theta1_max_deg_" + leg] = min(t1), max(t1)
        t2l = [r["theta2_" + leg] for r in cycle]
        s["theta2_min_deg_" + leg], s["theta2_max_deg_" + leg] = min(t2l), max(t2l)
    # hand-over: the last frame of one tripod and the first of the other are the
    # same instant. A jump here is a step the body would have to take in zero time.
    frames = len(cycle) // 2
    a_end, b_start = cycle[frames - 1], cycle[frames]
    s["handover_pitch_jump_deg"] = abs(a_end["pitch_deg"] - b_start["pitch_deg"])
    s["handover_roll_jump_deg"] = abs(a_end["roll_deg"] - b_start["roll_deg"])
    s["handover_height_jump_mm"] = abs(a_end["height_mm"] - b_start["height_mm"])
    return s


def longest_stride(k, measure, lo_mm=1.0, hi_mm=60.0, frames=101, iterations=50):
    """Longest stride in [lo, hi] at which the named swing measure stays >= 0.

    measure is 'min_swing_z_d414_mm' or 'min_swing_z_profile_mm'. Returns hi if
    hi itself clears, and raises if lo does not - no stride is invented.
    """
    def clears(stride):
        try:
            return summarise(stance_cycle(_with_stride(k, stride), stride, frames))[measure] >= 0.0
        except NoPose:
            return False

    if clears(hi_mm):
        return hi_mm
    if not clears(lo_mm):
        raise NoPose("{} is negative even at {:.4f} mm stride".format(measure, lo_mm))
    lo, hi = lo_mm, hi_mm
    for _ in range(iterations):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if clears(mid) else (lo, mid)
    return lo


class _with_stride(object):
    """The constant table with stride_mm replaced; everything else passes through."""

    def __init__(self, base, stride_mm):
        self._base, self._stride = base, stride_mm

    def value(self, name):
        return self._stride if name == "stride_mm" else self._base.value(name)
