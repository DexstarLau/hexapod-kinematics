"""The double-precision analysis path: derivation and the fourteen sweep outputs.

WHY THIS IS NOT A REIMPLEMENTATION OF hex_derive
------------------------------------------------
The algorithm workstream's 21 August handoff §2 rules that config/hexapod.json has
two consumers, and that the fourteen outputs must NOT be computed through the C
float path:

    config/hexapod.json
       |
       +-- sim/constants.py  --> double  --> the sweep, the emitted tables   <- this file
       |
       +-- the caller        --> hex_config_t (float) --> ik_core at 50 Hz

hex_config_t is float because the frozen API is float and the target is an
ESP32-S3, and single precision does not reliably carry the four decimal places
D147 requires on a 100 mm quantity. So the analysis path is deliberately separate
and deliberately double.

That separation is only safe if the two are checked against each other. When
hex_config.c lands on 27 August, tests/test_c_agreement.py compares this module's
output against hex_derive() within float tolerance. Until then this module stands
alone and says so.

THE IDENTITY THIS RESTS ON
--------------------------
With theta3 held constant the three-member leg collapses EXACTLY onto a
two-member leg. This is an identity, not an approximation:

    L2*cos(t2) + L3*cos(t2+t3)  =  R*cos(t2+psi)
    L2*sin(t2) + L3*sin(t2+t3)  =  R*sin(t2+psi)

    R    = sqrt(L2^2 + L3^2 + 2*L2*L3*cos(theta3))
    psi  = atan2( L3*sin(theta3), L2 + L3*cos(theta3) )

TRAP, named by the algorithm workstream as the most likely error in this file:
theta2_nom_deg is what the SERVO COMMANDS. Theta is what the GEOMETRY SEES. They
differ by psi, and psi is zero only when L3 is zero. Every expression below that
takes a sine or cosine of a femur angle uses Theta, never theta2.
"""

import math
from collections import OrderedDict


class Derived(object):
    """The hex_derived_t quantities, in double.

    Fields mirror hex_derived_t so that the 27 August agreement test can compare
    them field by field.
    """

    __slots__ = ("rigid_len_mm", "psi_deg", "theta_nom_deg",
                 "a_eff_nom_mm", "body_height_mm", "r_nom_mm")

    def __init__(self, **kw):
        for k in self.__slots__:
            setattr(self, k, kw[k])

    def as_dict(self):
        return OrderedDict((k, getattr(self, k)) for k in self.__slots__)


def swing_peak_factor(profile_name):
    """peak divided by mean, for the named velocity profile.

    D145 selects half-sine, whose ratio is pi/2. The ratio is COMPUTED from the
    profile rather than stored, because it is a consequence of the choice of
    profile. Storing it would let the profile change while the ratio silently
    did not - which is the failure the hard-coded-constant guard exists to catch.
    """
    if profile_name == "half_sine":
        return math.pi / 2.0
    raise ValueError("no peak/mean ratio defined for profile {!r}".format(profile_name))


def derive(k):
    """hex_derived_t from the constant table. Mirrors hex_derive(), in double."""
    L1 = k.value("coxa_length_mm")
    L2 = k.value("femur_length_mm")
    L3 = k.value("tibia_length_mm")
    t3 = math.radians(k.value("theta_3_deg"))
    t2_nom = k.value("theta_2_neutral_deg")

    R = math.sqrt(L2 * L2 + L3 * L3 + 2.0 * L2 * L3 * math.cos(t3))
    psi = math.degrees(math.atan2(L3 * math.sin(t3), L2 + L3 * math.cos(t3)))

    theta_nom = t2_nom + psi                      # Theta0, NOT theta2_nom
    theta_nom_rad = math.radians(theta_nom)

    a_eff_nom = R * math.cos(theta_nom_rad)
    body_height = R * math.sin(theta_nom_rad)

    return Derived(
        rigid_len_mm=R,
        psi_deg=psi,
        theta_nom_deg=theta_nom,
        a_eff_nom_mm=a_eff_nom,
        body_height_mm=body_height,
        r_nom_mm=L1 + a_eff_nom,
    )


# The fourteen outputs. Order and expressions are the algorithm workstream's §3.
OUTPUT_NAMES = [
    "r_nom_mm", "body_height_mm", "theta_nom_deg", "theta_extreme_deg",
    "theta_midswing_deg", "theta_span_deg", "femur_travel_swing_deg",
    "coxa_sweep_deg", "bob_mm", "a_eff_extreme_mm", "tau_femur_peak_kgcm",
    "swing_duration_ms", "cycle_duration_ms", "body_speed_mm_s",
]

OUTPUT_UNITS = {
    "r_nom_mm": "mm", "body_height_mm": "mm", "theta_nom_deg": "deg",
    "theta_extreme_deg": "deg", "theta_midswing_deg": "deg", "theta_span_deg": "deg",
    "femur_travel_swing_deg": "deg", "coxa_sweep_deg": "deg", "bob_mm": "mm",
    "a_eff_extreme_mm": "mm", "tau_femur_peak_kgcm": "kg*cm",
    "swing_duration_ms": "ms", "cycle_duration_ms": "ms", "body_speed_mm_s": "mm/s",
}


class UnreachableStride(Exception):
    """The stride extreme lies outside the leg's reach. HEX_CFG_E_REACH."""


def sweep_point(k, stride_mm, duty_factor):
    """One row of the sweep: the fourteen outputs at one (stride, duty).

    Returns (row, derived, trace) where trace carries the intermediate angles the
    span guard must recompute from - never from the stored outputs.
    """
    d = derive(k)
    L1 = k.value("coxa_length_mm")
    R = d.rigid_len_mm
    s = stride_mm / 2.0

    # 4. theta at the stride extreme.
    #    D58 TRAP: the stance foot path is the no-slip STRAIGHT LINE. That is why
    #    the reach at the extreme is sqrt(r_nom^2 + s^2) - the hypotenuse - and not
    #    r_nom held constant. asin here would give the constant-radius arc, which
    #    scuffs, and D11 rejects scuff.
    reach_extreme = math.sqrt(d.r_nom_mm ** 2 + s ** 2)
    cos_extreme = (reach_extreme - L1) / R
    if not -1.0 <= cos_extreme <= 1.0:
        raise UnreachableStride(
            "stride {:.4f} mm needs reach {:.4f} mm from the femur axis but R is only "
            "{:.4f} mm (HEX_CFG_E_REACH)".format(stride_mm, reach_extreme - L1, R))
    theta_extreme = math.degrees(math.acos(cos_extreme))

    # 5. theta at mid-swing: the foot is lifted swing_clearance above the ground.
    clearance = k.value("swing_clearance_mm")
    sin_mid = (d.body_height_mm - clearance) / R
    if not -1.0 <= sin_mid <= 1.0:
        raise UnreachableStride(
            "swing clearance {:.4f} mm is not achievable at body height {:.4f} mm "
            "(HEX_CFG_E_CLEARANCE)".format(clearance, d.body_height_mm))
    theta_midswing = math.degrees(math.asin(sin_mid))

    # 8. coxa sweep. D58 again: atan, not asin.
    coxa_sweep = 2.0 * math.degrees(math.atan(s / d.r_nom_mm))

    a_eff_extreme = R * math.cos(math.radians(theta_extreme))
    peak_factor = swing_peak_factor(k.value("swing_velocity_profile"))
    dtheta_peak = k.value("dtheta_peak_deg_s")

    swing_ms = 1000.0 * coxa_sweep * peak_factor / dtheta_peak
    cycle_ms = swing_ms / (1.0 - duty_factor)

    row = OrderedDict([
        ("r_nom_mm", d.r_nom_mm),
        ("body_height_mm", d.body_height_mm),
        ("theta_nom_deg", d.theta_nom_deg),
        ("theta_extreme_deg", theta_extreme),
        ("theta_midswing_deg", theta_midswing),
        ("theta_span_deg", d.theta_nom_deg - theta_extreme),
        ("femur_travel_swing_deg", 2.0 * (theta_extreme - theta_midswing)),
        ("coxa_sweep_deg", coxa_sweep),
        ("bob_mm", d.body_height_mm - R * math.sin(math.radians(theta_extreme))),
        ("a_eff_extreme_mm", a_eff_extreme),
        # D208: the TRIPOD SHARE, not whole-robot mass. Single-leg support needs five
        # feet off the ground - a fault, not a transient - so the whole-mass product is
        # retired as a constraint and emitted as a diagnostic instead (see below).
        ("tau_femur_peak_kgcm",
         k.value("payload_mass_kg") / k.value("tripod_support_legs") * a_eff_extreme / 10.0),
        ("swing_duration_ms", swing_ms),
        ("cycle_duration_ms", cycle_ms),
        ("body_speed_mm_s", 1000.0 * stride_mm / cycle_ms),
    ])

    # D208 diagnostic. NOT one of the fourteen: it is reported, never asserted on.
    # At the D209 ceiling it reads 24.0000 kg*cm, 1.2000x stall, and that is not a
    # failure - the vendor's own posture exceeds the same form by 1.3487x while walking.
    diagnostics = OrderedDict([
        ("tau_femur_singleleg_kgcm", k.value("payload_mass_kg") * a_eff_extreme / 10.0),
    ])

    # The trace exists so that the span guard can recompute output 6 from the
    # geometry rather than from outputs 3 and 4. Comparing output 6 against
    # output 3 minus output 4 tests nothing at all.
    trace = OrderedDict([
        ("stride_mm", stride_mm),
        ("duty_factor", duty_factor),
        ("half_stride_mm", s),
        ("rigid_len_mm", R),
        ("coxa_length_mm", L1),
        ("psi_deg", d.psi_deg),
        ("theta2_nom_deg", k.value("theta_2_neutral_deg")),
        ("reach_extreme_mm", reach_extreme),
        ("swing_clearance_mm", clearance),
        ("swing_peak_factor", peak_factor),
        ("dtheta_peak_deg_s", dtheta_peak),
    ])
    trace.update(diagnostics)
    return row, d, trace


def sweep(k, strides, duties):
    """Every (stride, duty) combination. Returns a list of (row, derived, trace)."""
    return [sweep_point(k, st, du) for st in strides for du in duties]


def coincidence_clearance_mm(k):
    """The swing clearance at which theta_span_deg and femur_travel_swing_deg coincide.

    D125 withdrew a guard asserting these two are never equal. They ARE equal at an
    entirely ordinary geometry, and in the collapse the withdrawn guard was written
    to catch they are maximally unequal. D211 gives the closed form:

        c = h0 - R*sin(2*Theta_extreme - Theta_0)

    D211's binding rule, which this function exists to respect:
    AN INTERMEDIATE QUANTITY IS NEVER ROUNDED BEFORE IT IS MULTIPLIED. Rounding
    Theta_extreme to 4 dp before the 2x gives 9.7483; at 3 dp it gives 9.7486; the
    true value is 9.74822449. The 2x amplifier has produced three errors in this one
    quantity across the project's history.
    """
    d = derive(k)
    R = d.rigid_len_mm
    L1 = k.value("coxa_length_mm")
    s_half = k.value("nominal_stride_mm") / 2.0
    theta_extreme = math.degrees(
        math.acos((math.sqrt(d.r_nom_mm ** 2 + s_half ** 2) - L1) / R))
    return d.body_height_mm - R * math.sin(
        math.radians(2.0 * theta_extreme - d.theta_nom_deg))


def a_eff_max_mm(k):
    """The moment-arm ceiling. D209 quotes 111.6279 mm; it is NOT stored.

        tau_femur_peak * margin_factor <= tau_servo
        (m / legs) * a_eff / 10 * margin <= tau_servo

    It moves with the servo torque, the mass and the margin, all three of which are
    live: mass is unmeasured, and the margin reverts to 3.0000 automatically if D190
    measures loaded torque below 16 kg*cm or mass above 2.30 kg. Storing the number
    is exactly the defect the hard-coded-constant guard exists to catch.
    """
    return (k.value("stall_torque_kgcm") * 10.0 * k.value("tripod_support_legs")
            / (k.value("payload_mass_kg") * k.value("torque_margin")))


# ---------------------------------------------------------------- body frame

LEG_ORDER = ("R1", "R2", "R3", "L1", "L2", "L3")


def body_to_leg(k, leg, bx_mm, by_mm, bz_mm):
    """Body frame -> leg frame, double precision analysis path.

    Mirrors ik_body_to_leg. beta_neutral_deg is NOT applied: it is a command
    offset, not a frame rotation, and folding the two together is the D23 error.

    Body frame is X forward, Y left, Z up, origin at the coxa centroid (D227).
    """
    pos = k.value("coxa_positions_mm")[leg]
    beta = math.radians(k.value("beta_mount_deg")[leg])
    dx, dy = bx_mm - pos[0], by_mm - pos[1]
    c, s = math.cos(beta), math.sin(beta)
    return (c * dx + s * dy, -s * dx + c * dy, bz_mm)


def foot_position_body(k, leg, theta1_deg, theta2_deg):
    """Foot position in the BODY frame, from the two commanded joint angles.

    The leg lies along beta_mount_deg when the commanded coxa angle equals
    beta_neutral_deg, so the yaw actually applied is
    beta_mount + (theta1 - beta_neutral).
    """
    d = derive(k)
    R = d.rigid_len_mm
    theta = math.radians(theta2_deg + d.psi_deg)
    reach = k.value("coxa_length_mm") + R * math.cos(theta)
    height = -R * math.sin(theta)

    yaw = math.radians(k.value("beta_mount_deg")[leg]
                       + theta1_deg - k.value("beta_neutral_deg")[leg])
    pos = k.value("coxa_positions_mm")[leg]
    return (pos[0] + reach * math.cos(yaw),
            pos[1] + reach * math.sin(yaw),
            height)


def corner_legs(k):
    """The four legs whose coxa axis is NOT radial from the body centre.

    Solved for, not listed: a leg is a corner leg when |beta_mount| differs from
    its own position angle. D227 measures that difference at 13.3626 deg on four
    legs and 0.03 deg on the middle two.
    """
    out = []
    for leg in LEG_ORDER:
        x, y = k.value("coxa_positions_mm")[leg]
        position_angle = math.degrees(math.atan2(y, x))
        offset = k.value("beta_mount_deg")[leg] - position_angle
        if abs(offset) > 1.0:
            out.append(leg)
    return out
