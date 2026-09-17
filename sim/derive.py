"""The double-precision analysis path: derivation and the sweep outputs.

WHY THIS IS NOT A REIMPLEMENTATION OF hex_derive
------------------------------------------------
The algorithm workstream's 21 August handoff §2 rules that config/hexapod.json has
two consumers, and that the sweep outputs must NOT be computed through the C
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
    t3 = math.radians(k.value("theta3_deg"))
    t2_nom = k.value("theta2_nom_deg")

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


# The sweep outputs, BY NAME. Outputs are reported by name and never by position
# (D415 clause 5); the order below carries no meaning and is not a numbering.
#
# MEMBERSHIP (D418, D425, D426 clause 8). The register names SEVEN outputs in terms;
# every other row here is computed with its membership NOT HELD, and coordination rules
# those rows when the pair is ruled. So this module does not describe itself as "the
# fifteen outputs": it computes sixteen names, of which seven are register-identified.
REGISTER_IDENTIFIED = (
    "femur_travel_swing_deg",       # D97 eleven; D99
    "femur_coxa_ratio_swing_avg",   # D97 twelve; D99; named by D102, D129
    "cobinding_clearance_mm",       # D97 thirteen; D148; D104 naming (D426 clause 7)
    "theta_span_deg",               # D100 fourteen
    "tau_femur_peak_kgcm",          # D208 status line, "sweep output 11"
    "foot_dz_per_quantum_mm",       # D263 output 15
    "bob_mm",                       # D86's tenth (D425 clause 1)
)
MEMBERSHIP_NOT_HELD = (
    "r_nom_mm", "body_height_mm", "theta_nom_deg", "theta_extreme_deg",
    "theta_midswing_deg", "coxa_sweep_deg", "a_eff_extreme_mm",
    "swing_duration_ms", "cycle_duration_ms",
)
OUTPUT_NAMES = [
    "r_nom_mm", "body_height_mm", "theta_nom_deg", "theta_extreme_deg",
    "theta_midswing_deg", "theta_span_deg", "femur_travel_swing_deg",
    "coxa_sweep_deg", "femur_coxa_ratio_swing_avg", "cobinding_clearance_mm",
    "bob_mm", "a_eff_extreme_mm", "tau_femur_peak_kgcm",
    "swing_duration_ms", "cycle_duration_ms", "foot_dz_per_quantum_mm",
]

# Output 15 is one output evaluated at two inputs (COREDROP_21 §1.1). Co-binding
# clearance is one output reported at one definition and NAMED at a second (D104,
# D426 clause 7): D104 "reports the more conservative and names the other with its
# definition". Both quantum inputs are read BY NAME at run time.
QUANTUM_INPUTS = ("command_step_deg", "joint_accuracy_deg")
COBINDING_REPORTED = "cobinding_clearance_mm"
COBINDING_NAMED = "cobinding_clearance_mm@span_named_d104"
NO_ROOT = None      # emitted as "no root", never a number (D425 clause 5)

OUTPUT_COLUMNS = []
for _n in OUTPUT_NAMES:
    if _n == "foot_dz_per_quantum_mm":
        OUTPUT_COLUMNS += ["foot_dz_per_quantum_mm@" + q for q in QUANTUM_INPUTS]
    elif _n == "cobinding_clearance_mm":
        OUTPUT_COLUMNS += [COBINDING_REPORTED, COBINDING_NAMED]
    else:
        OUTPUT_COLUMNS.append(_n)
del _n

OUTPUT_UNITS = {
    "r_nom_mm": "mm", "body_height_mm": "mm", "theta_nom_deg": "deg",
    "theta_extreme_deg": "deg", "theta_midswing_deg": "deg", "theta_span_deg": "deg",
    "femur_travel_swing_deg": "deg", "coxa_sweep_deg": "deg",
    "femur_coxa_ratio_swing_avg": "ratio", "cobinding_clearance_mm": "mm",
    "bob_mm": "mm", "a_eff_extreme_mm": "mm", "tau_femur_peak_kgcm": "kg*cm",
    "swing_duration_ms": "ms", "cycle_duration_ms": "ms",
    "foot_dz_per_quantum_mm": "mm per quantum",
}

# The model every swing-derived figure rests on (D426 clauses 1-2). Neither swing
# progress model is adopted, so the label travels with the column.
SWING_MODEL_LABEL = "level-body, body-frame progress"

# D101 / D148: every rate, ratio, distance and span output emits its definition and
# phase window. Written for all sixteen, so nothing has to be inferred from the code.
OUTPUT_META = {
    "r_nom_mm": ("L1 + R*cos(Theta0): implied reach at the nominal stance (D70)", "mid-stance", None),
    "body_height_mm": ("R*sin(Theta0) (D70)", "mid-stance", None),
    "theta_nom_deg": ("Theta0 = theta2_nom_deg + psi_deg (D70)", "mid-stance", None),
    "theta_extreme_deg": ("acos((sqrt(r_nom^2 + s^2) - L1) / R): stride extreme on D58's straight line", "stance extreme", None),
    "theta_midswing_deg": ("asin((body_height - swing_clearance) / R) (D99)", "mid-swing", SWING_MODEL_LABEL),
    "theta_span_deg": ("theta_nom_deg - theta_midswing_deg (D100; COREDROP_21 §2)", "whole cycle, while clearance >= bob", SWING_MODEL_LABEL),
    "femur_travel_swing_deg": ("2 * (theta_extreme_deg - theta_midswing_deg) (D99)", "one swing phase", SWING_MODEL_LABEL),
    "coxa_sweep_deg": ("2 * atan(s / r_nom): coxa travel over one swing (D58, D99)", "one swing phase", SWING_MODEL_LABEL),
    "femur_coxa_ratio_swing_avg": ("coxa_sweep_deg / femur_travel_swing_deg: AVERAGE-RATE ratio over the swing (D99, D102, D129). Above 1 the coxa sets the swing duration; below 1 the femur does and rows swing_duration_ms and cycle_duration_ms understate (D425 clause 6)", "one swing phase", SWING_MODEL_LABEL),
    "cobinding_clearance_mm": ("REPORTED (D97, D148, D418 clause 2): the swing clearance at which femur_travel_swing_deg = coxa_sweep_deg, c = body_height - R*sin(Theta_extreme - coxa_sweep/2), searched over every clearance for which theta_midswing is defined, |sin| <= 1; 'no root' otherwise (D425 clause 5). NAMED (D104, D426 clause 7): the clearance at which theta_span_deg = coxa_sweep_deg, c = body_height - R*sin(Theta0 - coxa_sweep), same domain rule", "one swing phase", SWING_MODEL_LABEL),
    "bob_mm": ("body_height - R*sin(Theta_extreme): geometric bob; the quantisation term is output 15 (D86, D425 clause 2)", "stance, nominal to extreme", None),
    "a_eff_extreme_mm": ("R*cos(Theta_extreme)", "stance extreme", None),
    "tau_femur_peak_kgcm": ("(mass_kg / tripod_support_legs) * a_eff_extreme_mm / 10 (D208)", "stance extreme", None),
    "swing_duration_ms": ("1000 * coxa_sweep_deg * swing_peak_factor / dtheta_peak_deg_s: set from the coxa", "one swing phase", SWING_MODEL_LABEL),
    "cycle_duration_ms": ("swing_duration_ms / (1 - duty_factor)", "whole cycle", SWING_MODEL_LABEL),
    "foot_dz_per_quantum_mm": ("a_eff_nom_mm * q_deg * pi / 180, LINEARISED, at q = command_step_deg and at q = joint_accuracy_deg (D263; COREDROP_21 §1)", "nominal stance", None),
}

# Kept for callers written against 16 September's module: the expression text.
OUTPUT_EXPRESSIONS = {n: m[0] for n, m in OUTPUT_META.items()}


def _cobinding_root(height_mm, rigid_len_mm, angle_deg):
    """c = height - R*sin(angle), provided the angle is one theta_midswing can take.

    theta_midswing = asin((height - c)/R) is defined for every c with |height - c| <= R,
    and asin's range is [-90, +90] degrees. A required mid-swing angle outside that
    range has no clearance behind it: 'no root', never a clamped or extrapolated number.
    """
    if not -90.0 <= angle_deg <= 90.0:
        return NO_ROOT
    return height_mm - rigid_len_mm * math.sin(math.radians(angle_deg))


class UnreachableStride(Exception):
    """The stride extreme lies outside the leg's reach. HEX_CFG_E_REACH."""


def sweep_point(k, stride_mm, duty_factor):
    """One row of the sweep: the named outputs at one (stride, duty).

    The row holds OUTPUT_COLUMNS in that order: output 15 is one output at two
    inputs, and co-binding clearance is reported at one definition and named at a
    second (D104). Every column is a float, with two exceptions that hold None: the
    co-binding columns where no clearance satisfies their equality ('no root'), and
    the ratio where the swing has no lift.

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
    bob = d.body_height_mm - R * math.sin(math.radians(theta_extreme))
    femur_travel = 2.0 * (theta_extreme - theta_midswing)
    # A ratio over zero or negative femur travel is not a figure: below bob the swing
    # has no lift (D126 guard 1). The row stays computable - the D126 guard tests walk
    # clearance through that edge on purpose - and the ratio column says so with None.
    ratio = coxa_sweep / femur_travel if femur_travel > 0.0 else None

    row = OrderedDict([
        ("r_nom_mm", d.r_nom_mm),
        ("body_height_mm", d.body_height_mm),
        ("theta_nom_deg", d.theta_nom_deg),
        ("theta_extreme_deg", theta_extreme),
        ("theta_midswing_deg", theta_midswing),
        # 6. D100's span over the WHOLE cycle: nominal to mid-swing. It equals the
        #    cycle peak-to-peak only while theta_midswing <= theta_extreme, which is
        #    swing_clearance_mm >= bob_mm; the trace carries that predicate. The
        #    stance-only span (nominal to extreme) is a different quantity and is
        #    kept in the trace as a diagnostic, never as this output.
        ("theta_span_deg", d.theta_nom_deg - theta_midswing),
        ("femur_travel_swing_deg", femur_travel),
        ("coxa_sweep_deg", coxa_sweep),
        # D97 twelve, named by D102: an AVERAGE-RATE ratio over one swing, coxa over femur.
        ("femur_coxa_ratio_swing_avg", ratio),
        # D97 thirteen. Reported: femur travel equals coxa travel, i.e.
        # 2(Theta_ext - Theta_mid) = coxa  =>  Theta_mid = Theta_ext - coxa/2.
        (COBINDING_REPORTED, _cobinding_root(d.body_height_mm, R, theta_extreme - coxa_sweep / 2.0)),
        # D104's second definition, NAMED beside it: span equals coxa travel, i.e.
        # Theta0 - Theta_mid = coxa  =>  Theta_mid = Theta0 - coxa.
        (COBINDING_NAMED, _cobinding_root(d.body_height_mm, R, d.theta_nom_deg - coxa_sweep)),
        ("bob_mm", bob),
        ("a_eff_extreme_mm", a_eff_extreme),
        # D208: the TRIPOD SHARE, not whole-robot mass. Single-leg support needs five
        # feet off the ground - a fault, not a transient - so the whole-mass product is
        # retired as a constraint and emitted as a diagnostic instead (see below).
        ("tau_femur_peak_kgcm",
         k.value("mass_kg") / k.value("tripod_support_legs") * a_eff_extreme / 10.0),
        ("swing_duration_ms", swing_ms),
        ("cycle_duration_ms", cycle_ms),
    ])
    # Output 15 (D263, defined by Spider AI in COREDROP_21 §1). foot_dz_per_quantum_mm:
    # LEG-frame z step of the foot for one femur quantum at the nominal stance, LINEARISED,
    # = a_eff_nom_mm * q_deg * pi/180, evaluated at q = command_step_deg and at
    # q = joint_accuracy_deg, both read by name. The linear form is the one that reproduces
    # D189's and D230's four printed figures; the exact difference reproduces none.
    for q in QUANTUM_INPUTS:
        row["foot_dz_per_quantum_mm@" + q] = (
            d.a_eff_nom_mm * k.value(q) * math.pi / 180.0)

    # D208 diagnostic. NOT one of the fourteen: it is reported, never asserted on.
    # At the D209 ceiling it reads 24.0000 kg*cm, 1.2000x stall, and that is not a
    # failure - the vendor's own posture exceeds the same form by 1.3487x while walking.
    diagnostics = OrderedDict([
        ("tau_femur_singleleg_kgcm", k.value("mass_kg") * a_eff_extreme / 10.0),
        # The stance-only span. NOT output 6 (D100 defines output 6 over the cycle);
        # kept so the difference stays visible and so a ruling the other way is a
        # one-line swap rather than a re-derivation.
        ("theta_span_stance_deg", d.theta_nom_deg - theta_extreme),
        # D415 clause 3: a derived convenience, labelled as one. Not a sweep output.
        ("body_speed_mm_s", 1000.0 * stride_mm / cycle_ms),
        # COREDROP_21 §2.3: output 6 is the cycle peak-to-peak only while this holds.
        ("span_predicate_holds", clearance >= bob),
        # D425 clause 6: rows swing_duration_ms and cycle_duration_ms are set from the
        # coxa. Where the ratio is below 1 the femur travels further and they understate.
        ("swing_duration_understated", None if ratio is None else ratio < 1.0),
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
        ("theta2_nom_deg", k.value("theta2_nom_deg")),
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

    With output 6 as D100 defines it (Theta_0 - Theta_mid), span == travel is
        Theta_0 - Theta_mid = 2 (Theta_extreme - Theta_mid)
        =>  Theta_mid = 2 Theta_extreme - Theta_0
    which is exactly this closed form. So D125's prose ("span equal to travel") and
    D211's closed form are ONE point. They only looked like two while output 6 was
    computed as the stance span; tests/test_d126_guards.py records the history.

    D211's binding rule, which this function exists to respect:
    AN INTERMEDIATE QUANTITY IS NEVER ROUNDED BEFORE IT IS MULTIPLIED. Rounding
    Theta_extreme to 4 dp before the 2x gives 9.7483; at 3 dp it gives 9.7486; the
    true value is 9.74822449. The 2x amplifier has produced three errors in this one
    quantity across the project's history.
    """
    d = derive(k)
    R = d.rigid_len_mm
    L1 = k.value("coxa_length_mm")
    s_half = k.value("stride_mm") / 2.0
    theta_extreme = math.degrees(
        math.acos((math.sqrt(d.r_nom_mm ** 2 + s_half ** 2) - L1) / R))
    return d.body_height_mm - R * math.sin(
        math.radians(2.0 * theta_extreme - d.theta_nom_deg))


def a_eff_max_mm(k):
    """The moment-arm ceiling. D209 quotes 111.6279 mm; it is NOT stored.

        tau_femur_peak * margin_factor <= tau_servo
        (m / legs) * a_eff / 10 * margin <= tau_servo

    It moves with the servo torque and the mass, both of which take their measured
    values when measured (D261). margin_factor is fixed at 2.5000 by D261, which
    struck D209's automatic revert to 3.0000; the re-sweep still PRESENTS a second
    column at 3.0000 because D415 clause 7 directs it. Storing the number is exactly
    the defect the hard-coded-constant guard exists to catch.
    """
    return (k.value("tau_servo_kgcm") * 10.0 * k.value("tripod_support_legs")
            / (k.value("mass_kg") * k.value("margin_factor")))


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
