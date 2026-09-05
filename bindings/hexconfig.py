"""ctypes binding to the algorithm workstream's C cores.

Three jobs, in this order:

1.  ASSERT THE LAYOUT.  The C library reports (name, offset, size) for every
    member of hex_config_t through hex_config_field_*.  We compare all three
    against the ctypes Structure.  sizeof alone is not sufficient: two layouts
    can share a total size while every offset after a swapped field is wrong.

2.  PROJECT THE JOINT ENVELOPES.  config/hexapod.json holds the vendor's
    envelope table in SERVO-COMMAND space, where the two sides are mirrored.
    hex_config_t wants KINEMATIC space.  The map between them is per JOINT,
    not per leg.  See sign_map() for why.

3.  FILL THE STRUCT, refusing on anything unspecified rather than defaulting
    it.  The cores ship no hex_config_default() precisely so that a missing
    constant cannot be silently substituted; a filler that supplies one would
    give back the hole the header was designed to close.

Nothing here computes physics.  Physics is core/'s and is not reimplemented.
"""

import ctypes
import math
from pathlib import Path

from sim.constants import ConstantError, load

# ---------------------------------------------------------------------------
# Frozen orders, from hex_config.h.  Not re-derived anywhere else.
# ---------------------------------------------------------------------------

LEGS = ("R1", "R2", "R3", "L1", "L2", "L3")   # enum HEX_R1 = 0 .. HEX_L3 = 5
N_LEGS = 6
N_JOINTS = 12                                  # coxa + femur per leg; tibia is not commanded


def hex_coxa(leg):
    """#define HEX_COXA(leg) (2 * (leg)) — interleaved, coxa first within a leg."""
    return 2 * leg


def hex_femur(leg):
    """#define HEX_FEMUR(leg) (2 * (leg) + 1)."""
    return 2 * leg + 1


# Which envelope group each leg reads, and which side of the vendor mirror it
# sits on.  COREDROP_04 §3: R is the right side and R1 is front-right, from
# D227's coxa table (R1 at x = +102.2010 forward, y = -62.9665 right).
LEG_GROUP = {
    "R1": ("coxa_front",  "tibia_front_middle", "right"),
    "R2": ("coxa_middle", "tibia_front_middle", "right"),
    "R3": ("coxa_rear",   "tibia_rear",         "right"),
    "L1": ("coxa_front",  "tibia_front_middle", "left"),
    "L2": ("coxa_middle", "tibia_front_middle", "left"),
    "L3": ("coxa_rear",   "tibia_rear",         "left"),
}


def sign_map(joint, side):
    """Vendor servo-command space -> kinematic space, for one joint on one side.

    Two facts compose, and the composition is the whole rule.

    The VENDOR mirror is uniform: pwm_R = 3000 - pwm_L on every channel
    (D248, verified across all 394 action groups), so angle_R = -angle_L.

    The KINEMATIC angles are not uniform.  In ik_core's leg frame (+x along
    beta_mount, +y to its left, +z up), for a left-right symmetric body pose:

        theta1  coxa   ANTISYMMETRIC   theta1_R = -theta1_L
        theta2  femur  SYMMETRIC       theta2_R = +theta2_L
        theta3  tibia  SYMMETRIC       theta3_R = +theta3_L

    Compose them: antisymmetric x antisymmetric cancels, symmetric x
    antisymmetric does not.

        coxa    identity on all six legs
        femur   identity on the left three, negated on the right three
        tibia   identity on the left three, negated on the right three

    Applying ONE sign to a whole leg does not lose travel — it reflects that
    leg's coxa window onto the wrong end of its own travel, and every layout
    assertion still passes.  That was MPTASK_05's error, corrected by
    COREDROP_05 §3 and ratified here.

    The OFFSET is a separate quantity, is not assumed zero for femur or tibia,
    and is hardware's under D262.  This function is the sign only.
    """
    if joint == "coxa":
        return 1.0
    if joint not in ("femur", "tibia"):
        raise ValueError("unknown joint %r" % (joint,))
    return 1.0 if side == "left" else -1.0


def apply_sign(lo, hi, s):
    """Map an interval through a sign, keeping it ordered."""
    a, b = s * lo, s * hi
    return (a, b) if a <= b else (b, a)


# ---------------------------------------------------------------------------
# The struct, in hex_config.h declaration order.  Order is load-bearing:
# hex_config_field_name(i) is compared against _fields_[i] positionally.
# ---------------------------------------------------------------------------

F = ctypes.c_float


class HexConfig(ctypes.Structure):
    _fields_ = [
        ("coxa_length_mm", F), ("femur_length_mm", F), ("tibia_length_mm", F),
        ("theta3_deg", F),
        ("coxa_x_mm", F * N_LEGS), ("coxa_y_mm", F * N_LEGS),
        ("beta_mount_deg", F * N_LEGS), ("beta_neutral_deg", F * N_LEGS),
        ("theta2_nom_deg", F), ("stride_mm", F), ("swing_clearance_mm", F),
        ("duty_factor", F), ("swing_peak_factor", F), ("dtheta_peak_deg_s", F),
        ("frame_period_us", F),
        ("stale_ramp_ms", F), ("swing_eps_mm_s", F),
        ("command_step_deg", F), ("joint_accuracy_deg", F),
        ("joint_min_deg", F * N_JOINTS), ("joint_max_deg", F * N_JOINTS),
        ("theta3_min_deg", F), ("theta3_max_deg", F),
        ("mass_kg", F), ("tau_servo_kgcm", F), ("margin_factor", F),
    ]


class HexDerived(ctypes.Structure):
    _fields_ = [("rigid_len_mm", F), ("psi_deg", F), ("theta_nom_deg", F),
                ("a_eff_nom_mm", F), ("body_height_mm", F), ("r_nom_mm", F)]


HEX_CFG_ERR = ("HEX_CFG_OK", "E_MEMBER", "E_DUTY", "E_STRIDE", "E_CLEARANCE",
               "E_RATE", "E_PROFILE", "E_REACH", "E_RAMP", "E_MARGIN",
               "E_RESOLUTION", "E_THETA3", "E_FOLD")


class LayoutError(Exception):
    """The C struct and the ctypes struct disagree. Never continue past this."""


def load_library(path):
    lib = ctypes.CDLL(str(path))
    lib.hex_config_sizeof.restype = ctypes.c_size_t
    lib.hex_config_field_count.restype = ctypes.c_int
    lib.hex_config_field_name.restype = ctypes.c_char_p
    lib.hex_config_field_name.argtypes = [ctypes.c_int]
    for fn in ("hex_config_field_offset", "hex_config_field_size"):
        f = getattr(lib, fn)
        f.restype = ctypes.c_size_t
        f.argtypes = [ctypes.c_int]
    lib.hex_config_validate.restype = ctypes.c_int
    lib.hex_config_validate.argtypes = [ctypes.POINTER(HexConfig)]
    lib.hex_derive.restype = ctypes.c_int
    lib.hex_derive.argtypes = [ctypes.POINTER(HexConfig), ctypes.POINTER(HexDerived)]
    return lib


def assert_layout(lib):
    """Compare (name, offset, size) for every field. Raise on the first disagreement.

    Returns the field count so a caller can report it.
    """
    c_size, py_size = lib.hex_config_sizeof(), ctypes.sizeof(HexConfig)
    if c_size != py_size:
        raise LayoutError("sizeof(hex_config_t) is %d in C and %d in ctypes" % (c_size, py_size))

    c_n, py_n = lib.hex_config_field_count(), len(HexConfig._fields_)
    if c_n != py_n:
        raise LayoutError("field count is %d in C and %d in ctypes" % (c_n, py_n))

    bad = []
    for i in range(c_n):
        c_name = lib.hex_config_field_name(i).decode("ascii")
        c_off = lib.hex_config_field_offset(i)
        c_sz = lib.hex_config_field_size(i)
        py_name = HexConfig._fields_[i][0]
        desc = getattr(HexConfig, py_name)
        if (c_name, c_off, c_sz) != (py_name, desc.offset, desc.size):
            bad.append("  [%2d] C(%s, %d, %d) != ctypes(%s, %d, %d)"
                       % (i, c_name, c_off, c_sz, py_name, desc.offset, desc.size))
    if bad:
        raise LayoutError("layout mismatch on %d field(s):\n%s" % (len(bad), "\n".join(bad)))

    # Out-of-range contract, from hex_config.h: NULL name, (size_t)-1 sizes.
    minus_one = ctypes.c_size_t(-1).value
    for i in (-1, c_n):
        if lib.hex_config_field_name(i) is not None:
            raise LayoutError("field_name(%d) should be NULL" % i)
        if lib.hex_config_field_offset(i) != minus_one or lib.hex_config_field_size(i) != minus_one:
            raise LayoutError("field_offset/size(%d) should be (size_t)-1" % i)
    return c_n


# ---------------------------------------------------------------------------
# Projection
# ---------------------------------------------------------------------------

def project_joint_limits(envelopes):
    """joint_envelopes_deg (vendor servo space) -> two 12-arrays (kinematic space).

    Returns (mins, maxs), each indexed by HEX_COXA(leg) / HEX_FEMUR(leg).
    """
    mins, maxs = [None] * N_JOINTS, [None] * N_JOINTS
    for leg_i, leg in enumerate(LEGS):
        coxa_group, _tibia_group, side = LEG_GROUP[leg]
        for joint, group, idx in (("coxa", coxa_group, hex_coxa(leg_i)),
                                  ("femur", "femur_all", hex_femur(leg_i))):
            lo, hi = envelopes[group][side]
            mins[idx], maxs[idx] = apply_sign(lo, hi, sign_map(joint, side))
    if any(v is None for v in mins + maxs):
        raise ConstantError("joint limit projection left a hole")
    for i in range(N_JOINTS):
        if not mins[i] < maxs[i]:
            # hex_config_validate does NOT check this: D258 authorised one new
            # error code, for the tibia pair only, and inventing a thirteenth
            # here would be the over-reach D258 was filed against. The check
            # therefore lives on this side (COREDROP_04 §9b).
            raise ConstantError("joint %d has min %.4f >= max %.4f" % (i, mins[i], maxs[i]))
    return mins, maxs


def project_theta3_envelope(envelopes):
    """The tibia pair: ONE theta3 serves all six legs (D221), so INTERSECT.

    A value inside the union but outside one leg's row is a value two legs
    cannot reach while hex_config_validate still reports HEX_CFG_OK.
    Intersect AFTER the sign map, never before: intersecting the raw table
    intersects a range with its own reflection and returns only the symmetric
    part of it.
    """
    los, his = [], []
    for leg in LEGS:
        _coxa_group, tibia_group, side = LEG_GROUP[leg]
        lo, hi = apply_sign(*envelopes[tibia_group][side], s=sign_map("tibia", side))
        los.append(lo)
        his.append(hi)
    t_min, t_max = max(los), min(his)
    if not t_min < t_max:
        raise ConstantError(
            "the six tibia envelopes have an empty intersection: [%.4f, %.4f]. "
            "That is a finding about the machine, not a config typo - file it."
            % (t_min, t_max))
    # COREDROP_04 §4.5: psi has a 360 deg branch cut at theta3 == 180 (mod 360),
    # and no ULP bound covers a 360 deg step. The core does not check this.
    for edge in (-180.0, 180.0):
        if t_min <= edge <= t_max:
            raise ConstantError(
                "the tibia envelope [%.4f, %.4f] contains %.1f deg, where psi is "
                "discontinuous. Refusing rather than straddling the branch cut."
                % (t_min, t_max, edge))
    return t_min, t_max


# ---------------------------------------------------------------------------
# Filler
# ---------------------------------------------------------------------------

SCALARS = ("coxa_length_mm", "femur_length_mm", "tibia_length_mm", "theta3_deg",
           "theta2_nom_deg", "stride_mm", "swing_clearance_mm",
           "duty_factor", "dtheta_peak_deg_s",
           "stale_ramp_ms", "swing_eps_mm_s",
           "command_step_deg", "joint_accuracy_deg",
           "mass_kg", "tau_servo_kgcm", "margin_factor")

# swing_peak_factor is peak joint rate divided by mean over one swing. It is a
# property of the velocity PROFILE, so it is computed from the profile's name
# and never stored. pi/2 was one of the three derived quantities caught being
# written into the table; storing it again would put the same defect back.
PEAK_OVER_MEAN = {
    # v(t) = A*sin(pi*t/T): mean = (2/pi)*A, peak = A, so peak/mean = pi/2.
    "half_sine": math.pi / 2.0,
}


# Fields that gait_core needs at init and that nothing in MP1 reads before it
# exists. They are unspecified today and are due from the algorithm workstream
# with gait_core on 30 September (COREDROP_03 §7).
DEFERRABLE = ("stale_ramp_ms", "swing_eps_mm_s")


def fill(constants=None, strict=True):
    """Build hex_config_t from the constant table.

    strict=True  — refuse on ANY unspecified field. This is what a real init
                   must use: the cores ship no hex_config_default(), so a
                   filler that substituted a value would give back the hole
                   the header exists to close.

    strict=False — the same, except that the DEFERRABLE fields are written as
                   NaN and their names returned. NaN is not a default and is
                   not a guess: it is unequal to everything including itself,
                   it propagates through any arithmetic that touches it, and
                   hex_config_validate rejects it on sight, because every
                   guard there is written as !(x > y) and that is TRUE for
                   NaN. So a deferred config passes hex_derive and ik_fk_leg,
                   which never read those fields, and fails
                   hex_config_validate loudly with the code that names the
                   missing field. A zero would have passed silently.

    Returns (cfg, surrogate_names, deferred_names). The surrogate list is not
    decoration: an output derived from a surrogate is not a measurement and
    every emitted table must be stamped with the names that fed it.
    """
    c = constants if constants is not None else load()
    cfg = HexConfig()
    deferred = []

    for name in SCALARS:
        if not strict and name in DEFERRABLE and c.status(name) == "unspecified":
            setattr(cfg, name, float("nan"))
            deferred.append(name)
            continue
        setattr(cfg, name, float(c.value(name)))     # raises if unspecified

    # Two struct fields have no key and are derived here, never stored, so that
    # nobody adds a second source of truth for the same number.
    cfg.frame_period_us = 1.0e6 / float(c.value("update_rate_hz"))

    profile = c.value("swing_velocity_profile")
    if profile not in PEAK_OVER_MEAN:
        raise ConstantError(
            "swing_velocity_profile is %r; no peak-over-mean factor is defined for it. "
            "Add the derivation, do not store the number." % (profile,))
    cfg.swing_peak_factor = PEAK_OVER_MEAN[profile]

    pos = c.value("coxa_positions_mm")
    bm = c.value("beta_mount_deg")
    bn = c.value("beta_neutral_deg")
    for i, leg in enumerate(LEGS):
        cfg.coxa_x_mm[i], cfg.coxa_y_mm[i] = (float(v) for v in pos[leg])
        cfg.beta_mount_deg[i] = float(bm[leg])
        cfg.beta_neutral_deg[i] = float(bn[leg])

    env = c.value("joint_envelopes_deg")
    mins, maxs = project_joint_limits(env)
    for i in range(N_JOINTS):
        cfg.joint_min_deg[i] = mins[i]
        cfg.joint_max_deg[i] = maxs[i]
    cfg.theta3_min_deg, cfg.theta3_max_deg = project_theta3_envelope(env)

    # D358: a FOUR-tuple. The disputed reads travel beside the surrogate reads
    # for the same reason -- a value that hands itself out silently defeats the
    # label. The signature was changed while nothing depended on the new
    # element, which is the only time it is cheap; the moment mass_kg or
    # theta2_nom_deg becomes disputed it would have been done under pressure on
    # a live constant. D358's override did not fire: no caller exists outside
    # hexapod-kinematics, checked by grep before the signature moved.
    #
    # NOT DORMANT, contrary to what FINDING_12 section 3 asserted and D358
    # recites. dtheta_peak_deg_s IS a hex_config_t field -- HEX_FIELD is on it
    # in hex_config.c -- and D324 clause 2 had already made it disputed. fill()
    # was writing a disputed constant into the struct and reporting only the
    # surrogates. Corrected to coordination by FINDING_13 section 7.1.
    return cfg, c.surrogates_read(), c.disputed_read(), deferred


# ---------------------------------------------------------------------------
# Agreement
# ---------------------------------------------------------------------------

K_ULP = 32          # COREDROP_04 §4.4. A count, not a tolerance: no unit, and
                    # it tracks both magnitude and storage type.

SCALE_FIELD = {"rigid_len_mm": "rigid_len_mm", "a_eff_nom_mm": "rigid_len_mm",
               "body_height_mm": "rigid_len_mm", "r_nom_mm": "rigid_len_mm",
               "psi_deg": 180.0, "theta_nom_deg": 180.0}


def ulp32(x):
    """Distance to the next float32 above |x|."""
    x = abs(ctypes.c_float(x).value)
    if x == 0.0:
        return math.ldexp(1.0, -149)                 # smallest subnormal
    _, e = math.frexp(x)
    return math.ldexp(1.0, max(e - 24, -149))


def agree(a, b, field, derived):
    """Is a float result and a double result the same number, as closely as float allows?

    NOT a comparison at N decimal places. Rounding two precisions to a shared
    decimal grid compares which side of a grid line each landed on; the
    disagreement rate falls as places are dropped and never reaches zero
    (5.58 % at 4 dp, 0.06 % at 2 dp, on a 60,001-sample theta3 sweep).
    """
    s = SCALE_FIELD[field]
    scale = derived.rigid_len_mm if isinstance(s, str) else s
    return abs(a - b) <= K_ULP * ulp32(max(abs(a), abs(b), abs(scale)))


def describe(err):
    return HEX_CFG_ERR[err] if 0 <= err < len(HEX_CFG_ERR) else "UNKNOWN(%d)" % err
