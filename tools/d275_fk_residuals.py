"""D275 - the vendor pose set validated at 2-DOF, theta3 fed per pose.

WHAT THIS TESTS. Table 1 tests ik_core's GEOMETRY, not a shipped configuration.
Every one of the 394 vendor poses is fed its own theta3, twelve distinct values
spanning [-67.5000, +135.0000], and most of them are outside any envelope this
project would ship. hex_derive is called directly and hex_config_validate is NOT,
because validate would reject configurations the geometry is nonetheless required
to be correct on.

Table 2 DOES test a shipped configuration: the 284 uniform-tibia poses at the one
fixed theta3 the v1 config carries, through hex_config_validate first.

THE RESIDUAL is a round trip, ik_fk_leg -> ik_solve_leg:

    (theta1, theta2)  --FK-->  (x, y, z)  --IK-->  (theta1', theta2')
    residual = max(|theta1' - theta1|, |theta2' - theta2|)   in degrees

The command OFFSET (D262, Spider Hardware's) is unmodelled and does not need to be
modelled: it is the same constant on the way in and on the way out, so it cancels in
a round trip. What does not cancel, and what this measures, is R and psi -- the
two-segment reduction the whole 2-DOF treatment rests on.

The cores are single precision (float, cosf/sinf/atan2f), so the floor here is
float32 and not float64. Residuals are therefore reported against the 0.1350 deg
command grid, which is the only scale at which they mean anything physical.

EXCLUDING THE 110 NON-UNIFORM POSES IS REFUSED. They are not a random subset: they
are the widest joint combinations in the corpus, which is exactly where a geometry
defect would show. Table 1 carries all 394.
"""
from __future__ import annotations

import ctypes
import json
import math
import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from bindings.hexconfig import HexConfig, HexDerived, load_library  # noqa: E402
from tools.vendor_poses import read_actions  # noqa: E402

# --------------------------------------------------------------- the predicate
#
# D275's predicate, verbatim in effect: the tibia angle of a RIGHT leg is
# (1500 - pwm) * 0.1350 and of a LEFT leg (pwm - 1500) * 0.1350, so both sides end
# up in one leg-local convention.
#
# It generalises to the other two joints through hex_config.h's per-JOINT sign map,
# and the generalisation is not uniform. The vendor mirrors the sides as
# pwm_R = 3000 - pwm_L, so angle_R = -angle_L. theta1 is antisymmetric between the
# sides and theta2/theta3 are symmetric, so the vendor's mirror CANCELS for the coxa
# and does not cancel for the other two:
#
#     coxa  theta1 : identity on all six legs
#     femur theta2 : identity on the left three, negated on the right three
#     tibia theta3 : identity on the left three, negated on the right three
#
# Applying one sign to a whole leg silently reflects its coxa window.

STEP_DEG = 0.1350
CENTRE = 1500

# RULE_MP_WS section 3. The low block is LEFT and runs tibia, femur, coxa; the high
# block is RIGHT and runs coxa, femur, tibia. Both are the opposite of the natural
# reading, and channel r faces 23 - r.
LEG_CHANNELS = {
    "L3": {"coxa": 2, "femur": 1, "tibia": 0},
    "L2": {"coxa": 5, "femur": 4, "tibia": 3},
    "L1": {"coxa": 8, "femur": 7, "tibia": 6},
    "R1": {"coxa": 15, "femur": 16, "tibia": 17},
    "R2": {"coxa": 18, "femur": 19, "tibia": 20},
    "R3": {"coxa": 21, "femur": 22, "tibia": 23},
}
LEG_ORDER = ("R1", "R2", "R3", "L1", "L2", "L3")   # frozen, hex_config.h
RIGHT_LEGS = {"R1", "R2", "R3"}


def angle_deg(pwm: int, leg: str, joint: str) -> float:
    """One vendor channel value -> one kinematic angle, per the sign map above."""
    if joint == "coxa" or leg not in RIGHT_LEGS:
        return (pwm - CENTRE) * STEP_DEG
    return (CENTRE - pwm) * STEP_DEG


# ------------------------------------------------------------------ the library

IK_OK = 0
IK_PROJ_NONE = 0
IK_STATUS = {0: "IK_OK", 1: "E_UNREACHABLE_FAR", 2: "E_UNREACHABLE_NEAR", 3: "E_LIMIT"}
CFG_ERR = {
    0: "HEX_CFG_OK", 1: "E_MEMBER", 2: "E_DUTY", 3: "E_STRIDE", 4: "E_CLEARANCE",
    5: "E_RATE", 6: "E_PROFILE", 7: "E_REACH", 8: "E_RAMP", 9: "E_MARGIN",
    10: "E_RESOLUTION", 11: "E_THETA3",
}

CFLAGS = ["-std=c99", "-Wall", "-Wextra", "-pedantic", "-O2", "-fPIC", "-shared",
          "-ffp-contract=off"]


def build(out: pathlib.Path) -> pathlib.Path:
    """Build on MP1's own flags. -ffp-contract=off matters: a fused multiply-add
    would change the last bits of R and psi and move every residual here."""
    cmd = (["gcc"] + CFLAGS + ["-I", str(ROOT / "core/include"),
           str(ROOT / "core/src/hex_config.c"), str(ROOT / "core/src/ik_core.c"),
           "-o", str(out), "-lm"])
    subprocess.run(cmd, check=True)
    return out


def bind(lib):
    F, PF = ctypes.c_float, ctypes.POINTER(ctypes.c_float)
    lib.ik_fk_leg.restype = None
    lib.ik_fk_leg.argtypes = [ctypes.POINTER(HexConfig), ctypes.POINTER(HexDerived),
                              F, F, PF, PF, PF]
    lib.ik_solve_leg.restype = ctypes.c_int
    lib.ik_solve_leg.argtypes = [ctypes.POINTER(HexConfig), ctypes.POINTER(HexDerived),
                                 F, F, F, ctypes.c_int, PF, PF]
    return lib


def geometry_config(k, theta3_deg: float) -> HexConfig:
    """A config carrying ONLY what hex_derive reads: the three member lengths and
    theta3. Everything else is left at zero deliberately. hex_derive does not touch
    the rest, and a caller that filled them would be asserting a posture this table
    is not about."""
    c = HexConfig()
    c.coxa_length_mm = k.value("coxa_length_mm")
    c.femur_length_mm = k.value("femur_length_mm")
    c.tibia_length_mm = k.value("tibia_length_mm")
    c.theta3_deg = theta3_deg
    return c


def signed_reach_mm(cfg, der, t2: float) -> float:
    """r = L1 + R*cos(theta2 + psi), WITH ITS SIGN.

    ik_solve_leg recovers this as sqrt(x^2 + y^2), which is |r|. When the foot
    folds back past the coxa axis the sign is destroyed and the azimuth flips by
    180 deg, so the forward map stops being injective and the inverse correctly
    refuses. This column exists so that refusal is explicable from the table."""
    th = math.radians(t2 + der.psi_deg)
    return cfg.coxa_length_mm + der.rigid_len_mm * math.cos(th)


def round_trip(lib, cfg, der, t1: float, t2: float):
    """FK then IK. Returns (status, d_theta1, d_theta2, foot xyz)."""
    x, y, z = (ctypes.c_float() for _ in range(3))
    lib.ik_fk_leg(ctypes.byref(cfg), ctypes.byref(der), t1, t2,
                  ctypes.byref(x), ctypes.byref(y), ctypes.byref(z))
    o1, o2 = ctypes.c_float(), ctypes.c_float()
    st = lib.ik_solve_leg(ctypes.byref(cfg), ctypes.byref(der),
                          x.value, y.value, z.value, IK_PROJ_NONE,
                          ctypes.byref(o1), ctypes.byref(o2))
    if st != IK_OK:
        return st, None, None, (x.value, y.value, z.value)
    return st, o1.value - t1, o2.value - t2, (x.value, y.value, z.value)


# ----------------------------------------------------------------------- tables

def pose_angles(pwm: dict) -> dict:
    """One vendor frame -> {leg: (theta1, theta2, theta3)} in kinematic degrees."""
    out = {}
    for leg, ch in LEG_CHANNELS.items():
        out[leg] = tuple(angle_deg(pwm[ch[j]], leg, j)
                         for j in ("coxa", "femur", "tibia"))
    return out


def table1(lib, k, frames):
    """All 394 action poses, theta3 fed PER POSE. Geometry only."""
    rows = []
    for gid, pwm, _ms in frames:
        ang = pose_angles(pwm)
        for leg in LEG_ORDER:
            t1, t2, t3 = ang[leg]
            cfg = geometry_config(k, t3)
            der = HexDerived()
            e = lib.hex_derive(ctypes.byref(cfg), ctypes.byref(der))
            if e != 0:
                rows.append((gid, leg, t1, t2, t3, None, None, None, CFG_ERR[e]))
                continue
            st, d1, d2, xyz = round_trip(lib, cfg, der, t1, t2)
            rows.append((gid, leg, t1, t2, t3, d1, d2, xyz, IK_STATUS.get(st, st),
                         signed_reach_mm(cfg, der, t2)))
    return rows


def table2(lib, k, frames, theta3_fixed: float):
    """The 284 uniform-tibia poses at ONE fixed theta3. A shipped configuration."""
    rows = []
    cfg = geometry_config(k, theta3_fixed)
    der = HexDerived()
    e = lib.hex_derive(ctypes.byref(cfg), ctypes.byref(der))
    if e != 0:
        raise SystemExit("hex_derive rejected the fixed theta3: " + CFG_ERR[e])
    for gid, pwm, _ms in frames:
        ang = pose_angles(pwm)
        for leg in LEG_ORDER:
            t1, t2, _t3 = ang[leg]
            st, d1, d2, xyz = round_trip(lib, cfg, der, t1, t2)
            rows.append((gid, leg, t1, t2, theta3_fixed, d1, d2, xyz,
                         IK_STATUS.get(st, st), signed_reach_mm(cfg, der, t2)))
    return rows, der


def summarise(rows, label):
    ok = [r for r in rows if r[8] == "IK_OK"]
    bad = [r for r in rows if r[8] != "IK_OK"]
    res = [max(abs(r[5]), abs(r[6])) for r in ok]
    worst = max(ok, key=lambda r: max(abs(r[5]), abs(r[6]))) if ok else None
    print(f"\n--- {label} ---")
    print(f"  rows                  {len(rows)}   ({len(rows)//6} poses x 6 legs)")
    print(f"  IK_OK                 {len(ok)}")
    print(f"  not IK_OK             {len(bad)}")
    if res:
        print(f"  max |d theta|         {max(res):.6e} deg")
        print(f"  mean |d theta|        {sum(res)/len(res):.6e} deg")
        print(f"  max as command steps  {max(res)/STEP_DEG:.6e}")
        print(f"  worst pose            G{worst[0]:04d} {worst[1]}  "
              f"theta3={worst[4]:.4f}  d1={worst[5]:+.3e} d2={worst[6]:+.3e}")
    if bad:
        neg = [r for r in bad if r[9] < 0.0]
        pos_ok = [r for r in ok if r[9] < 0.0]
        print(f"  of the not-OK rows, r < 0   {len(neg)} of {len(bad)}")
        print(f"  of the OK rows,     r < 0   {len(pos_ok)} of {len(ok)}")
        poses_bad = len({r[0] for r in bad})
        print(f"  poses with >=1 not-OK leg   {poses_bad}")
        print(f"  poses with all six OK       {len({r[0] for r in rows}) - poses_bad}")
    return {"rows": len(rows), "ok": len(ok), "bad": len(bad),
            "max_deg": max(res) if res else None,
            "mean_deg": sum(res)/len(res) if res else None}


def write_csv(rows, path):
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write("frame,leg,theta1_deg,theta2_deg,theta3_deg,"
                "d_theta1_deg,d_theta2_deg,residual_deg,residual_steps,"
                "foot_x_mm,foot_y_mm,foot_z_mm,signed_reach_mm,status\n")
        for gid, leg, t1, t2, t3, d1, d2, xyz, st, rs in rows:
            if d1 is None:
                f.write(f"G{gid:04d},{leg},{t1:.4f},{t2:.4f},{t3:.4f},,,,,"
                        f"{xyz[0]:.4f},{xyz[1]:.4f},{xyz[2]:.4f},{rs:.4f},{st}\n")
                continue
            r = max(abs(d1), abs(d2))
            f.write(f"G{gid:04d},{leg},{t1:.4f},{t2:.4f},{t3:.4f},"
                    f"{d1:.6e},{d2:.6e},{r:.6e},{r/STEP_DEG:.6e},"
                    f"{xyz[0]:.4f},{xyz[1]:.4f},{xyz[2]:.4f},{rs:.4f},{st}\n")


def main():
    from sim import constants as C
    k = C.load()
    lib = bind(load_library(str(build(pathlib.Path("/tmp/libhex_d275.so")))))

    frames, digest = read_actions(str(ROOT.parent / "Lm2六足机器人动作组V3.ini")) \
        if False else read_actions(sys.argv[1])
    action = frames[1:]                     # D302: frame 0 is the home frame

    print(f"pose file sha256 {digest}")
    print(f"action poses     {len(action)}  (group_liu_zu, frame 0 excluded)")
    print(f"geometry         L1={k.value('coxa_length_mm'):.4f} "
          f"L2={k.value('femur_length_mm'):.4f} "
          f"L3={k.value('tibia_length_mm'):.4f}  (all measured)")

    uniform, nonuniform = [], []
    for fr in action:
        t3 = {round(angle_deg(fr[1][c["tibia"]], lg, "tibia"), 4)
              for lg, c in LEG_CHANNELS.items()}
        (uniform if len(t3) == 1 else nonuniform).append(fr)
    print(f"uniform tibia    {len(uniform)} of {len(action)} = "
          f"{100*len(uniform)/len(action):.4f} %")
    print(f"non-uniform      {len(nonuniform)} of {len(action)} = "
          f"{100*len(nonuniform)/len(action):.4f} %   NOT excluded from table 1")

    r1 = table1(lib, k, action)
    s1 = summarise(r1, "TABLE 1  all 394 poses, theta3 per pose, geometry only")

    t3_fixed = k.value("theta3_deg")
    r2, der = table2(lib, k, uniform, t3_fixed)
    print(f"\n  fixed theta3 = {t3_fixed:.4f} ({k.status('theta3_deg')}), "
          f"R={der.rigid_len_mm:.4f} psi={der.psi_deg:.4f}")
    s2 = summarise(r2, f"TABLE 2  284 uniform poses at fixed theta3={t3_fixed:.4f}")

    out = ROOT / "reports"
    out.mkdir(exist_ok=True)
    write_csv(r1, out / "d275_table1_per_pose_theta3.csv")
    write_csv(r2, out / "d275_table2_fixed_theta3.csv")
    print(f"\nwrote {out/'d275_table1_per_pose_theta3.csv'}")
    print(f"wrote {out/'d275_table2_fixed_theta3.csv'}")
    return s1, s2


if __name__ == "__main__":
    main()
