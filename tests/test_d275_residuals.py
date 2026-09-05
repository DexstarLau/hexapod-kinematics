"""D275: the vendor pose set validated at 2-DOF, theta3 fed per pose.

These pin the four figures D275 publishes and the two the run produces. They are
regression tests on the PREDICATE as much as on the geometry: if the channel map or
the per-joint sign map is ever changed, the twelve distinct tibia angles and the
284/110 split move, and these go red before any residual does.
"""
import math
import pathlib

import pytest

from tools.d275_fk_residuals import LEG_CHANNELS, angle_deg
from tools.vendor_poses import read_actions

POSE_FILE = pathlib.Path("/mnt/project/Lm2六足机器人动作组V3.ini")
pytestmark = pytest.mark.skipif(not POSE_FILE.exists(),
                                reason="vendor pose file is not redistributed (THIRD_PARTY.md)")


@pytest.fixture(scope="module")
def action():
    frames, digest = read_actions(str(POSE_FILE))
    assert digest == "2d1e4365806d34aec696064731d3ac38af75fcc98102c3dc5a1a38f1c459ecc8"
    return frames[1:]                      # D302: frame 0 is the all-1500 home frame


def tibiae(frame):
    _gid, pwm, _ms = frame
    return {lg: round(angle_deg(pwm[c["tibia"]], lg, "tibia"), 4)
            for lg, c in LEG_CHANNELS.items()}


def test_the_corpus_is_394_action_poses(action):
    assert len(action) == 394


def test_twelve_distinct_tibia_angles(action):
    """D275. Twelve, not eleven and not thirteen -- the count is what makes the
    fixed-theta3 reduction quantifiable."""
    assert len({v for f in action for v in tibiae(f).values()}) == 12


def test_the_uniform_split_is_284_and_110(action):
    """D275. Excluding the 110 is refused: they are the widest joint combinations."""
    uniform = [f for f in action if len(set(tibiae(f).values())) == 1]
    assert len(uniform) == 284
    assert len(action) - len(uniform) == 110
    assert round(100 * len(uniform) / len(action), 4) == 72.0812


def test_the_six_leg_intersection_and_its_single_outlier(action):
    """D275. [-67.5000, +121.5000], and G0117's rear pair at 135.0000 is the only
    pose outside it. theta3_min_deg / theta3_max_deg are NOT set from this -- they
    are quarantined under D260 and this is an upper bound on any fixed choice."""
    per = {lg: (min(tibiae(f)[lg] for f in action), max(tibiae(f)[lg] for f in action))
           for lg in LEG_CHANNELS}
    lo = max(v[0] for v in per.values())
    hi = min(v[1] for v in per.values())
    assert (lo, hi) == (-67.5, 121.5)
    outside = {f[0] for f in action if any(not lo <= v <= hi for v in tibiae(f).values())}
    assert outside == {117}


def test_reach_sign_partitions_the_solvable_rows_exactly(action):
    """THE FINDING, rewritten as a property. D359 clause 1.

    Before D342 this pinned three integers out of the tool's summary:
    rows 2364, ok 1298, bad 1066. D342 clause 3 changed what "bad" MEANS on
    those 1,066 rows -- IK_PROJ_NONE now returns IK_W_REFLECTED with the exact
    pose written, instead of IK_E_UNREACHABLE_NEAR with nothing written. The
    old assertion would have kept passing and kept saying something false,
    because the tool classifies by `status == "IK_OK"` and an unknown code
    falls through `IK_STATUS.get(st, st)` without raising.

    So the counts survive, attached to the status they now mean, and the thing
    actually pinned is the equivalence in both directions:

        r = L1 + R*cos(theta2 + psi) < 0   <=>   IK_W_REFLECTED
        r > 0                             <=>   IK_OK

    1,066 of 1,066 and 0 of 1,298. That exactness is what makes the fold a
    property of the map and not a numerical accident, and it is the only
    committed evidence for it.

    This test does its own round trip rather than calling main(). round_trip()
    in the tool returns (st, None, None, xyz) for any non-IK_OK status, so it
    throws away exactly the angles D342 exists to produce. The tool is Mini
    Project's and is not edited from here.
    """
    import ctypes
    import pathlib

    from bindings.hexconfig import HexDerived
    from tools.d275_fk_residuals import (LEG_ORDER, bind, build, geometry_config,
                                         load_library, signed_reach_mm)
    from sim import constants as C

    IK_OK, IK_W_REFLECTED = 0, 4
    IK_PROJ_NONE, IK_PROJ_RADIAL, IK_PROJ_VERTICAL = 0, 1, 2

    lib = bind(load_library(str(build(pathlib.Path("/tmp/libhex_d275_prop.so")))))
    k = C.load()

    def solve(cfg, der, t1, t2, mode):
        x, y, z = (ctypes.c_float() for _ in range(3))
        lib.ik_fk_leg(ctypes.byref(cfg), ctypes.byref(der), t1, t2,
                      ctypes.byref(x), ctypes.byref(y), ctypes.byref(z))
        o1, o2 = ctypes.c_float(), ctypes.c_float()
        st = lib.ik_solve_leg(ctypes.byref(cfg), ctypes.byref(der),
                              x.value, y.value, z.value, mode,
                              ctypes.byref(o1), ctypes.byref(o2))
        return st, o1.value, o2.value

    def wrap(d):
        """D360: both angles are determined only modulo 360, and the core
        returns them normalised into (-180, +180]. A residual measured without
        this reports a full turn as an error of 360 degrees."""
        while d > 180.0:
            d -= 360.0
        while d <= -180.0:
            d += 360.0
        return d

    n_ok = n_reflected = n_err = 0
    worst_ok = worst_reflected = 0.0

    for _gid, pwm, _ms in action:
        for leg in LEG_ORDER:
            ch = LEG_CHANNELS[leg]
            t1 = angle_deg(pwm[ch["coxa"]], leg, "coxa")
            t2 = angle_deg(pwm[ch["femur"]], leg, "femur")
            t3 = angle_deg(pwm[ch["tibia"]], leg, "tibia")

            cfg = geometry_config(k, t3)
            der = HexDerived()
            assert lib.hex_derive(ctypes.byref(cfg), ctypes.byref(der)) == 0
            r = signed_reach_mm(cfg, der, t2)

            st, o1, o2 = solve(cfg, der, t1, t2, IK_PROJ_NONE)
            residual = max(abs(wrap(o1 - t1)), abs(wrap(o2 - t2)))

            if r < 0.0:
                assert st == IK_W_REFLECTED, (
                    "r = %.4f is folded but the solver returned %d" % (r, st))
                n_reflected += 1
                worst_reflected = max(worst_reflected, residual)
                # D342 clause 2: the reflected branch is tested BEFORE any
                # projection, so all three modes give the same exact answer.
                # D359 section 3: IK_PROJ_RADIAL / IK_PROJ_VERTICAL had zero
                # coverage in the whole suite before this line.
                for mode in (IK_PROJ_RADIAL, IK_PROJ_VERTICAL):
                    m_st, m1, m2 = solve(cfg, der, t1, t2, mode)
                    assert m_st == IK_W_REFLECTED
                    assert max(abs(wrap(m1 - t1)), abs(wrap(m2 - t2))) < 1e-4
            elif r > 0.0:
                assert st == IK_OK, (
                    "r = %.4f is on the near side but the solver returned %d" % (r, st))
                n_ok += 1
                worst_ok = max(worst_ok, residual)
            else:
                n_err += 1

    assert (n_ok, n_reflected, n_err) == (1298, 1066, 0)
    assert n_ok + n_reflected == 2364
    # float32 floor, three orders below the 0.1350 command grid. The reflected
    # figure is D342 section 3's 1.525879e-05, reproduced here under cosf/sinf.
    assert worst_ok < 1e-5
    assert worst_reflected < 1e-4


def test_the_shipped_configuration_has_no_folded_row():
    """Table 2's figures are unaffected by D342 and are kept rather than lost
    with the rewrite above. At the shipped theta3_deg = -30.0000 the fold
    boundary sits at theta2 = 121.5921 and no vendor pose reaches it, so all
    1,704 rows solve as IK_OK and none is reflected. This is the test that
    would go red if the shipped theta3 ever moved past the boundary."""
    from tools.d275_fk_residuals import main
    import sys
    argv = sys.argv
    sys.argv = ["d275", str(POSE_FILE)]
    try:
        _s1, s2 = main()
    finally:
        sys.argv = argv
    assert s2["rows"] == 1704 and s2["bad"] == 0
    assert s2["max_deg"] < 1e-5
