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


def test_reach_sign_partitions_the_solvable_rows_exactly():
    """The finding. ik_solve_leg recovers the radial reach as sqrt(x^2 + y^2), which
    is |r|. Where r = L1 + R*cos(theta2 + psi) is negative the foot has folded past
    the coxa axis, the azimuth flips 180 deg, the forward map stops being injective
    and the solve correctly refuses. The partition is exact in both directions --
    that is what makes it a property and not a coincidence."""
    from tools.d275_fk_residuals import main
    import sys
    argv = sys.argv
    sys.argv = ["d275", str(POSE_FILE)]
    try:
        s1, s2 = main()
    finally:
        sys.argv = argv
    assert s1["rows"] == 2364 and s1["ok"] == 1298 and s1["bad"] == 1066
    assert s2["rows"] == 1704 and s2["bad"] == 0
    # float32 floor, three orders below the 0.1350 command grid
    assert s1["max_deg"] < 1e-5
    assert s2["max_deg"] < 1e-5
