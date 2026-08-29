"""Tests for tools/vendor_poses.py.

THE VENDOR FILE IS NOT IN THE REPOSITORY and CI does not have it. A test that
skipped when it is absent would be CI green over a tool that never ran, which
is the exact failure this project's 2 OS x 2 Python matrix exists to prevent.

So these tests do not need it. Every function under test is pure, so the frames
are synthesised here, from this project's own constants, with the defects
deliberately built in. Each test names the defect it was written for; each of
them was a real defect in this file, not a hypothetical one.

The one thing that does need the real file is the committed report, and that is
covered by test_report_matches_current_config, which fails if the constants
move and the report is not regenerated.
"""

import hashlib
import json
import re
from pathlib import Path

import pytest

from sim.constants import load
from tools.vendor_poses import (
    CHANNEL_MAP, N_FRAMES_EXPECTED, analyse, check_envelopes, check_mirror_against_map,
    check_structure, derive_mirror_pairs, envelope_for, raw_range, sign_for,
    text_sha256, to_angles,
)

ROOT = Path(__file__).resolve().parent.parent
REPORT = ROOT / "docs" / "vendor_pose_check.md"


@pytest.fixture(scope="module")
def c():
    return load()


# --------------------------------------------------------------------------
# Synthetic frames
# --------------------------------------------------------------------------

def _counts_for(leg, joint, frac, c):
    """The count that puts one channel at `frac` of the way across its envelope."""
    step = float(c["command_step_deg"])
    centre = int(c["pwm_centre_count"])
    _key, lo, hi = envelope_for(leg, joint, c["joint_envelopes_deg"])
    return centre + round((lo + frac * (hi - lo)) / step)


def synth(c, n=N_FRAMES_EXPECTED, frac=lambda i, ch: 0.5):
    """A well-formed action group: n frames, 32 channels, nothing out of range.

    Only the LEFT block is placed from `frac`. Each right channel is then set
    to 2*centre - its partner, which is the vendor mirror written exactly.
    Placing both sides independently rounds each to the command grid on its
    own, and the two roundings disagree by one count often enough that the
    mirror comes out one step off -- 0.1350 deg of fixture error, arriving in
    a test whose subject is a sign, not a rounding.
    """
    centre = int(c["pwm_centre_count"])
    out = []
    for i in range(n):
        pwm = {ch: centre for ch in range(32)}
        for ch, (leg, joint) in CHANNEL_MAP.items():
            if ch < 9:
                pwm[ch] = _counts_for(leg, joint, frac(i, ch), c)
        for ch in range(15, 24):
            pwm[ch] = 2 * centre - pwm[23 - ch]
        out.append((i, pwm, {ch: 300 for ch in range(32)}))
    return out


def sweep(c):
    """Frames that visit both ends of every envelope, mirrored left to right.

    Fraction f of a left envelope and fraction 1-f of the matching right one
    are the same physical pose: the two rows are reflections, so
    lo + f*(hi-lo) on the left negates to the 1-f point on the right. Driving
    them that way makes the raw counts come out mirrored on their own rather
    than being written in mirrored, which is what the pairing test needs.

    THE THREE LEG POSITIONS ARE GIVEN DIFFERENT PHASES. Move all six legs
    together and every same-joint pairing scores identically, so the pairing is
    not identifiable at all -- the frames, not the code, would be what failed.
    """
    phase = {"1": 0.0, "2": 0.31, "3": 0.67}

    def frac(i, ch):
        leg, _joint = CHANNEL_MAP[ch]
        # Frames 0 and 1 pin both endpoints on EVERY channel. A phased sweep
        # alone never lands exactly on them except for the zero-phase leg, and
        # check_envelopes compares endpoints, so the fixture would fail for a
        # reason that has nothing to do with the code under test.
        if i < 2:
            f = float(i)
        else:
            f = (((i - 2) / (N_FRAMES_EXPECTED - 3)) + phase[leg[1]]) % 1.0
        return f

    return synth(c, frac=frac)


# --------------------------------------------------------------------------
# Structure
# --------------------------------------------------------------------------

def test_structure_accepts_a_well_formed_set(c):
    assert check_structure(synth(c), c) == []


def test_structure_rejects_a_short_set(c):
    problems = check_structure(synth(c, n=100), c)
    assert any("expected %d frames" % N_FRAMES_EXPECTED in p for p in problems)


def test_structure_rejects_a_count_outside_the_bus_range(c):
    frames = synth(c)
    frames[7][1][0] = int(c["pwm_count_max"]) + 1
    assert any("outside" in p for p in check_structure(frames, c))


def test_structure_rejects_an_unpopulated_channel_that_moved(c):
    """#024-#031 and #009, #013 hold centre. If one moves, the map has shifted."""
    frames = synth(c)
    frames[3][1][27] = 1600
    assert any("#027" in p for p in check_structure(frames, c))


# --------------------------------------------------------------------------
# Labelling -- the defect that produced two wrong channel maps
# --------------------------------------------------------------------------

def test_envelopes_accept_the_declared_labelling(c):
    assert check_envelopes(sweep(c), c) == []


def test_envelopes_reject_a_reversed_joint_triple(c):
    """Read a right triple as coxa, femur, tibia when it is wired the other way.

    This was the first version of CHANNEL_MAP. Every mirror pair still derived,
    because reversing a triple on both sides leaves the pairing untouched.
    """
    frames = sweep(c)
    swapped = dict(CHANNEL_MAP)
    for a, b in ((0, 2), (3, 5), (6, 8), (15, 17), (18, 20), (21, 23)):
        swapped[a], swapped[b] = CHANNEL_MAP[b], CHANNEL_MAP[a]
    import tools.vendor_poses as vp
    original = vp.CHANNEL_MAP
    try:
        vp.CHANNEL_MAP = swapped
        problems = vp.check_envelopes(frames, c)
    finally:
        vp.CHANNEL_MAP = original
    assert problems, "reversing every triple must be caught by travel"


def test_envelopes_reject_exchanged_sides(c):
    """The second wrong map: the low block read as the right side.

    16 of 18 channels are reflected by this. The two that are not are the front
    coxae, whose envelope is symmetric about zero and therefore carries no side
    information -- which is why the middle and rear legs decide the question.
    """
    frames = sweep(c)
    flipped = {ch: (("R" if leg[0] == "L" else "L") + leg[1], joint)
               for ch, (leg, joint) in CHANNEL_MAP.items()}
    import tools.vendor_poses as vp
    original = vp.CHANNEL_MAP
    try:
        vp.CHANNEL_MAP = flipped
        problems = vp.check_envelopes(frames, c)
    finally:
        vp.CHANNEL_MAP = original
    assert len(problems) == 16, "expected 16 reflected channels, got %d" % len(problems)


def test_front_coxa_envelope_is_symmetric(c):
    """States the reason the previous test expects 16 and not 18."""
    lo, hi = c["joint_envelopes_deg"]["coxa_front"]["left"]
    assert lo == -hi


# --------------------------------------------------------------------------
# Pairing
# --------------------------------------------------------------------------

def test_pairing_derives_to_23_minus_r(c):
    table = derive_mirror_pairs(sweep(c), c)
    assert check_mirror_against_map(table) == []
    for r, row in table.items():
        assert row["partner"] == 23 - r, "#%03d paired with #%03d" % (r, row["partner"])


def test_pairing_survives_the_tripod_coincidence(c):
    """R1, R3 and L2 lift together, so their femur counts track each other.

    Scoring the femur over every frame then prefers L2 as R1's partner. The
    two-stage derivation scores the femur only over frames the coxa and tibia
    have already shown to be symmetric, and gets the right answer.
    """
    step = float(c["command_step_deg"])
    centre = int(c["pwm_centre_count"])
    tripod = {"R1", "R3", "L2"}
    walking, standing = 300, N_FRAMES_EXPECTED - 300
    per_position = {"1": 27.0, "2": -13.5, "3": 40.5}
    frames = []

    for i in range(N_FRAMES_EXPECTED):
        turning = i < walking
        pwm = {ch: centre for ch in range(32)}
        for ch, (leg, joint) in CHANNEL_MAP.items():
            if joint == "femur":
                # kinematic angle, then back through the sign to a raw count
                kin = ((54.0 if leg in tripod else -27.0) if turning
                       else per_position[leg[1]])
                pwm[ch] = centre + round(kin / sign_for(leg, joint) / step)
            elif turning:
                # a turn is not left-right symmetric: drive both sides alike,
                # which in mirrored vendor space is exactly not a mirror
                pwm[ch] = _counts_for(leg, joint, 0.25, c)
            else:
                f = {"1": 0.2, "2": 0.55, "3": 0.8}[leg[1]]
                pwm[ch] = _counts_for(leg, joint, f if leg.startswith("L") else 1.0 - f, c)
        frames.append((i, pwm, {}))

    naive = {r: max(((sum(1 for _g, p, _t in frames
                          if p[r] + p[l] == 2 * centre), l)
                     for l in (16, 19, 22)))[1] for r in (1, 4, 7)}
    assert naive[7] != 16, (
        "the frames do not contain the coincidence this test exists to defeat")

    table = derive_mirror_pairs(frames, c)
    for r in (7, 4, 1):
        assert table[r]["partner"] == 23 - r, (
            "#%03d paired with #%03d -- the tripod won" % (r, table[r]["partner"]))


# --------------------------------------------------------------------------
# Sign map and the command grid
# --------------------------------------------------------------------------

def test_sign_map_is_per_joint_not_per_leg(c):
    assert sign_for("R1", "coxa") == sign_for("L1", "coxa") == 1.0
    assert sign_for("L1", "femur") == 1.0 and sign_for("R1", "femur") == -1.0
    assert sign_for("L3", "tibia") == 1.0 and sign_for("R3", "tibia") == -1.0


def test_the_sign_map_leaves_coxa_antisymmetric_and_the_rest_symmetric(c):
    """What the sign map is FOR, and it is not "make both sides equal".

    The vendor mirror is uniform -- raw_R = -raw_L on every channel. The
    kinematics are not: for a left-right symmetric pose theta1 is
    antisymmetric and theta2, theta3 are symmetric. After the sign map the
    coxa must still come out equal and OPPOSITE, and only the femur and tibia
    equal. A map that made all three equal would have flattened the coxa.
    """
    for p in to_angles(sweep(c), c):
        for i in ("1", "2", "3"):
            assert abs(p[("L" + i, "coxa")] + p[("R" + i, "coxa")]) < 1e-9
            for joint in ("femur", "tibia"):
                assert abs(p[("L" + i, joint)] - p[("R" + i, joint)]) < 1e-9


def test_a_pose_exactly_on_the_limit_is_not_a_breach(c):
    """0.1350 has no exact binary form.

    (1000 - 1500) * 0.1350 is -67.50000000000001; the envelope endpoint parsed
    from JSON is -67.5. A degree comparison calls that outside the limit and
    reports a breach for every pose sitting exactly ON it -- 918 of them, on
    legs whose printed min and max were both inside. Comparison happens on the
    integer command grid for this reason.
    """
    frames = sweep(c)
    _ranges, breaches, _tm, _ta = analyse(to_angles(frames, c), c)
    for leg, joint in CHANNEL_MAP.values():
        if joint == "tibia":
            continue    # the six-leg intersection is narrower by design
        assert breaches[(leg, joint)] == [], (
            "%s %s: %d poses called outside a limit they sit on"
            % (leg, joint, len(breaches[(leg, joint)])))


def test_raw_range_is_reported_in_vendor_space(c):
    """joint_envelopes_deg is vendor space, so raw_range must not sign-map."""
    frames = sweep(c)
    lo, hi = raw_range(frames, 15, c)               # R1 coxa
    _key, e_lo, e_hi = envelope_for("R1", "coxa", c["joint_envelopes_deg"])
    assert abs(lo - e_lo) < 1e-9 and abs(hi - e_hi) < 1e-9


# --------------------------------------------------------------------------
# The committed report
# --------------------------------------------------------------------------

def test_report_matches_current_config():
    """A committed report is a result, and a result goes stale when its inputs move.

    The report records the SHA-256 of config/hexapod.json. If the constants
    change and nobody re-runs the tool, this fails and names the file. Without
    it the repository would publish numbers computed from a table it no longer
    contains.
    """
    if not REPORT.exists():
        pytest.fail("docs/vendor_pose_check.md is missing; re-run tools/vendor_poses.py")
    text = REPORT.read_text(encoding="utf-8")
    m = re.search(r"`config/hexapod\.json`, SHA-256 `([0-9a-f]{64})`", text)
    assert m, "the report does not record which constant table produced it"
    actual = text_sha256(ROOT / "config" / "hexapod.json")
    assert m.group(1) == actual, (
        "config/hexapod.json has changed since the report was generated.\n"
        "  report: %s\n  actual: %s\n"
        "Re-run: python -m tools.vendor_poses --actions <your copy>" % (m.group(1), actual))


def test_report_does_not_reproduce_the_vendor_data():
    """docs/THIRD_PARTY.md permits statistics, not poses.

    A frame is 32 entries of the form #nnnPnnnnTnnnn. If one ever appears in
    the report, the report has stopped being a summary.
    """
    text = REPORT.read_text(encoding="utf-8")
    assert not re.search(r"#\d{3}P\d{4}T\d{4}", text), (
        "the report contains vendor frame data verbatim")


def test_line_endings_are_pinned():
    """The guard for CI #6. Deleting .gitattributes must fail here, not there.

    Two things depend on it: the config stamp above, and the print set's
    checksums. Neither PDF carries a NUL byte in its first 8000, so without an
    explicit `binary` attribute Git treats them as text and rewrites them on a
    Windows checkout.
    """
    attrs = ROOT / ".gitattributes"
    assert attrs.exists(), ".gitattributes is missing; see CI #6"
    text = attrs.read_text()
    assert "eol=lf" in text, "text files are not pinned to LF"
    for ext in (".pdf", ".png"):
        assert re.search(r"\*%s\s+binary" % re.escape(ext), text), (
            "%s is not marked binary; a Windows checkout will corrupt it" % ext)


def test_vendor_file_is_not_committed():
    tracked = [p for p in ROOT.rglob("*.ini") if ".git" not in p.parts]
    assert tracked == [], "vendor .ini present in the tree: %s" % tracked
