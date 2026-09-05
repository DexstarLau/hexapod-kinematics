"""Count the DISTINCT leg configurations in the vendor pose corpus.

THIRD-PARTY DATA. The file this reads is the manufacturer's, is copyrighted,
and is NOT redistributed with this repository. See docs/THIRD_PARTY.md. Supply
your own copy with --actions. What this tool prints is a COUNT, plus a SHA-256
of the input so the count can be traced to the exact file that produced it.

WHY THIS EXISTS (D348, and FINDING_11 section 5 before it)
----------------------------------------------------------
D275's Table 1 has 394 rows. It does not have 394 independent checks. The 394
action poses collapse to 71 distinct 18-channel configurations, and one single
configuration accounts for 24 of the rows. A suite reporting "394 green" has
exercised 71. The multiplicity is itself information and a table that hides it
misrepresents what was exercised.

This tool is what makes the labels in reports/D275_vendor_pose_validation.md
sections 4 and 5 checkable. A number in a report that no committed code can
reproduce is a hand-written number, however carefully it was derived.

TWO THINGS THAT ARE EASY TO GET WRONG, both recorded because both were hit
--------------------------------------------------------------------------
1.  DISTINCTNESS is counted in CHANNEL space, on the raw integer commands.
    That is deliberate: the map from command to angle is
    (pwm - centre) * step * sign, which is a bijection per channel, so the
    distinct count is identical either way -- and the integer grid has no
    floating-point comparison in it at all. See D258's note on 0.1350 having
    no exact binary form.

2.  SUBSET SELECTION is NOT. "Uniform tibia" means the six tibiae are at the
    same KINEMATIC ANGLE, and the two sides are mirrored -- sign_for() returns
    +1 on the left and -1 on the right for the tibia. Two legs at the same
    physical angle therefore have commands summing to 3000, not commands that
    are equal. Selecting on raw command equality returns 2 poses, not 284.
    The first version of this file did exactly that.

Usage:
    python -m tools.distinct_configurations --actions "C:/path/Lm2...V3.ini"
"""

import argparse
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from sim.constants import load                                    # noqa: E402
from tools.vendor_poses import (                                  # noqa: E402
    CHANNEL_MAP,
    read_actions,
    sign_for,
)

# The eighteen leg joints, taken from the map rather than from a range().
# #009-#014 are the arm's and #024-#031 are unpopulated. Never loop 0..17.
LEG_CHANNELS = sorted(CHANNEL_MAP)
TIBIA_CHANNELS = sorted(ch for ch, (_, j) in CHANNEL_MAP.items() if j == "tibia")


def configuration(frame):
    """The 18-tuple that identifies a pose, in raw command space."""
    _, pwm, _ = frame
    return tuple(pwm[ch] for ch in LEG_CHANNELS)


def angle_of(pwm_value, channel, centre, step):
    leg, joint = CHANNEL_MAP[channel]
    return (pwm_value - centre) * step * sign_for(leg, joint)


def uniform_tibia(frame, centre, step):
    """True when all six tibiae sit at one kinematic angle. Angle space, not command space."""
    _, pwm, _ = frame
    angles = {round(angle_of(pwm[ch], ch, centre, step), 4) for ch in TIBIA_CHANNELS}
    return len(angles) == 1


def summarise(frames):
    """(n_rows, n_distinct, largest_multiplicity, histogram) for a set of poses."""
    multiplicity = Counter(configuration(f) for f in frames)
    histogram = Counter(multiplicity.values())
    return (
        len(frames),
        len(multiplicity),
        max(multiplicity.values()) if multiplicity else 0,
        dict(sorted(histogram.items())),
    )


def format_histogram(histogram):
    return "  ".join("%dx:%d" % (k, v) for k, v in histogram.items())


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--actions", required=True, help="path to the vendor .ini")
    args = ap.parse_args(argv)

    k = load()
    centre = int(k.value("pwm_centre_count"))
    step = float(k.value("command_step_deg"))

    frames, digest = read_actions(args.actions)
    if not frames:
        raise SystemExit("no frames parsed -- wrong file?")

    # D302: frame 0 is G0000, the all-centre home frame. The corpus is what follows.
    home, action = frames[0], frames[1:]
    home_is_centre = set(home[1][ch] for ch in LEG_CHANNELS) == {centre}

    print("source sha256    %s" % digest)
    print("scope            [servo] group_liu_zu")
    print("frames           %d   ids %d..%d" % (len(frames), frames[0][0], frames[-1][0]))
    print("frame 0 at centre on all eighteen leg channels: %s" % home_is_centre)
    print("corpus           %d action poses                            (D302)" % len(action))
    print("counted over     %s" % ", ".join("#%03d" % c for c in LEG_CHANNELS))
    print()

    uniform = [f for f in action if uniform_tibia(f, centre, step)]

    for label, subset in (("Table 1  all action poses", action),
                          ("Table 2  uniform-tibia poses", uniform)):
        rows, distinct, largest, histogram = summarise(subset)
        assert sum(k_ * v for k_, v in histogram.items()) == rows
        print("%s" % label)
        print("  poses                        %d" % rows)
        print("  distinct configurations      %d" % distinct)
        print("  largest multiplicity         %d" % largest)
        print("  multiplicity histogram       %s" % format_histogram(histogram))
        print()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
