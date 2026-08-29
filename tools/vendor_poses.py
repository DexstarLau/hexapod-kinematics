"""Read the vendor's hexapod action group and convert it to kinematic angles.

THIRD-PARTY DATA. The file this reads is the manufacturer's, is copyrighted,
and is NOT redistributed with this repository. See docs/THIRD_PARTY.md. Supply
your own copy with --actions; nothing derived from it that would reconstruct it
is written to the repository either. What this tool commits is a REPORT --
counts, ranges and residual statistics -- plus a SHA-256 of the input so a
result can be traced to the exact file that produced it.

Usage:
    python -m tools.vendor_poses --actions "C:/path/Lm2...V3.ini"
    python -m tools.vendor_poses --actions <path> --report docs/vendor_pose_check.md
"""

import argparse
import hashlib
import re
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from sim.constants import load                                    # noqa: E402

# --------------------------------------------------------------------------
# 1. The channel map
# --------------------------------------------------------------------------
#
# 32 channels are present in every frame. 18 are leg joints, in two blocks of
# nine, and the two blocks run in OPPOSITE order -- the left block is the right
# block reversed, so channel r pairs with channel 23 - r:
#
#     low  block #000..#008   per leg: tibia, femur, coxa   legs L3, L2, L1
#     high block #015..#023   per leg: coxa, femur, tibia   legs R1, R2, R3
#
# THE LOW BLOCK IS THE LEFT SIDE. The obvious reading puts #000 on leg one of
# the right side and it is wrong on both counts.
#
# #009 and #013 never move; #010-#012 and #014 move but are the arm's;
# #024-#031 are unpopulated and sit at centre. Do not loop 0..17.
#
# TWO SEPARATE THINGS HAVE TO BE ESTABLISHED and only one of them is what the
# mirror statistic establishes:
#
#   the PAIRING   -- which right channel faces which left channel.
#                    derive_mirror_pairs() recovers this from the counts.
#   the LABELLING -- which of the three is coxa, which leg, and which side.
#                    check_envelopes() recovers this, from travel.
#
# The mirror test cannot see the labelling at all. Relabel every channel
# consistently -- call the tibia the coxa on both sides -- and every mirror
# pair still holds, because the pairing is untouched. A first version of this
# file did exactly that: it read each right triple as coxa, femur, tibia when
# the block is wired tibia, femur, coxa. All nine pairs derived, the report
# said the map agreed with the data, and the labels were wrong. What caught it
# was travel: the channel called the coxa swung 202.5000 deg.
#
# The independent evidence is that observed travel matches
# joint_envelopes_deg span-for-span under this labelling and under no other:
# six coxae onto three coxa envelopes, six femurs onto femur_all, six tibiae
# onto two tibia envelopes, all to four decimals. check_spans() asserts it.
#
# CAVEAT, and it is not resolved here: joint_envelopes_deg carries status
# provisional. If those envelopes were themselves read off this same vendor
# file by whoever supplied them, the span match is a restatement, not a
# confirmation. Their provenance is a question for coordination.

CHANNEL_MAP = {
    #  channel: (leg, joint)
    0: ("L3", "tibia"),  1: ("L3", "femur"),  2: ("L3", "coxa"),
    3: ("L2", "tibia"),  4: ("L2", "femur"),  5: ("L2", "coxa"),
    6: ("L1", "tibia"),  7: ("L1", "femur"),  8: ("L1", "coxa"),
    15: ("R1", "coxa"),  16: ("R1", "femur"), 17: ("R1", "tibia"),
    18: ("R2", "coxa"),  19: ("R2", "femur"), 20: ("R2", "tibia"),
    21: ("R3", "coxa"),  22: ("R3", "femur"), 23: ("R3", "tibia"),
}

ARM_CHANNELS = (9, 10, 11, 12, 13, 14)
UNPOPULATED = tuple(range(24, 32))
N_FRAMES_EXPECTED = 395          # G0000 is the all-centre home frame; 394 poses follow


# --------------------------------------------------------------------------
# 2. Parsing
# --------------------------------------------------------------------------

FRAME_RE = re.compile(r"\{G(\d+)((?:#\d+P\d+T\d+!)*)\}")
ENTRY_RE = re.compile(r"#(\d+)P(\d+)T(\d+)!")


def read_actions(path):
    """Return (frames, sha256). frames[i] = (frame_id, {channel: pwm}, {channel: ms})."""
    raw = Path(path).read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    for enc in ("utf-8", "gbk", "utf-16"):
        try:
            text = raw.decode(enc)
            break
        except UnicodeDecodeError:
            continue
    else:
        raise ValueError("could not decode %s as utf-8, gbk or utf-16" % path)

    m = re.search(r"^group_liu_zu=(.*)$", text, re.M)
    if m is None:
        raise ValueError("no 'group_liu_zu' key in %s -- wrong file?" % path)

    frames = []
    for gid, body in FRAME_RE.findall(m.group(1)):
        pwm, ms = {}, {}
        for ch, p, t in ENTRY_RE.findall(body):
            pwm[int(ch)] = int(p)
            ms[int(ch)] = int(t)
        frames.append((int(gid), pwm, ms))
    return frames, digest


def check_structure(frames, c):
    """Fail loudly on anything that would make the rest of the run meaningless."""
    problems = []
    if len(frames) != N_FRAMES_EXPECTED:
        problems.append("expected %d frames, found %d" % (N_FRAMES_EXPECTED, len(frames)))

    ids = [g for g, _, _ in frames]
    if ids != list(range(len(ids))):
        problems.append("frame ids are not G0000..G%04d contiguous" % (len(ids) - 1))

    lo = int(c["pwm_count_min"])
    hi = int(c["pwm_count_max"])
    centre = int(c["pwm_centre_count"])

    for gid, pwm, _ in frames:
        if set(pwm) != set(range(32)):
            problems.append("frame G%04d does not carry channels 0..31" % gid)
            break

    for ch in UNPOPULATED + (9, 13):
        vals = {pwm[ch] for _, pwm, _ in frames}
        if vals != {centre}:
            problems.append("channel #%03d was expected to hold %d, found %s"
                            % (ch, centre, sorted(vals)))

    for gid, pwm, _ in frames:
        for ch, p in pwm.items():
            if not lo <= p <= hi:
                problems.append("G%04d #%03d pwm %d outside [%d, %d]" % (gid, ch, p, lo, hi))
    return problems


# --------------------------------------------------------------------------
# 3. Re-derive the channel map from the data, do not trust the table above
# --------------------------------------------------------------------------

def _score(frames, r, candidates, total):
    """(hits, channel) for every candidate partner of r, best first."""
    return sorted(((sum(1 for _, pwm, _ in frames if pwm[r] + pwm[l] == total), l)
                   for l in candidates), reverse=True)


def derive_mirror_pairs(frames, c):
    """Recover the right-to-left channel pairing from the counts alone.

    The vendor mirror is pwm_R + pwm_L == 2 * centre, and it holds only for a
    left-right SYMMETRIC pose. A turning frame is not symmetric, so this is
    never a universal property of the file and any claim that it holds on all
    394 action groups is false as stated.

    TWO STAGES, because one stage gets the femur wrong.

    Stage 1, coxa and tibia. Scored over every frame. Both resolve cleanly.

    Stage 2, femur. Scored ONLY over frames that stage 1 has shown to be
    symmetric. Scoring the femur over all frames picks the wrong partner and
    picks it confidently: in a tripod gait R1, R3 and L2 lift together, so
    #001, #007 and #019 carry the same commanded femur angle for the whole of
    every walking sequence, and #001 + #019 == 3000 on more frames than
    #001 + #022 does. The runner-up is the truth and the winner is the tripod.
    A single-pass derivation reports a confident, wrong map and nothing
    downstream notices.
    """
    centre = int(c["pwm_centre_count"])
    total = 2 * centre
    right = [ch for ch in CHANNEL_MAP if ch < 9]
    left = [ch for ch in CHANNEL_MAP if ch >= 15]
    table = {}

    for joint in ("coxa", "tibia"):
        for r in [x for x in right if CHANNEL_MAP[x][1] == joint]:
            s = _score(frames, r, [l for l in left if CHANNEL_MAP[l][1] == joint], total)
            table[r] = {"partner": s[0][1], "hits": s[0][0],
                        "runner_up": s[1][1], "runner_up_hits": s[1][0],
                        "frames": len(frames), "basis": "all frames"}

    symmetric = [f for f in frames
                 if all(f[1][r] + f[1][table[r]["partner"]] == total for r in table)]

    for r in [x for x in right if CHANNEL_MAP[x][1] == "femur"]:
        s = _score(symmetric, r, [l for l in left if CHANNEL_MAP[l][1] == "femur"], total)
        table[r] = {"partner": s[0][1], "hits": s[0][0],
                    "runner_up": s[1][1], "runner_up_hits": s[1][0],
                    "frames": len(symmetric),
                    "basis": "%d symmetric frames" % len(symmetric)}
    return table


def raw_range(frames, ch, c):
    """Observed travel of one channel in VENDOR command degrees."""
    step = float(c["command_step_deg"])
    centre = int(c["pwm_centre_count"])
    vals = [pwm[ch] for _, pwm, _ in frames]
    return (min(vals) - centre) * step, (max(vals) - centre) * step


def check_envelopes(frames, c, tol=1e-9):
    """Assert each channel reproduces the vendor envelope it is labelled with.

    This is the check that fixes the LABELLING, and it is the one that did not
    exist while CHANNEL_MAP was wrong -- twice. joint_envelopes_deg is in
    vendor servo space, so no sign map and no offset stand between a raw count
    and this comparison: it is 36 numbers against 36 numbers.
    """
    envelopes = c["joint_envelopes_deg"]
    problems = []
    for ch, (leg, joint) in sorted(CHANNEL_MAP.items()):
        lo_o, hi_o = raw_range(frames, ch, c)
        key, lo, hi = envelope_for(leg, joint, envelopes)
        if abs(lo_o - lo) > tol or abs(hi_o - hi) > tol:
            problems.append(
                "#%03d labelled %s %s: raw [%.4f, %.4f], but `%s` %s is [%.4f, %.4f]"
                % (ch, leg, joint, lo_o, hi_o, key,
                   "left" if leg.startswith("L") else "right", lo, hi))
    return problems


def check_mirror_against_map(table):
    """A derived partner must be the same leg-position and joint on the far side."""
    problems = []
    for r, row in table.items():
        leg_r, joint_r = CHANNEL_MAP[r]
        leg_l, joint_l = CHANNEL_MAP[row["partner"]]
        if joint_r != joint_l:
            problems.append("#%03d (%s %s) mirrors #%03d (%s %s) -- joints differ"
                            % (r, leg_r, joint_r, row["partner"], leg_l, joint_l))
        if leg_r[1] != leg_l[1]:
            problems.append("#%03d (%s) mirrors #%03d (%s) -- not the same leg position"
                            % (r, leg_r, row["partner"], leg_l))
        if row["hits"] <= row["runner_up_hits"]:
            problems.append("#%03d has no clear mirror partner: %d vs %d"
                            % (r, row["hits"], row["runner_up_hits"]))
    return problems


# --------------------------------------------------------------------------
# 4. Vendor command space -> kinematic space
# --------------------------------------------------------------------------

def sign_for(leg, joint):
    """Per-JOINT sign map, not per-leg. bindings/hexconfig.sign_map is the source."""
    if joint == "coxa":
        return 1.0
    return 1.0 if leg.startswith("L") else -1.0


def to_angles(frames, c):
    """frames -> [{(leg, joint): kinematic_degrees}], one dict per frame.

    angle = (pwm - centre) * command_step_deg, then the per-joint sign.

    The command OFFSET is NOT applied and is not assumed zero for femur or
    tibia (D262, hardware's). Everything downstream of this function is
    therefore a comparison of SHAPE. A constant per-joint bias appears as a
    constant residual and must not be read as a defect.
    """
    centre = int(c["pwm_centre_count"])
    step = float(c["command_step_deg"])
    out = []
    for _, pwm, _ in frames:
        pose = {}
        for ch, (leg, joint) in CHANNEL_MAP.items():
            pose[(leg, joint)] = (pwm[ch] - centre) * step * sign_for(leg, joint)
        out.append(pose)
    return out


# --------------------------------------------------------------------------
# 5. What the poses say about the model
# --------------------------------------------------------------------------

def envelope_for(leg, joint, envelopes):
    side = "left" if leg.startswith("L") else "right"
    pos = leg[1]
    if joint == "coxa":
        key = {"1": "coxa_front", "2": "coxa_middle", "3": "coxa_rear"}[pos]
    elif joint == "femur":
        key = "femur_all"
    else:
        key = "tibia_rear" if pos == "3" else "tibia_front_middle"
    lo, hi = envelopes[key][side]
    return key, float(lo), float(hi)


def analyse(poses, c):
    """Kinematic travel per joint, against the PROJECTED kinematic limits.

    The comparison in section 4 of the report has to happen in one space or the
    other and cannot straddle them. joint_envelopes_deg is vendor space;
    project_joint_limits and project_theta3_envelope carry it into kinematic
    space with the per-joint sign map. Comparing a kinematic angle against a
    vendor envelope reflects half the table and reports breaches that are an
    artefact of the reader.
    """
    from bindings.hexconfig import (LEGS, hex_coxa, hex_femur,
                                    project_joint_limits, project_theta3_envelope)
    envelopes = c["joint_envelopes_deg"]
    step = float(c["command_step_deg"])
    mins, maxs = project_joint_limits(envelopes)
    t_min, t_max = project_theta3_envelope(envelopes)

    ranges, breaches = {}, {}
    for leg, joint in CHANNEL_MAP.values():
        i = LEGS.index(leg)
        vals = [p[(leg, joint)] for p in poses]
        if joint == "coxa":
            lo, hi, src = mins[hex_coxa(i)], maxs[hex_coxa(i)], "projected coxa"
        elif joint == "femur":
            lo, hi, src = mins[hex_femur(i)], maxs[hex_femur(i)], "projected femur"
        else:
            lo, hi, src = t_min, t_max, "theta3 intersection"
        # COUNT THE COMMAND GRID, DO NOT COMPARE DEGREES.
        # command_step_deg is 0.1350, which has no exact binary form, so
        # (1000 - 1500) * 0.1350 is -67.50000000000001 while the envelope
        # endpoint parsed from JSON is -67.5 exactly. A naive v < lo then
        # reports every pose that sits exactly ON the limit as outside it --
        # 134 phantom breaches on a leg whose min and max both printed as
        # inside. Both sides are whole multiples of the step, so round to the
        # grid and compare integers, where the question has an exact answer.
        g = lambda x: round(x / step)
        lo_n, hi_n = g(lo), g(hi)
        ranges[(leg, joint)] = (min(vals), max(vals), len(set(vals)), src, lo, hi)
        breaches[(leg, joint)] = [i for i, v in enumerate(vals, start=1)
                                  if not lo_n <= g(v) <= hi_n]

    # The tibia is a fixed member in this model (HEX_JOINTS = 12, D199).
    # Does the vendor hold it fixed?
    tibia_moves = Counter()
    for p in poses:
        vals = {round(p[(leg, "tibia")], 6) for leg in ("R1", "R2", "R3", "L1", "L2", "L3")}
        tibia_moves[len(vals)] += 1
    tibia_all = sorted({round(p[(leg, "tibia")], 4)
                        for p in poses
                        for leg in ("R1", "R2", "R3", "L1", "L2", "L3")})
    return ranges, breaches, tibia_moves, tibia_all


# --------------------------------------------------------------------------
# 6. Report
# --------------------------------------------------------------------------

def write_report(path, src, digest, frames, structure, spans, table, mirror_problems,
                 ranges, breaches, tibia_moves, tibia_all, c, stamp):
    L = []
    w = L.append
    w("# Vendor pose set -- extraction and structural check\n")
    w("Generated by `tools/vendor_poses.py`. **The source file is the manufacturer's, "
      "is copyrighted, and is not in this repository.** No pose is reproduced below; "
      "this report holds counts, ranges and statistics only.\n")
    w("| | |")
    w("|---|---|")
    w("| Source file name | `%s` |" % Path(src).name)
    w("| SHA-256 | `%s` |" % digest)
    w("| Frames | %d (`G0000` home + %d poses) |" % (len(frames), len(frames) - 1))
    w("| Constants | `config/hexapod.json`, SHA-256 `%s` |" % stamp["config_sha256"])
    w("| Surrogates used | %s |" % (", ".join(stamp["surrogates"]) or "none"))
    w("")
    w("## 1. Structure\n")
    if structure:
        w("**FAILED.**\n")
        for p in structure:
            w("- %s" % p)
    else:
        w("All checks passed: %d contiguous frames, 32 channels each, every count "
          "inside [%d, %d], channels #009 #013 and #024-#031 held at centre.\n"
          % (len(frames), int(c["pwm_count_min"]), int(c["pwm_count_max"])))
    w("")
    w("## 2. Labelling -- which channel is which joint\n")
    w("The mirror statistic in section 3 fixes which right channel faces which left "
      "channel. It cannot fix which of the three is the coxa: relabel all 18 "
      "consistently and every mirror pair still holds. Travel does fix it. A "
      "constant command offset shifts both ends of a range equally and leaves the "
      "span alone, so span survives the unknown offset.\n")
    if spans:
        w("**FAILED -- the labelling does not match the envelopes:**\n")
        for q in spans:
            w("- %s" % q)
    else:
        w("All 18 channels match the span of the envelope they are labelled with, to "
          "four decimals. **Caveat:** `joint_envelopes_deg` is `provisional`. If it "
          "was itself read off this vendor file, this is a restatement and not a "
          "confirmation; its provenance is an open question.\n")
    w("")
    w("## 3. Pairing, re-derived from the data\n")
    w("For each right channel, the left channel whose counts sum to %d on the most "
      "frames. A pose that turns is not left-right symmetric, so this is a majority "
      "property, never a universal one.\n" % (2 * int(c["pwm_centre_count"])))
    w("| Right | | Best partner | Mirrored | Next best | its count | Scored over |")
    w("|---|---|---|---|---|---|---|")
    for r in sorted(table):
        row = table[r]
        leg_r, joint_r = CHANNEL_MAP[r]
        leg_l, joint_l = CHANNEL_MAP[row["partner"]]
        w("| `#%03d` | %s %s | `#%03d` %s %s | %d / %d | `#%03d` | %d | %s |"
          % (r, leg_r, joint_r, row["partner"], leg_l, joint_l,
             row["hits"], row["frames"], row["runner_up"], row["runner_up_hits"],
             row["basis"]))
    w("")
    if mirror_problems:
        w("**Derived map disagrees with the declared map:**\n")
        for p in mirror_problems:
            w("- %s" % p)
    else:
        w("The derived map agrees with `CHANNEL_MAP` on all nine pairs, and every "
          "pair beats its runner-up.\n")
    w("")
    w("## 4. Joint travel, kinematic space\n")
    w("Per-joint sign applied; **no command offset**, which is hardware's and is not "
      "assumed zero for femur or tibia (D262). These are shapes, not absolute angles. "
      "Limits are `joint_envelopes_deg` carried into kinematic space by "
      "`project_joint_limits`; the tibia row is the six-leg intersection from "
      "`project_theta3_envelope`, which is why one pair serves all six legs.\n")
    w("| Leg | Joint | min | max | distinct | Limit source | lo | hi | Poses outside |")
    w("|---|---|---|---|---|---|---|---|---|")
    for leg in ("R1", "R2", "R3", "L1", "L2", "L3"):
        for joint in ("coxa", "femur", "tibia"):
            lo_v, hi_v, n, key, lo, hi = ranges[(leg, joint)]
            b = breaches[(leg, joint)]
            cell = "0" if not b else ("%d (%s)" % (
                len(b), ", ".join("G%04d" % i for i in b[:4]) + (" ..." if len(b) > 4 else "")))
            w("| %s | %s | %.4f | %.4f | %d | `%s` | %.4f | %.4f | %s |"
              % (leg, joint, lo_v, hi_v, n, key, lo, hi, cell))
    w("")
    w("## 4. The tibia\n")
    w("`hex_config.h` commands twelve joints and holds the tibia at a single "
      "`theta3_deg`. Distinct tibia angles per pose, across the six legs:\n")
    w("| Distinct tibia angles in one pose | Poses |")
    w("|---|---|")
    for k in sorted(tibia_moves):
        w("| %d | %d |" % (k, tibia_moves[k]))
    w("")
    w("Distinct tibia angles anywhere in the set: **%d** -- %s\n"
      % (len(tibia_all), ", ".join("%.4f" % v for v in tibia_all)))
    w("Configured `theta3_deg` = **%.4f** (status `surrogate`).\n" % float(c["theta3_deg"]))
    Path(path).write_text("\n".join(L) + "\n", encoding="utf-8")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--actions", required=True, help="path to your own copy of the vendor .ini")
    ap.add_argument("--report", default=str(ROOT / "docs" / "vendor_pose_check.md"))
    a = ap.parse_args(argv)

    c = load()
    frames, digest = read_actions(a.actions)
    structure = check_structure(frames, c)
    spans = check_envelopes(frames, c)
    table = derive_mirror_pairs(frames, c)
    mirror_problems = check_mirror_against_map(table)
    poses = to_angles(frames[1:], c)          # drop G0000, the home frame
    ranges, breaches, tibia_moves, tibia_all = analyse(poses, c)

    stamp = {
        "config_sha256": hashlib.sha256(
            (ROOT / "config" / "hexapod.json").read_bytes()).hexdigest(),
        "surrogates": c.surrogates_read(),
    }
    write_report(a.report, a.actions, digest, frames, structure, spans, table,
                 mirror_problems, ranges, breaches, tibia_moves, tibia_all, c, stamp)

    print("frames            : %d" % len(frames))
    print("structure problems: %d" % len(structure))
    for p in structure:
        print("   ", p)
    print("span problems     : %d" % len(spans))
    for q in spans:
        print("   ", q)
    print("mirror problems   : %d" % len(mirror_problems))
    for p in mirror_problems:
        print("   ", p)
    print("limit breaches    : %d joint-poses" % sum(len(v) for v in breaches.values()))
    print("distinct tibia    : %d" % len(tibia_all))
    print("report            : %s" % a.report)
    return 1 if (structure or spans or mirror_problems) else 0


if __name__ == "__main__":
    sys.exit(main())
