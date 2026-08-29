"""Check an MP2 collection manifest against the frames on disk.

Three things about a photograph cannot be recovered after the session ends:
the TRUE DISTANCE, WHICH SESSION it came from, and whether it was DELIBERATELY
HARD. Everything else can be redone by re-processing the file. This script
exists to make the loss of those three loud on the day it happens rather than
in December.

    python -m tools.check_manifest --manifest data/manifest.csv --frames data/frames

Exit status is 1 if anything is an error. Warnings do not fail: they are
things that are probably wrong and are the operator's call.
"""

import argparse
import csv
import sys
from collections import Counter, defaultdict
from pathlib import Path

REQUIRED = ["session_id", "frame", "marker_id", "marker_width_mm", "distance_mm",
            "distance_method", "lighting", "background", "blur", "hard"]

FRAMES_PER_SESSION = 50
SESSIONS_PLANNED = 40
NOMINAL_MARKER_MM = 100.0

DISTANCE_BINS_MM = [(0, 600), (600, 1200), (1200, 2000), (2000, 3200), (3200, 10000)]
LIGHTING = {"bright", "normal", "dim", "mixed", "backlit"}
BACKGROUND = {"plain", "cluttered", "similar_texture"}
BLUR = {"none", "motion", "defocus"}
METHODS = {"tape", "laser", "jig"}


def load(path):
    with open(path, newline="", encoding="utf-8-sig") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        return [], ["manifest is empty"]
    missing = [c for c in REQUIRED if c not in rows[0]]
    return rows, (["manifest is missing columns: %s" % ", ".join(missing)] if missing else [])


def check(rows, frames_dir):
    errors, warnings = [], []
    seen = set()
    per_session = defaultdict(list)
    dist_hist, light_hist, back_hist, blur_hist = Counter(), Counter(), Counter(), Counter()
    widths = Counter()
    hard = 0

    for i, r in enumerate(rows, start=2):          # row 1 is the header
        where = "row %d (%s)" % (i, r.get("frame") or "no frame")

        name = (r.get("frame") or "").strip()
        if not name:
            errors.append("%s: no frame filename" % where)
        elif name in seen:
            errors.append("%s: filename listed twice" % where)
        else:
            seen.add(name)
            if frames_dir and not (Path(frames_dir) / name).exists():
                errors.append("%s: file not found under %s" % (where, frames_dir))

        sid = (r.get("session_id") or "").strip()
        if not sid:
            errors.append("%s: no session_id -- NOT RECOVERABLE LATER" % where)
        else:
            per_session[sid].append(name)

        # --- distance. The one number the whole dataset is for. ---
        raw = (r.get("distance_mm") or "").strip()
        try:
            d = float(raw)
            if d <= 0:
                errors.append("%s: distance_mm is %s" % (where, raw))
            else:
                for lo, hi in DISTANCE_BINS_MM:
                    if lo <= d < hi:
                        dist_hist[(lo, hi)] += 1
                        break
        except ValueError:
            errors.append("%s: distance_mm %r is not a number -- NOT RECOVERABLE LATER"
                          % (where, raw))

        if (r.get("distance_method") or "").strip() not in METHODS:
            warnings.append("%s: distance_method %r is not one of %s"
                            % (where, r.get("distance_method"), sorted(METHODS)))

        # --- the printed marker's true width ---
        raw_w = (r.get("marker_width_mm") or "").strip()
        try:
            w = float(raw_w)
            widths[w] += 1
            if w == NOMINAL_MARKER_MM:
                warnings.append(
                    "%s: marker_width_mm is exactly %.1f. That is the NOMINAL value, "
                    "not a measurement -- a printer that came out at 99.6 gives the "
                    "same number here and a 0.4%% range error everywhere."
                    % (where, NOMINAL_MARKER_MM))
        except ValueError:
            errors.append("%s: marker_width_mm %r is not a number" % (where, raw_w))

        for field, allowed, hist in (("lighting", LIGHTING, light_hist),
                                     ("background", BACKGROUND, back_hist),
                                     ("blur", BLUR, blur_hist)):
            v = (r.get(field) or "").strip()
            if v not in allowed:
                warnings.append("%s: %s %r is not one of %s"
                                % (where, field, v, sorted(allowed)))
            else:
                hist[v] += 1

        if (r.get("hard") or "").strip() in ("1", "true", "yes"):
            hard += 1

    # --- untracked files ---
    if frames_dir:
        on_disk = {p.name for p in Path(frames_dir).rglob("*")
                   if p.is_file() and p.suffix.lower() in (".jpg", ".jpeg", ".png")}
        for extra in sorted(on_disk - seen):
            errors.append("%s is on disk but not in the manifest -- an unlabelled "
                          "frame has no distance and no session" % extra)

    for sid, names in sorted(per_session.items()):
        if len(names) != FRAMES_PER_SESSION:
            warnings.append("session %s has %d frames, planned %d"
                            % (sid, len(names), FRAMES_PER_SESSION))

    return errors, warnings, {
        "rows": len(rows), "sessions": len(per_session), "hard": hard,
        "distance": dist_hist, "lighting": light_hist,
        "background": back_hist, "blur": blur_hist, "widths": widths,
    }


def report(errors, warnings, s):
    print("frames listed   : %d" % s["rows"])
    print("sessions        : %d of %d planned" % (s["sessions"], SESSIONS_PLANNED))
    print("deliberately hard: %d (%.1f%%)"
          % (s["hard"], 100.0 * s["hard"] / max(1, s["rows"])))

    print("\ndistance, mm")
    for lo, hi in DISTANCE_BINS_MM:
        n = s["distance"][(lo, hi)]
        print("  %5d - %-5d  %4d  %s" % (lo, hi, n, "#" * min(40, n // 5)))
    empty = [b for b in DISTANCE_BINS_MM if not s["distance"][b]]
    if empty:
        print("  EMPTY BINS: %s -- diversity designed in advance beats diversity "
              "accumulated" % ", ".join("%d-%d" % b for b in empty))

    for k in ("lighting", "background", "blur"):
        print("\n%s" % k)
        for v, n in sorted(s[k].items()):
            print("  %-16s %4d" % (v, n))

    if len(s["widths"]) > 1:
        print("\nmarker_width_mm values in use: %s"
              % ", ".join("%.4f (x%d)" % (w, n) for w, n in sorted(s["widths"].items())))

    if warnings:
        print("\n%d warning(s)" % len(warnings))
        for w in warnings[:25]:
            print("  ! %s" % w)
        if len(warnings) > 25:
            print("  ... and %d more" % (len(warnings) - 25))

    if errors:
        print("\n%d ERROR(s)" % len(errors))
        for e in errors[:25]:
            print("  X %s" % e)
        if len(errors) > 25:
            print("  ... and %d more" % (len(errors) - 25))
    else:
        print("\nno errors")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--frames", default=None,
                    help="folder holding the image files; omit to check the CSV alone")
    a = ap.parse_args(argv)

    rows, header_errors = load(a.manifest)
    if header_errors:
        for e in header_errors:
            print("X %s" % e)
        return 1
    errors, warnings, s = check(rows, a.frames)
    report(errors, warnings, s)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
