"""Tests for tools/check_manifest.py and for the committed print set.

The checker is pure stdlib and is tested properly here. The print set is NOT
regenerated in CI: tools/make_print_set.py needs opencv and reportlab, and
there is no guarantee of a wheel for every Python in the matrix. Pinning CI to
the availability of a third-party wheel for a one-off artefact would be
trading a real risk for no benefit.

The generator verifies itself instead, at the moment it runs -- it renders
each marker from the rectangle list it is about to draw and detects it back,
and it detects the board before embedding it. What is tested here is that the
committed PDFs are the ones that passed, by checksum.
"""

import csv
import hashlib
from pathlib import Path

import pytest

from tools.check_manifest import (FRAMES_PER_SESSION, NOMINAL_MARKER_MM, REQUIRED,
                                  check, load)

ROOT = Path(__file__).resolve().parent.parent
PRINT = ROOT / "print"
TEMPLATE = ROOT / "data" / "manifest_template.csv"

GOOD = {
    "session_id": "S001", "frame": "S001_0001.jpg", "marker_id": "0",
    "marker_width_mm": "99.6400", "distance_mm": "1500.0", "distance_method": "tape",
    "angle_deg": "15", "lighting": "normal", "background": "plain", "blur": "none",
    "occlusion_pct": "0", "hard": "0", "notes": "",
}


def rows(*overrides):
    out = []
    for i, over in enumerate(overrides, start=1):
        r = dict(GOOD)
        r["frame"] = "S001_%04d.jpg" % i
        r.update(over)
        out.append(r)
    return out


# --------------------------------------------------------------------------
# The three unrecoverable things must be ERRORS, not warnings
# --------------------------------------------------------------------------

def test_a_missing_distance_is_an_error():
    errors, _w, _s = check(rows({"distance_mm": ""}), None)
    assert any("distance_mm" in e and "NOT RECOVERABLE" in e for e in errors)


def test_a_non_numeric_distance_is_an_error():
    errors, _w, _s = check(rows({"distance_mm": "about 2m"}), None)
    assert any("not a number" in e for e in errors)


def test_a_missing_session_is_an_error():
    errors, _w, _s = check(rows({"session_id": ""}), None)
    assert any("session_id" in e and "NOT RECOVERABLE" in e for e in errors)


def test_a_frame_on_disk_but_not_in_the_manifest_is_an_error(tmp_path):
    (tmp_path / "S001_0001.jpg").touch()
    (tmp_path / "S001_0099.jpg").touch()          # shot, never written down
    errors, _w, _s = check(rows({}), tmp_path)
    assert any("S001_0099.jpg" in e for e in errors)


def test_a_manifest_row_with_no_file_is_an_error(tmp_path):
    errors, _w, _s = check(rows({}), tmp_path)
    assert any("not found" in e for e in errors)


def test_a_duplicate_filename_is_an_error():
    r = rows({}, {})
    r[1]["frame"] = r[0]["frame"]
    errors, _w, _s = check(r, None)
    assert any("twice" in e for e in errors)


# --------------------------------------------------------------------------
# The nominal-value trap
# --------------------------------------------------------------------------

def test_a_marker_width_of_exactly_nominal_warns():
    """100.0 is what the PDF says, not what the printer did.

    Nothing downstream can tell a measured 100.0 from an unmeasured one, and
    the error it causes is a clean multiplicative bias on every frame -- the
    kind that survives every sanity check because it moves nothing relative
    to anything else.
    """
    _e, warnings, _s = check(rows({"marker_width_mm": "%.1f" % NOMINAL_MARKER_MM}), None)
    assert any("NOMINAL" in w for w in warnings)


def test_a_measured_marker_width_does_not_warn():
    _e, warnings, _s = check(rows({"marker_width_mm": "99.6400"}), None)
    assert not any("NOMINAL" in w for w in warnings)


# --------------------------------------------------------------------------
# Diversity
# --------------------------------------------------------------------------

def test_distance_bins_are_counted_at_their_lower_edge():
    """2000.0 belongs to 2000-3200, not to 1200-2000.

    Half-open bins, stated: a value sitting exactly on a boundary has to land
    in one bin and it lands in the upper one.
    """
    _e, _w, s = check(rows({"distance_mm": "2000.0"}), None)
    assert s["distance"][(2000, 3200)] == 1
    assert s["distance"][(1200, 2000)] == 0


def test_hard_frames_are_counted():
    _e, _w, s = check(rows({"hard": "1"}, {"hard": "0"}, {"hard": "yes"}), None)
    assert s["hard"] == 2


def test_an_unknown_lighting_value_warns_but_does_not_fail():
    errors, warnings, _s = check(rows({"lighting": "quite dark"}), None)
    assert errors == []
    assert any("lighting" in w for w in warnings)


def test_a_short_session_warns():
    _e, warnings, _s = check(rows({}, {}), None)
    assert any("planned %d" % FRAMES_PER_SESSION in w for w in warnings)


# --------------------------------------------------------------------------
# The template
# --------------------------------------------------------------------------

def test_the_template_has_every_required_column():
    with open(TEMPLATE, newline="", encoding="utf-8-sig") as fh:
        header = next(csv.reader(fh))
    assert [c for c in REQUIRED if c not in header] == []


def test_the_template_loads_and_its_example_rows_are_valid():
    data, header_errors = load(TEMPLATE)
    assert header_errors == []
    errors, _w, _s = check(data, None)
    assert errors == [], errors


def test_the_template_does_not_ship_a_nominal_marker_width():
    """The template is the first thing copied. A 100.0 in it propagates."""
    data, _e = load(TEMPLATE)
    for r in data:
        assert float(r["marker_width_mm"]) != NOMINAL_MARKER_MM


# --------------------------------------------------------------------------
# The committed print set
# --------------------------------------------------------------------------

EXPECTED_PDFS = ("aruco_4x4_100mm.pdf", "charuco_a4_landscape.pdf")


def test_the_print_set_is_committed():
    for name in EXPECTED_PDFS:
        assert (PRINT / name).exists(), "%s is missing from print/" % name


def test_the_print_set_matches_its_checksums():
    """A PDF that has been re-exported, flattened or 'optimised' is a new target.

    Scaling introduced by a well-meaning PDF tool is invisible on screen and
    fatal on paper, and there is no way to tell from the file itself. If this
    fails, regenerate rather than patch:
        python -m tools.make_print_set --out print/
    """
    listed = {}
    for line in (PRINT / "CHECKSUMS.txt").read_text().splitlines():
        if line.strip() and not line.startswith("#"):
            digest, name = line.split()
            listed[name] = digest
    assert set(listed) == set(EXPECTED_PDFS)
    for name, digest in listed.items():
        actual = hashlib.sha256((PRINT / name).read_bytes()).hexdigest()
        assert actual == digest, "%s does not match its recorded checksum" % name
