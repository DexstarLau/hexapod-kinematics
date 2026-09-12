"""Guards on reports/D275_vendor_pose_validation.md that need no vendor data.

WHY THIS FILE EXISTS
--------------------
Two figures in that report were wrong in ways no existing test could see, and
neither failure needed the vendor pose file to catch.

1.  THE FOLD BOUNDARY.  MPTASK_10 section 1.1 published `folds at theta2 >
    121.1461` for `theta3 = -30.0000`.  The correct value is 121.5921.  The
    published figure was `arccos(-L1/R)` from the `theta3 = 0.0000` row composed
    with `psi` from the `-30.0000` row:

        102.9918 - (-18.1543) = 121.1460624685

    exact to every printed digit, so a rounding-tolerance test would have passed
    it.  Three of the four rows were right, so a spot check would have passed it
    too.  The only check with power over this shape is one that recomputes EVERY
    row INDEPENDENTLY from its own theta3.  That is test 1.

    Note the shape rather than the number: a table whose rows are produced in a
    loop can carry one row's intermediate into the next, and the surviving rows
    look like corroboration.  Compare COREDROP_10 (4 September) section 1.1.

2.  D366 CLAUSE 2.  Table 2's count is stated as *48 distinct configurations
    across 284 poses*, never as *284*.  A bare 284 says the shipped
    configuration passed 284 independent checks; it passed 48.  That is test 2.

NEITHER TEST READS THE VENDOR FILE, so unlike the six D275 checks in
tools/check_d275_pins.py both of these RUN ON CI.  D366 clause 4 is explicit
that no vendor-dependent test is required, on the ground that a skipping test is
CI green over something that never ran.  These two are the part that can run.

Link lengths are READ FROM config/hexapod.json, never written here.  A guard that
hard-codes the constants it guards is asserting against itself.
"""

import math
import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from sim.constants import load  # noqa: E402

REPORT = ROOT / "reports" / "D275_vendor_pose_validation.md"

# Four decimal places are what the report prints, so agreement is asserted there.
PLACES = 4
TOL = 0.5e-4


@pytest.fixture(scope="module")
def report_text():
    return REPORT.read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def links():
    k = load()
    return (
        float(k.value("coxa_length_mm")),
        float(k.value("femur_length_mm")),
        float(k.value("tibia_length_mm")),
    )


def fold_boundary(theta3_deg, L1, L2, L3):
    """R, psi and the femur angle past which r = L1 + R*cos(theta2 + psi) goes negative.

    Computed from theta3 alone.  Nothing is carried between calls -- that is the
    whole point of the function existing separately from the loop that uses it.
    """
    t = math.radians(theta3_deg)
    a = L2 + L3 * math.cos(t)
    b = L3 * math.sin(t)
    R = math.hypot(a, b)
    psi = math.degrees(math.atan2(b, a))
    boundary = math.degrees(math.acos(-L1 / R)) - psi
    return R, psi, boundary


ROW = re.compile(
    r"^\|\s*\**(-?\d+\.\d{4})\**\s*\|"      # theta3
    r"\s*\**(\d+\.\d{4})\**\s*\|"           # R
    r"\s*\**(-?\d+\.\d{4})\**\s*\|"         # psi
    r"\s*\**(-?\d+\.\d{4})\**\s*\|\s*$"     # folds at theta2 >
)


def parse_boundary_table(text):
    rows = [ROW.match(line) for line in text.splitlines()]
    rows = [tuple(float(g) for g in m.groups()) for m in rows if m]
    return rows


def test_the_boundary_table_was_found_and_has_every_shipped_and_vendor_theta3(report_text):
    """A parse that finds nothing passes every assertion below it."""
    rows = parse_boundary_table(report_text)
    assert len(rows) >= 6, "section 4.1's boundary table did not parse -- %d rows" % len(rows)
    thetas = {r[0] for r in rows}
    # -30.0000 is config/hexapod.json's shipped theta3_deg and was absent until
    # this row was added; 121.5000 is the vendor's own G0001 tibia angle, where
    # 878 of the 1,066 refusals sit.
    assert -30.0 in thetas, "the shipped theta3 is missing from the table"
    assert 121.5 in thetas, "the vendor's G0001 theta3 is missing from the table"


def test_every_fold_boundary_recomputes_independently(report_text, links):
    """Each row from its own theta3.  This is what catches a carried intermediate."""
    L1, L2, L3 = links
    rows = parse_boundary_table(report_text)
    for theta3, R_pub, psi_pub, b_pub in rows:
        R, psi, b = fold_boundary(theta3, L1, L2, L3)
        assert abs(R - R_pub) < TOL, "R at theta3=%.4f: %.4f published, %.4f here" % (
            theta3, R_pub, R)
        assert abs(psi - psi_pub) < TOL, "psi at theta3=%.4f: %.4f published, %.4f here" % (
            theta3, psi_pub, psi)
        assert abs(b - b_pub) < TOL, "boundary at theta3=%.4f: %.4f published, %.4f here" % (
            theta3, b_pub, b)


def test_the_superseded_boundary_is_reproduced_as_a_composition_not_a_rounding(links):
    """The withdrawn 121.1461 is derived here, so the diagnosis is checkable.

    If this ever stops reproducing, the explanation in the report is wrong and the
    withdrawal needs a different reason -- which is a thing worth being told.
    """
    L1, L2, L3 = links
    _, _, arc_minus_psi_at_zero = fold_boundary(0.0, L1, L2, L3)
    # at theta3 = 0, psi = 0, so the boundary IS arccos(-L1/R) there
    _, psi_at_minus_30, correct = fold_boundary(-30.0, L1, L2, L3)
    composed = arc_minus_psi_at_zero - psi_at_minus_30
    assert round(composed, PLACES) == 121.1461
    assert round(correct, PLACES) == 121.5921
    # A rounding explanation would predict these two agreeing to three places.
    assert abs(composed - correct) > 0.4


def paragraphs(text):
    """(start_line, joined_text) per blank-line-separated block.

    Line-by-line was the first version and it FAILED ON FIRST RUN against a
    correctly labelled sentence that markdown had wrapped: the 48 sat on one line
    and the second 284 on the next.  A rule about statements has to be checked on
    statements, and the smallest honest unit here is the paragraph.
    """
    out, start, buf = [], 1, []
    for n, line in enumerate(text.splitlines(), 1):
        if line.strip():
            if not buf:
                start = n
            buf.append(line)
        elif buf:
            out.append((start, " ".join(buf)))
            buf = []
    if buf:
        out.append((start, " ".join(buf)))
    return out


def test_no_bare_284_characterises_what_table_2_established(report_text):
    """D366 clause 2, enforced per paragraph with the exemptions named in the open.

    WHAT THIS HAS POWER OVER: a paragraph that says the shipped configuration
    passed on 284 and never says 48.  That is the defect D366 clause 2 names.

    WHAT IT DOES NOT: a paragraph that carries the label once and then uses a bare
    284 three more times.  Stated rather than left to be discovered -- the check
    confirms the label is present, not that every sentence carries it.

    Exemptions are listed here rather than silently skipped, because a guard whose
    exclusions are invisible is a guard nobody can review.
    """
    exempt_substrings = (
        # Section 2 reproduces D275's own published figures.  284 there is the
        # figure being reproduced, not a claim about what Table 2 established.
        "poses with six tibiae all equal",
        # Operands in arithmetic.  Rewriting these breaks the sums they display;
        # the label is carried on the line that follows each block.
        "1,704 rows = 284 poses x 6 legs",
        "the 284 in the line above is what caught it",
        "from the 284 uniform poses",
    )
    offenders = []
    for n, block in paragraphs(report_text):
        if "284" not in block:
            continue
        if any(s in block for s in exempt_substrings):
            continue
        if "48" in block:
            continue
        offenders.append("line %d: %s" % (n, block[:110]))
    assert not offenders, (
        "D366 clause 2 -- 284 used without the distinct count:\n  " + "\n  ".join(offenders))


def test_the_ruled_label_appears_in_full_at_least_once(report_text):
    """The exact ruled wording, so the guard above cannot be satisfied by an accident."""
    assert "48 distinct configurations across 284" in report_text
