"""tools/check_d275_pins.py -- the six D275 figures, and why they are not tests.

UNTIL 12 SEPTEMBER THIS FILE HELD THE SIX CHECKS THEMSELVES. Each read the vendor
action-group file from a fixed path and skipped when it was absent -- on CI, on a
fresh clone, and on every machine that did not happen to hold the file. Six tests
that report SKIPPED everywhere are CI green over something that never ran, which
D366 clause 4 names and this repository's RULE forbids.

The checks moved to tools/check_d275_pins.py, unchanged, and run on request
against a copy the user supplies. What stays here is what CAN run without the
file, and it runs everywhere:

  - the tool refuses a missing file and a file that is not the pinned one, and it
    refuses BEFORE any count is taken (RULE section 3, D302);
  - the set of six checks cannot shrink or grow without this file changing;
  - a failing check is reported by name and turns the exit code non-zero, so the
    runner cannot print a pass over a failure.

WHAT THIS FILE HAS NO POWER OVER
  whether the six figures still hold. Only running the tool on the real file
  says that. docs/THIRD_PARTY.md says how to obtain it.
"""

import hashlib

import pytest

from tools import check_d275_pins as P

EXPECTED_CHECKS = [
    "the_corpus_is_394_action_poses",
    "twelve_distinct_tibia_angles",
    "the_uniform_split_is_284_and_110",
    "the_six_leg_intersection_and_its_single_outlier",
    "reach_sign_partitions_the_solvable_rows_exactly",
    "the_shipped_configuration_has_no_folded_row",
]


def test_a_missing_file_is_refused_with_exit_2(tmp_path, capsys):
    absent = tmp_path / "not_here.ini"
    assert P.main(["--actions", str(absent)]) == 2
    assert str(absent) in capsys.readouterr().err


def test_a_file_that_is_not_the_pinned_one_is_refused_before_any_count(tmp_path, capsys, monkeypatch):
    impostor = tmp_path / "copy.ini"
    impostor.write_bytes(b"[servo]\ngroup_liu_zu=G0000\n")

    def must_not_parse(*_a, **_k):
        raise AssertionError("read_actions was reached: a count was about to be taken")
    import tools.vendor_poses
    monkeypatch.setattr(tools.vendor_poses, "read_actions", must_not_parse)

    assert P.main(["--actions", str(impostor)]) == 2
    err = capsys.readouterr().err
    assert hashlib.sha256(impostor.read_bytes()).hexdigest() in err
    assert P.PINNED_SHA256 in err


def test_the_six_checks_are_exactly_the_six(capsys):
    assert [c.__name__ for c in P.CHECKS] == EXPECTED_CHECKS


def test_a_failing_check_is_named_and_the_exit_code_says_so(monkeypatch, capsys):
    def always_holds(action, tmp):
        pass

    def never_holds(action, tmp):
        raise AssertionError("deliberately false")

    monkeypatch.setattr(P, "load_action", lambda path: ["not", "a", "corpus"])
    monkeypatch.setattr(P, "CHECKS", (always_holds, never_holds))
    assert P.main(["--actions", "ignored"]) == 1
    out = capsys.readouterr().out
    assert "PASS  always_holds" in out
    assert "FAIL  never_holds: deliberately false" in out
    assert "1 of 2 checks hold" in out


def test_the_count_check_fails_on_a_corpus_of_the_wrong_size():
    """The first check has power over a short corpus. The other five need the real
    poses to fail meaningfully and are not exercised here."""
    with pytest.raises(AssertionError, match="393"):
        P.the_corpus_is_394_action_poses([None] * 393, None)
