"""Spider AI's two C bench harnesses, run as tests.

WHY THEY LIVE HERE
------------------
`COREDROP_15` §2 and `COREDROP_16` §1 deliver `gait_bench_check.c` and
`att_bench_check.c` as bench harnesses, explicitly NOT proposed for `core/src/`,
and leave it to Mini Project to decide whether they become tests and where.

**They become tests.** A harness that only runs when someone remembers to run it
is a harness that stops running. `gait_core` and `att_core` are the two newest
files in `core/` and the two with no other coverage at all: the Python side has no
mirror of either, so `tests/test_c_agreement.py` cannot reach them and neither can
anything else in `tests/`. Wrapping them here puts twenty-eight property checks on
CI across all four ubuntu/windows x 3.12/3.14 combinations instead of on one
machine on one day.

**`tests/bench/` rather than `core/src/`**, because `core/OWNERSHIP.md` and D6.1
make `core/` the algorithm workstream's and forbid I/O in it, and both harnesses
have a `main` and print. They are compiled against `core/` and never linked into
it.

**The sources are not edited.** They arrive from `COREDROP_16`'s archive at the
hashes that document records, and this file only compiles and runs them. If one
needs changing, that is a bug report to the algorithm workstream, not a patch
(`core/OWNERSHIP.md`).

WHAT THIS FILE HAS POWER OVER
  a harness that stops compiling; a harness that starts failing; and -- via the
  check counts below -- a harness that starts passing for the wrong reason, by
  running fewer checks than it used to.

WHAT IT HAS NO POWER OVER
  **whether the robot walks, or whether the filter tracks a real IMU.** Both are
  property harnesses over synthetic inputs. `COREDROP_15` §3 and `COREDROP_16` §6
  both say so plainly and this wrapper does not improve on them: the visualiser
  (`HANDOFF_31` §3 component 8, Mini Project's, not written) and a raw IMU log
  (D78, held by no role) are what would reach those questions.

NO SKIP. If no compiler is found this fails, for the reason `tests/test_binding.py`
gives: a skip would be CI green over the only coverage these two files have.
"""

import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
BENCH = ROOT / "tests" / "bench"

# [PASS] line counts as delivered, 10 September 2026, counted with a pattern that
# excludes the trailing ALL CHECKS PASSED summary. Counting bare "PASS" gives 15
# and 17 and is wrong by exactly that line -- the first version of this file did
# that and the floor test caught it on first run.
#
# COREDROP_16 section 4 describes att_bench_check as "ten groups, twenty checks"
# against 16 [PASS] lines here. Not a defect: one printed line can cover several
# cases, as gait's own "(4 cases)" and "(400 frames)" annotations show. The floor
# below tracks PRINTED LINES, which is the only thing this wrapper can see.
#
# These are a FLOOR, not an
# identity: a harness that grows checks should not fail here, but one that
# silently runs fewer has changed in a way worth being told about. A bare exit
# code cannot see that -- a harness that printed nothing and returned 0 would
# pass on exit code alone, which is the family F question asked of this wrapper.
HARNESSES = {
    "gait_bench_check": {"min_pass": 14, "source_sha_prefix": "4c6d6d48"},
    "att_bench_check": {"min_pass": 16, "source_sha_prefix": "4b0e697a"},
}


def build_and_run(name, tmp_path):
    src = BENCH / (name + ".c")
    assert src.exists(), "%s is missing from tests/bench/" % src.name

    exe = tmp_path / (name + (".exe" if os.name == "nt" else ""))
    cc = os.environ.get("CC", "gcc")
    sources = sorted(str(p) for p in (ROOT / "core" / "src").glob("*.c"))
    cmd = [cc, "-std=c99", "-Wall", "-Wextra", "-O2",
           "-I", str(ROOT / "core" / "include"), str(src), *sources,
           "-o", str(exe), "-lm"]
    try:
        build = subprocess.run(cmd, capture_output=True, text=True)
    except FileNotFoundError:
        pytest.fail(
            "no C compiler on PATH as %r. This test does not skip: these two "
            "harnesses are the only coverage gait_core and att_core have." % cc)
    assert build.returncode == 0, "%s failed to build:\n%s" % (name, build.stderr)
    assert not build.stderr.strip(), (
        "%s built with warnings, and both harnesses are delivered warning-free "
        "at -Wall -Wextra:\n%s" % (name, build.stderr))

    run = subprocess.run([str(exe)], capture_output=True, text=True)
    return run


@pytest.mark.parametrize("name", sorted(HARNESSES))
def test_the_harness_builds_and_every_check_passes(name, tmp_path):
    run = build_and_run(name, tmp_path)
    assert run.returncode == 0, (
        "%s exited %d:\n%s" % (name, run.returncode, run.stdout[-3000:]))
    assert "ALL CHECKS PASSED" in run.stdout, (
        "%s exited 0 without its own summary line, so it may not have reached "
        "the end:\n%s" % (name, run.stdout[-3000:]))
    assert "[FAIL]" not in run.stdout, (
        "%s reported a failing check while exiting 0:\n%s"
        % (name, run.stdout[-3000:]))


@pytest.mark.parametrize("name", sorted(HARNESSES))
def test_the_harness_still_runs_at_least_as_many_checks_as_delivered(name, tmp_path):
    """Exit code alone cannot see a harness that quietly stopped checking."""
    run = build_and_run(name, tmp_path)
    passes = run.stdout.count("[PASS]")
    floor = HARNESSES[name]["min_pass"]
    assert passes >= floor, (
        "%s ran %d checks; it ran %d as delivered on 10 September. Fewer checks "
        "passing is not the same as more checks passing." % (name, passes, floor))


def test_both_harnesses_are_present_and_none_has_been_added_unnoticed():
    """The set is what the two COREDROPs delivered. A third would want a decision."""
    found = sorted(p.stem for p in BENCH.glob("*_bench_check.c"))
    assert found == sorted(HARNESSES), (
        "tests/bench/ holds %s; this file knows about %s. A harness that arrives "
        "without a row here runs untracked." % (found, sorted(HARNESSES)))
