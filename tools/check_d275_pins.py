"""The six D275 figures that need the vendor pose file, as a check run on request.

WHY THESE ARE NOT TESTS
-----------------------
They read the manufacturer's action-group file, which is not redistributable
(docs/THIRD_PARTY.md). No CI runner has it and neither does a fresh clone. As
tests/test_d275_residuals.py they skipped on every machine without it, which
was every machine in practice, and **a skip is CI green over something that
never ran** (D366 clause 4). Moved here on 12 September 2026.

They now run only when asked, and they **fail rather than skip** when the file is
missing or is not the file the figures were derived from.

    python -m tools.check_d275_pins --actions "path/to/your/copy.ini"

    exit 0   all six hold
    exit 1   at least one does not; each failure is printed by name
    exit 2   refused: no file, or its SHA-256 is not the one below

**The checks themselves are moved, not rewritten.** Two things around them
changed: the library is built in a temporary directory rather than at a fixed
/tmp path, which does not exist on Windows; and the last check computes table 2
directly rather than calling tools/d275_fk_residuals.main(), because main() also
rewrites the two committed CSVs in reports/ -- a check that edits the evidence it
checks is not a check.

tests/test_d275_residuals.py now tests THIS FILE's refusals and bookkeeping, on
CI, without the vendor file.
"""

import argparse
import ctypes
import hashlib
import sys
import tempfile
from pathlib import Path

# RULE section 3 and D302: identify the file by hash before any count is taken.
PINNED_SHA256 = "2d1e4365806d34aec696064731d3ac38af75fcc98102c3dc5a1a38f1c459ecc8"

IK_OK, IK_W_REFLECTED = 0, 4
IK_PROJ_NONE, IK_PROJ_RADIAL, IK_PROJ_VERTICAL = 0, 1, 2


def _tibiae(frame):
    from tools.d275_fk_residuals import LEG_CHANNELS, angle_deg
    _gid, pwm, _ms = frame
    return {lg: round(angle_deg(pwm[c["tibia"]], lg, "tibia"), 4)
            for lg, c in LEG_CHANNELS.items()}


def _wrap(d):
    """D360: angles are determined only modulo 360 and returned in (-180, +180]."""
    while d > 180.0:
        d -= 360.0
    while d <= -180.0:
        d += 360.0
    return d


def _library(tmp):
    from tools.d275_fk_residuals import bind, build, load_library
    name = "libhex_d275_pins" + (".dll" if sys.platform.startswith("win") else ".so")
    return bind(load_library(str(build(Path(tmp) / name))))


# ---------------------------------------------------------------------------
# The six checks. Each takes the 394 action poses and raises AssertionError.
# ---------------------------------------------------------------------------

def the_corpus_is_394_action_poses(action, tmp):
    assert len(action) == 394, "corpus has %d action poses, not 394" % len(action)


def twelve_distinct_tibia_angles(action, tmp):
    """D275. Twelve, not eleven and not thirteen -- the count is what makes the
    fixed-theta3 reduction quantifiable."""
    n = len({v for f in action for v in _tibiae(f).values()})
    assert n == 12, "%d distinct tibia angles, not 12" % n


def the_uniform_split_is_284_and_110(action, tmp):
    """D275. Excluding the 110 is refused: they are the widest joint combinations."""
    uniform = [f for f in action if len(set(_tibiae(f).values())) == 1]
    assert len(uniform) == 284, "%d uniform-tibia poses, not 284" % len(uniform)
    assert len(action) - len(uniform) == 110
    assert round(100 * len(uniform) / len(action), 4) == 72.0812


def the_six_leg_intersection_and_its_single_outlier(action, tmp):
    """D275. [-67.5000, +121.5000], and G0117's rear pair at 135.0000 is the only
    pose outside it. theta3_min_deg / theta3_max_deg are NOT set from this -- they
    are quarantined under D260 and this is an upper bound on any fixed choice."""
    from tools.d275_fk_residuals import LEG_CHANNELS
    per = {lg: (min(_tibiae(f)[lg] for f in action), max(_tibiae(f)[lg] for f in action))
           for lg in LEG_CHANNELS}
    lo = max(v[0] for v in per.values())
    hi = min(v[1] for v in per.values())
    assert (lo, hi) == (-67.5, 121.5), "intersection is [%.4f, %.4f]" % (lo, hi)
    outside = {f[0] for f in action if any(not lo <= v <= hi for v in _tibiae(f).values())}
    assert outside == {117}, "poses outside the intersection: %s" % sorted(outside)


def reach_sign_partitions_the_solvable_rows_exactly(action, tmp):
    """THE FINDING, as a property. D359 clause 1, D342.

        r = L1 + R*cos(theta2 + psi) < 0   <=>   IK_W_REFLECTED
        r > 0                             <=>   IK_OK

    1,066 of 1,066 and 0 of 1,298. The round trip is done here rather than through
    the tool's round_trip(), which returns no angles for a non-IK_OK status and so
    throws away exactly the angles D342 exists to produce.
    """
    from bindings.hexconfig import HexDerived
    from sim import constants as C
    from tools.d275_fk_residuals import (LEG_CHANNELS, LEG_ORDER, angle_deg,
                                         geometry_config, signed_reach_mm)

    lib = _library(tmp)
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
            residual = max(abs(_wrap(o1 - t1)), abs(_wrap(o2 - t2)))
            if r < 0.0:
                assert st == IK_W_REFLECTED, "r = %.4f is folded but the solver returned %d" % (r, st)
                n_reflected += 1
                worst_reflected = max(worst_reflected, residual)
                for mode in (IK_PROJ_RADIAL, IK_PROJ_VERTICAL):
                    m_st, m1, m2 = solve(cfg, der, t1, t2, mode)
                    assert m_st == IK_W_REFLECTED
                    assert max(abs(_wrap(m1 - t1)), abs(_wrap(m2 - t2))) < 1e-4
            elif r > 0.0:
                assert st == IK_OK, "r = %.4f is on the near side but the solver returned %d" % (r, st)
                n_ok += 1
                worst_ok = max(worst_ok, residual)
            else:
                n_err += 1
    assert (n_ok, n_reflected, n_err) == (1298, 1066, 0), \
        "partition is %d / %d / %d" % (n_ok, n_reflected, n_err)
    assert n_ok + n_reflected == 2364
    assert worst_ok < 1e-5
    assert worst_reflected < 1e-4


def the_shipped_configuration_has_no_folded_row(action, tmp):
    """At the shipped theta3_deg the fold boundary sits past every vendor pose, so
    all 1,704 table-2 rows solve as IK_OK and none is reflected. This is the check
    that goes red if the shipped theta3 ever moves past the boundary."""
    from sim import constants as C
    from tools.d275_fk_residuals import summarise, table2
    lib = _library(tmp)
    k = C.load()
    uniform = [f for f in action if len(set(_tibiae(f).values())) == 1]
    rows, _der = table2(lib, k, uniform, k.value("theta3_deg"))
    s2 = summarise(rows, "TABLE 2 (check only, no CSV written)")
    assert s2["rows"] == 1704 and s2["bad"] == 0, "table 2: %d rows, %d bad" % (s2["rows"], s2["bad"])
    assert s2["max_deg"] < 1e-5


CHECKS = (
    the_corpus_is_394_action_poses,
    twelve_distinct_tibia_angles,
    the_uniform_split_is_284_and_110,
    the_six_leg_intersection_and_its_single_outlier,
    reach_sign_partitions_the_solvable_rows_exactly,
    the_shipped_configuration_has_no_folded_row,
)


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

class Refused(Exception):
    """The input is not the pinned file. No count is taken."""


def load_action(path):
    """The 394 action poses, after the digest is checked on the raw bytes."""
    p = Path(path)
    if not p.is_file():
        raise Refused("no file at %s. Supply your own copy of the vendor action-group "
                      "file; see docs/THIRD_PARTY.md." % p)
    digest = hashlib.sha256(p.read_bytes()).hexdigest()
    if digest != PINNED_SHA256:
        raise Refused("%s has SHA-256 %s, not the pinned %s. A count taken from a "
                      "different variant is a different number, not a corrected one."
                      % (p, digest, PINNED_SHA256))
    from tools.vendor_poses import read_actions
    frames, _ = read_actions(str(p))
    return frames[1:]                         # D302: frame 0 is the all-1500 home frame


def run_checks(action, checks):
    """[(name, None)] for a pass, [(name, message)] for a failure. Runs every check."""
    results = []
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        for check in checks:
            try:
                check(action, tmp)
                results.append((check.__name__, None))
            except AssertionError as e:
                results.append((check.__name__, str(e) or "assertion failed"))
    return results


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m tools.check_d275_pins",
                                 description="Check the six D275 figures against your copy "
                                             "of the vendor action-group file.")
    ap.add_argument("--actions", required=True, help="path to your copy of the .ini file")
    a = ap.parse_args(argv)
    try:
        action = load_action(a.actions)
    except Refused as e:
        print("refused: %s" % e, file=sys.stderr)
        return 2
    results = run_checks(action, CHECKS)      # read at call time, not bound at definition
    for name, message in results:
        print("%s  %s%s" % ("PASS" if message is None else "FAIL", name,
                            "" if message is None else ": " + message))
    failed = sum(message is not None for _n, message in results)
    print("%d of %d checks hold" % (len(results) - failed, len(results)))
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
