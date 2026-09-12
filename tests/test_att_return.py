"""`att_step`'s return value, D389, and the two cases `MPTASK_15` §6 clause 3 promised.

D389 amends D20 part 1 so `att_step` returns `int`: 0 when the estimate is valid,
non-zero when it is not, **at minimum when `att_init` has not been called**. `out_rpy`
is written on every call either way, three finite floats, so a caller that ignores
the return gets v1's behaviour exactly.

**A branch reached only on a failure is tested only on failures.** That is the whole
reason this file exists: the non-zero return is unreachable in normal operation, so
without a test aimed at it nothing would ever execute it.

WHY EACH TEST LOADS ITS OWN COPY OF THE LIBRARY
-----------------------------------------------
`att_core.c` holds its state in file-scope statics -- `g_ready`, `g_seeded`, the three
angles. `att_init` sets `g_ready = 1` **for the lifetime of the process**, and nothing
clears it. `dlopen` on a path already loaded returns the same handle and the same
statics, so a second `ctypes.CDLL` of the same file would NOT give a fresh estimator.

**A before-`att_init` test that shares a library with an after-`att_init` test is a
test whose result depends on collection order.** It would pass today, pass while the
file sits alone, and fail silently -- reporting 0 and calling it correct -- the day
pytest runs something else first.

So each test copies the built library to its own filename and loads that. One
estimator per test, guaranteed, no ordering assumption. The cost is a file copy.

WHAT THESE TESTS HAVE POWER OVER
  a `void` return (the binding declares `int` and a wrong signature shows up as a
  garbage return); a zero return before `att_init`; a non-zero return in steady
  operation; `out_rpy` left unwritten or non-finite on either path.

WHAT THEY HAVE NO POWER OVER
  **whether the estimate is correct.** Nothing here checks an angle against a known
  attitude. `att_bench_check.c` does that, and every result on the record is from
  synthetic input -- no IMU log in D78's form exists.

NO SKIP. No compiler fails, for `tests/test_binding.py`'s reason.
"""

import ctypes
import math
import shutil

import pytest

from sim.visualise import build_core

FINITE = "out_rpy must hold three finite floats on every call, D389"


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    try:
        return build_core(tmp_path_factory.mktemp("att"))
    except Exception as e:                       # BuildError, or no compiler
        pytest.fail("%s. This test does not skip: the non-zero branch of att_step is "
                    "unreachable in normal operation and has no other coverage." % e)


def fresh(built, tmp_path, name):
    """A library with its own statics: copy to a new path, then load THAT path."""
    copy = tmp_path / (name + built.suffix)
    shutil.copy(built, copy)
    lib = ctypes.CDLL(str(copy))
    lib.att_init.argtypes = [ctypes.c_void_p]
    lib.att_init.restype = None
    lib.att_step.argtypes = [ctypes.POINTER(ctypes.c_float)] * 2 + \
                            [ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
    lib.att_step.restype = ctypes.c_int          # D389. A void att_step returns garbage here
    return lib


class AttConfig(ctypes.Structure):
    """D388's four members, in D388's order. Values below are this test's, not proposals."""
    _fields_ = [("gravity_mps2", ctypes.c_float),
                ("tau_s", ctypes.c_float),
                ("accel_gate_mps2", ctypes.c_float),
                ("gyro_bias_dps", ctypes.c_float * 3)]


def config():
    c = AttConfig()
    c.gravity_mps2 = 9.7936                      # not 9.81: a round number would hide a units error
    c.tau_s = 0.3700
    c.accel_gate_mps2 = 1.3000
    c.gyro_bias_dps[0] = c.gyro_bias_dps[1] = c.gyro_bias_dps[2] = 0.0
    return c


def call(lib, gyro, accel, dt_ms):
    """One att_step. out_rpy starts as NaN, so 'written' is demonstrated, not assumed."""
    g = (ctypes.c_float * 3)(*gyro)
    a = (ctypes.c_float * 3)(*accel)
    out = (ctypes.c_float * 3)(float("nan"), float("nan"), float("nan"))
    rc = lib.att_step(g, a, ctypes.c_float(dt_ms), out)
    return rc, [out[i] for i in range(3)]


LEVEL = (0.0, 0.0, 9.7936)                       # accelerometer reading gravity, body level
STILL = (0.0, 0.0, 0.0)


def test_before_att_init_the_return_is_non_zero_and_out_rpy_is_still_written(built, tmp_path):
    """Case A. The estimator has never been initialised: this library was just loaded."""
    lib = fresh(built, tmp_path, "case_a")
    rc, rpy = call(lib, STILL, LEVEL, 10.0)
    assert rc != 0, "att_step must report a non-zero return before att_init, D389"
    assert all(math.isfinite(v) for v in rpy), FINITE


def test_after_att_init_the_return_is_zero_and_stays_zero(built, tmp_path):
    """Case B, and its second call: steady operation must not report a fault."""
    lib = fresh(built, tmp_path, "case_b")
    cfg = config()
    lib.att_init(ctypes.byref(cfg))

    rc, rpy = call(lib, STILL, LEVEL, 10.0)
    assert rc == 0, "att_step must return 0 once att_init has run, D389"
    assert all(math.isfinite(v) for v in rpy), FINITE

    for _ in range(50):
        rc, rpy = call(lib, (1.3, -0.7, 0.9), (0.4, -0.3, 9.7), 10.0)
        assert rc == 0, "a valid estimate must keep returning 0"
        assert all(math.isfinite(v) for v in rpy), FINITE


def test_the_two_libraries_really_are_separate_estimators(built, tmp_path):
    """The control for this file's whole method. If `fresh` handed back one shared
    library, the second load would already be initialised and return 0 -- and every
    before-att_init test above would be vacuous."""
    first = fresh(built, tmp_path, "control_1")
    cfg = config()
    first.att_init(ctypes.byref(cfg))
    assert call(first, STILL, LEVEL, 10.0)[0] == 0

    second = fresh(built, tmp_path, "control_2")
    assert call(second, STILL, LEVEL, 10.0)[0] != 0, \
        "the second library shares statics with the first: fresh() is not giving a new estimator"


def test_a_caller_that_ignores_the_return_gets_three_finite_floats_either_way(built, tmp_path):
    """D389's compatibility promise, stated in att_core.h: out_rpy is written on every
    call including the non-zero one. Asserted on both paths from the same NaN buffer."""
    before = fresh(built, tmp_path, "compat_before")
    rc_before, rpy_before = call(before, STILL, LEVEL, 10.0)

    after = fresh(built, tmp_path, "compat_after")
    cfg = config()
    after.att_init(ctypes.byref(cfg))
    rc_after, rpy_after = call(after, STILL, LEVEL, 10.0)

    assert rc_before != 0 and rc_after == 0
    for rpy in (rpy_before, rpy_after):
        assert len(rpy) == 3 and all(math.isfinite(v) for v in rpy), FINITE


def test_yaw_is_wrapped_into_the_stated_interval(built, tmp_path):
    """att_core.h says out_rpy is wrapped into (-180, +180]. The interval cannot be
    asserted on the tests above: the FIRST att_step after att_init takes the seed path
    and returns before the wrap runs, so a check there passes on a file with no wrap at
    all. This drives yaw past half a turn and asserts on the value that comes back.

    Yaw only: it is pure gyro integration with no reference (D390), so it is the one
    channel a test can drive to a known place without modelling the filter."""
    lib = fresh(built, tmp_path, "wrap")
    cfg = config()
    lib.att_init(ctypes.byref(cfg))
    call(lib, STILL, LEVEL, 10.0)                # the seed call, discarded

    rate = 197.0                                 # deg/s about body z
    for _ in range(100):                         # 100 x 10 ms = 1.0 s -> about 197 deg
        rc, rpy = call(lib, (0.0, 0.0, rate), LEVEL, 10.0)
        assert rc == 0
    unwrapped = rate * 1.0
    assert unwrapped > 180.0, "the control is vacuous unless yaw actually passes 180"
    assert -180.0 < rpy[2] <= 180.0, "out_rpy[2] must be wrapped into (-180, +180]"
    assert rpy[2] == pytest.approx(unwrapped - 360.0, abs=1.0)
