"""The hard-coded-constant guard.

PROJECT_01 §5.3 - change L2, re-run the sweep, UNCHANGED OUTPUTS ARE A FAILURE.
PROJECT_02 §4  - adds L3 and theta_3 as guarded members.
PROJECT_04 §4  - adds command_step_deg, which moved from 0.4392 to 0.3000.

The guard needs the sweep runner to exist before it can run. It does not exist.
These tests SKIP rather than pass, so the suite never reports a guard as green
when it has not executed.
"""

import pytest

from sim import constants as C

GUARDED = [
    "femur_length_mm",     # L2   - PROJECT_01 §5.3
    "tibia_length_mm",     # L3   - PROJECT_02 §4
    "theta_3_deg",         # th3  - PROJECT_02 §4
    "command_step_deg",    #      - PROJECT_04 §4
]


def test_every_guarded_name_exists_in_the_table():
    """A typo in GUARDED would silently disarm the guard. This is the wiring check
    and it runs today, before the sweep exists."""
    k = C.load()
    for name in GUARDED:
        assert name in k.names(), "GUARDED lists '{}' but no such constant exists".format(name)


def test_every_guarded_constant_is_numeric_and_mutable():
    """The guard mutates these. A string or a null cannot be perturbed."""
    k = C.load()
    for name in GUARDED:
        assert k.status(name) != "unspecified", "{} cannot be perturbed while unspecified".format(name)
        assert isinstance(k.value(name), float), "{} is not a float".format(name)


@pytest.mark.parametrize("name", GUARDED)
def test_sweep_outputs_move_when_constant_moves(name):
    pytest.skip(
        "sweep runner does not exist yet - core/ is empty. "
        "This guard is wired but has never executed. Do not read the suite as covering it."
    )
