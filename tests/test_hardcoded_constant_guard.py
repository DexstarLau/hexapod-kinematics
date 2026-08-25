"""The hard-coded-constant guard, now executing.

PROJECT_01 §5.3 - change L2, re-run the sweep. UNCHANGED OUTPUTS ARE A FAILURE.
PROJECT_02 §4  - adds L3 and theta_3.
PROJECT_04 §4  - adds command_step_deg, which moved from 0.4392 to 0.3000.

Until 21 August these tests reported SKIPPED, because the guard needs a sweep and
core/ was empty. The algorithm workstream's ruling that the fourteen outputs are
computed in double from the JSON - not through the C float path - is what makes
the guard runnable today: it is four lines, derive twice with two configs and
assert the outputs differ. Under the rejected header-generation option a constant
change would have been a recompile, and a test cannot vary L2 within one process.
"""

import pytest

from sim import constants as C
from sim import derive as D

STRIDE, DUTY = 60.0, 0.5

# Perturbations chosen to be physically ordinary, not extreme. A guard that only
# fires on an absurd input is not a guard.
# name -> (original, perturbed, outputs expected to stay PUT)
#
# The expected-untouched set is the real content of this guard. "Something moved"
# only catches a constant that has been baked in. The set catches the opposite
# error too: an output that moves when it has no business moving. Each set below
# is reasoned from the defining expressions, not read off a previous run.
GUARDED = {
    # mass appears in output 11 alone
    "payload_mass_kg": (2.15, 2.60, [
        n for n in ("r_nom_mm", "body_height_mm", "theta_nom_deg", "theta_extreme_deg",
                    "theta_midswing_deg", "theta_span_deg", "femur_travel_swing_deg",
                    "coxa_sweep_deg", "bob_mm", "a_eff_extreme_mm", "swing_duration_ms",
                    "cycle_duration_ms", "body_speed_mm_s")]),

    # clearance enters output 5, and 7 through it. Nothing else sees it.
    "swing_clearance_mm": (15.0, 18.0, [
        n for n in ("r_nom_mm", "body_height_mm", "theta_nom_deg", "theta_extreme_deg",
                    "theta_span_deg", "coxa_sweep_deg", "bob_mm", "a_eff_extreme_mm",
                    "tau_femur_peak_kgcm", "swing_duration_ms", "cycle_duration_ms",
                    "body_speed_mm_s")]),

    # peak rate is a pure time scaling: outputs 12, 13, 14 and no geometry
    "dtheta_peak_deg_s": (375.0, 300.0, [
        n for n in ("r_nom_mm", "body_height_mm", "theta_nom_deg", "theta_extreme_deg",
                    "theta_midswing_deg", "theta_span_deg", "femur_travel_swing_deg",
                    "coxa_sweep_deg", "bob_mm", "a_eff_extreme_mm", "tau_femur_peak_kgcm")]),

    # L1 shifts reach but not body height, not Theta0, not the mid-swing angle -
    # those three depend on R and Theta only
    "coxa_length_mm": (50.0, 62.0, ["body_height_mm", "theta_nom_deg", "theta_midswing_deg"]),

    # the members and both angles reach everything once psi is non-zero
    "femur_length_mm":     (90.0, 97.0, []),
    "tibia_length_mm":     (90.0, 78.0, []),
    "theta_3_deg":         (-30.0, -12.0, []),
    "theta_2_neutral_deg": (40.0, 46.0, []),
}

# command_step_deg is guarded by PROJECT_04 §4 but is an actuator quantisation
# and does not enter the fourteen geometric outputs. It is guarded separately
# below, against the quantisation path, so that its absence here is deliberate
# and recorded rather than an oversight.


class Table(object):
    def __init__(self, base, **override):
        self._v = dict(base)
        self._v.update(override)

    def value(self, name):
        return self._v[name]


BASE = {
    "coxa_length_mm": 50.0, "femur_length_mm": 90.0, "tibia_length_mm": 90.0,
    "theta_3_deg": -30.0, "theta_2_neutral_deg": 40.0, "swing_clearance_mm": 15.0,
    "payload_mass_kg": 2.15, "dtheta_peak_deg_s": 375.0,
    "swing_velocity_profile": "half_sine", "tripod_support_legs": 3,
    "stall_torque_kgcm": 20.0, "torque_margin": 2.5,
    "nominal_stride_mm": 60.0, "coxa_length_mm_": None,
}
del BASE["coxa_length_mm_"]


@pytest.mark.parametrize("name", sorted(GUARDED))
def test_outputs_move_when_the_constant_moves(name):
    original, perturbed, _ = GUARDED[name]
    assert BASE[name] == original, "GUARDED and BASE disagree on {}".format(name)

    before, _, _ = D.sweep_point(Table(BASE), STRIDE, DUTY)
    after, _, _ = D.sweep_point(Table(BASE, **{name: perturbed}), STRIDE, DUTY)

    moved = [n for n in D.OUTPUT_NAMES if before[n] != after[n]]
    assert moved, (
        "changing {} from {} to {} left all fourteen outputs identical. Something "
        "downstream is holding a constant it should be computing.".format(
            name, original, perturbed))


@pytest.mark.parametrize("name", sorted(GUARDED))
def test_footprint_is_exactly_what_the_expressions_predict(name):
    """Catches over-coupling as well as under-coupling: an output that moves when
    the defining expressions say it cannot is as much a bug as one that will not."""
    _, perturbed, expected_untouched = GUARDED[name]
    before, _, _ = D.sweep_point(Table(BASE), STRIDE, DUTY)
    after, _, _ = D.sweep_point(Table(BASE, **{name: perturbed}), STRIDE, DUTY)

    actual_untouched = [n for n in D.OUTPUT_NAMES if before[n] == after[n]]
    assert actual_untouched == expected_untouched, (
        "{}: expected these outputs to stay put {}, but they were {}".format(
            name, expected_untouched, actual_untouched))


def test_psi_is_non_zero_at_the_guard_configuration():
    """COREDROP_02 §4. At theta3 == 0, theta_nom_deg is LEGITIMATELY independent of L2
    because psi is identically zero - so the guard fires correctly but cannot tell a
    hard-coded constant from a genuinely insensitive output. If a future edit returns
    theta_3_deg to 0.0, fail here loudly rather than silently lose a column."""
    d = D.derive(Table(BASE))
    assert abs(d.psi_deg) > 1e-9, (
        "psi is zero at the guard configuration, so the guard cannot distinguish a "
        "baked-in constant from an output with no sensitivity to the perturbed parameter")

    k = C.load()
    assert abs(D.derive(k).psi_deg) > 1e-9, "psi is zero in config/hexapod.json"


def test_every_guarded_name_exists_in_the_real_table():
    """A typo in GUARDED would silently disarm the guard."""
    k = C.load()
    for name in list(GUARDED) + ["command_step_deg", "joint_accuracy_deg"]:
        assert name in k.names(), "GUARDED names '{}' but no such constant exists".format(name)


def test_quantisation_members_are_present_and_live():
    """PROJECT_07 §3 gives the guard a fourth member, joint_accuracy_deg. Neither it
    nor command_step_deg enters the fourteen geometric outputs, so both are guarded
    against their values until the quantisation path exists."""
    k = C.load()
    assert k.value("command_step_deg") == 0.1350
    assert k.value("joint_accuracy_deg") == 0.2400
