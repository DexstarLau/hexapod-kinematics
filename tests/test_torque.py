"""D208 and D209: the tripod share, the margin, and why the ceiling is not stored."""

from sim import constants as C
from sim import derive as D


def test_margin_is_ruled_and_is_a_divisor_on_demand():
    """D209. hex_config.h rejects margin_factor < 1, so it divides demand rather than
    multiplying availability."""
    k = C.load()
    assert k.status("margin_factor") == "decided"
    assert k.value("margin_factor") == 2.5


def test_a_eff_ceiling_is_computed_and_equals_the_ruled_figure():
    """D209 quotes 111.6279 mm. The figure is checked; the number is not stored."""
    k = C.load()
    assert "a_eff_max_mm" not in k.names()
    assert round(D.a_eff_max_mm(k), 4) == 111.6279


def test_the_invariant_closes_exactly_at_the_ceiling():
    """(2.15 / 3) * 11.16279 cm = 8.0000 kg*cm; x 2.5 = 20.0000 = tau_servo exactly."""
    k = C.load()
    a_eff = D.a_eff_max_mm(k)
    tripod = k.value("mass_kg") / k.value("tripod_support_legs") * a_eff / 10.0
    assert round(tripod, 4) == 8.0
    assert round(tripod * k.value("margin_factor"), 4) == k.value("tau_servo_kgcm")


def test_d188s_retired_ceiling_was_a_3x_margin_all_along():
    """D209: the project carried a 3.0000 margin while recording the margin as unwritten.
    93.0233 mm was never unmargined."""
    k = C.load()
    implied = (k.value("tau_servo_kgcm") * 10.0 * k.value("tripod_support_legs")
               / (k.value("mass_kg") * 93.0233))
    assert round(implied, 4) == 3.0


def test_single_leg_diagnostic_is_emitted_and_is_not_a_failure():
    """D208 retires the whole-mass product as a constraint. At the D209 ceiling it reads
    24.0000 kg*cm, 1.2000x stall - and the vendor's own posture exceeds the same form by
    1.3487x while walking. An inequality a working machine breaks daily is not an
    invariant."""
    k = C.load()
    _, _, trace = D.sweep_point(k, 60.0, 0.5)
    assert "tau_femur_singleleg_kgcm" in trace
    ratio = (k.value("mass_kg") * D.a_eff_max_mm(k) / 10.0
             / k.value("tau_servo_kgcm"))
    assert round(ratio, 4) == 1.2
