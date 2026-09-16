"""Output 15, foot_dz_per_quantum_mm (D263; defined by Spider AI in COREDROP_21 §1).

Three things are checked, and each against something other than this module:
  1. the four printed register figures (D189, D230) - the linear form reproduces all
     four and the exact finite difference reproduces none, so the test can tell them
     apart;
  2. Spider AI's reference table at the measured geometry (COREDROP_21 §1.3), which
     this workstream re-derived as the second derivation;
  3. that both inputs are read by name - the value is never written into the code.
"""

import math

import pytest

from sim import constants as C
from sim import derive as D


class Table(object):
    def __init__(self, base, **kw):
        self._base, self._kw = base, kw

    def value(self, name):
        return self._kw[name] if name in self._kw else self._base.value(name)


# D189 and D230, at the old two-segment geometry: L2 = 90, theta2 = 40.
REGISTER_FIGURES = [(0.4392, 0.5285), (0.3000, 0.3610), (0.2400, 0.2888), (0.1350, 0.1624)]


def _legacy(q):
    return Table(C.load(), coxa_length_mm=50.0, femur_length_mm=90.0, tibia_length_mm=0.0,
                 theta3_deg=0.0, theta2_nom_deg=40.0, command_step_deg=q)


@pytest.mark.parametrize("q,printed", REGISTER_FIGURES)
def test_linear_form_reproduces_the_register(q, printed):
    row, _, _ = D.sweep_point(_legacy(q), 60.0, 0.5)
    assert round(row["foot_dz_per_quantum_mm@command_step_deg"], 4) == printed


@pytest.mark.parametrize("q,printed", REGISTER_FIGURES)
def test_exact_difference_does_not_reproduce_the_register(q, printed):
    """The control. If the exact difference also matched, test 1 would have no power
    to separate the two forms."""
    exact = 90.0 * (math.sin(math.radians(40.0 + q)) - math.sin(math.radians(40.0)))
    assert round(exact, 4) != printed


# COREDROP_21 §1.3, at config/hexapod.json's measured geometry. No row at 40.0000 (D260).
REFERENCE = [
    (70.0098, 111.6279, 0.2630, 1.9483),
    (75.0000, 98.8409, 0.2329, 1.7251),
    (80.0000, 85.2774, 0.2009, 1.4884),
    (85.0000, 71.0649, 0.1674, 1.2403),
    (89.0000, 59.3002, 0.1397, 1.0350),
]


@pytest.mark.parametrize("theta2,a_eff,at_step,at_accuracy", REFERENCE)
def test_reference_table_reproduces(theta2, a_eff, at_step, at_accuracy):
    k = Table(C.load(), theta2_nom_deg=theta2)
    row, d, _ = D.sweep_point(k, 60.0, 0.5)
    assert round(d.a_eff_nom_mm, 4) == a_eff
    assert round(row["foot_dz_per_quantum_mm@command_step_deg"], 4) == at_step
    assert round(row["foot_dz_per_quantum_mm@joint_accuracy_deg"], 4) == at_accuracy


def test_output15_is_row1_minus_coxa_times_the_quantum():
    """COREDROP_21 §1.4's fact for D415's override, checked rather than restated:
    output 15 is (r_nom_mm - coxa_length_mm) * q * pi/180."""
    k = C.load()
    row, _, _ = D.sweep_point(k, 60.0, 0.5)
    for q in D.QUANTUM_INPUTS:
        expected = (row["r_nom_mm"] - k.value("coxa_length_mm")) * k.value(q) * math.pi / 180.0
        assert math.isclose(row["foot_dz_per_quantum_mm@" + q], expected, rel_tol=1e-12)


@pytest.mark.parametrize("name", D.QUANTUM_INPUTS)
def test_each_input_is_read_by_name_and_scales_its_own_column_linearly(name):
    k = C.load()
    base, _, _ = D.sweep_point(k, 60.0, 0.5)
    doubled, _, _ = D.sweep_point(Table(k, **{name: 2.0 * k.value(name)}), 60.0, 0.5)
    assert math.isclose(doubled["foot_dz_per_quantum_mm@" + name],
                        2.0 * base["foot_dz_per_quantum_mm@" + name], rel_tol=1e-12)
    other = [q for q in D.QUANTUM_INPUTS if q != name][0]
    assert doubled["foot_dz_per_quantum_mm@" + other] == base["foot_dz_per_quantum_mm@" + other]


def test_expression_is_written_out_beside_the_name():
    """D415 clause 6's discipline, applied to both outputs whose expression was ever
    in question: a figure must not be quotable against a different sentence."""
    assert "a_eff_nom_mm * q_deg * pi / 180" in D.OUTPUT_EXPRESSIONS["foot_dz_per_quantum_mm"]
    assert "theta_nom_deg - theta_midswing_deg" in D.OUTPUT_EXPRESSIONS["theta_span_deg"]
