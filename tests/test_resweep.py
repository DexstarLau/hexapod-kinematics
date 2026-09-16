"""tools/resweep.py - the D197 re-sweep. Every figure a document quotes from it must be
reproduced by committed code (D366), so the committed CSV is regenerated here and
compared cell by cell."""

import csv
import io
import os

import pytest

from sim import constants as C
from tools import resweep as R

HERE = os.path.dirname(os.path.abspath(__file__))
COMMITTED = os.path.join(HERE, os.pardir, "reports", "d197_resweep.csv")


@pytest.fixture(scope="module")
def rows():
    return R.run(C.load())


def test_committed_csv_is_reproduced(rows, tmp_path):
    fresh = tmp_path / "resweep.csv"
    R.write_csv(rows, str(fresh))
    with open(COMMITTED, newline="") as a, open(str(fresh), newline="") as b:
        committed, regenerated = list(csv.reader(a)), list(csv.reader(b))
    assert committed == regenerated, (
        "reports/d197_resweep.csv is not what tools/resweep.py produces from the "
        "committed table. Regenerate it with `python -m tools.resweep --csv "
        "reports/d197_resweep.csv` and say why it moved.")


def test_the_d209_floor_is_derived_and_matches_the_register():
    assert round(R.posture_floor_deg(C.load(), 2.5), 4) == 70.0098


def test_no_row_at_the_quarantined_posture(rows):
    """D260: nothing derived from theta2_nom_deg = 40.0000 is quoted."""
    assert all(abs(r["theta2_nom_deg"] - 40.0) > 1e-9 for r in rows)


def test_every_posture_has_all_three_stride_kinds(rows):
    kinds = {}
    for r in rows:
        kinds.setdefault(r["posture"], set()).add(r["stride_kind"])
    assert kinds and all(len(v) == 3 for v in kinds.values())


def test_no_output_is_reported_by_number(rows):
    """D415 clause 5. A column heading that is a bare number, or 'output N', is a
    numbering this sweep has no authority to invent."""
    for key in rows[0]:
        assert not key.strip().isdigit()
        assert not key.lower().startswith("output ")
