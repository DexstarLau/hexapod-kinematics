"""The Python status tables in tools/d275_fk_residuals.py against the C enums.

WHY THIS FILE EXISTS
--------------------
`IK_STATUS` and `CFG_ERR` are hand-maintained Python dictionaries that name codes
returned by `core/`.  Nothing checked them against the enums they name, and the
consequence has already happened once, in full:

    D342 appended IK_W_REFLECTED = 4 to ik_status_t.  The Python table did not
    gain the key.  status_name() was a dict .get with the code as its own
    default, so the unknown 4 came back as the integer 4, compared unequal to
    "IK_OK", and was swept into the error bucket.  The counts stayed at
    1298/1066 while meaning something entirely different, and 1,066 rows went to
    the CSV carrying a bare 4 with four empty residual columns.  Nothing went
    red.

`status_name` was then repaired to RAISE on an unnamed code.  Raising turns a
silent reclassification into a loud stop -- but only at the moment the unknown
code is actually produced, which on this tool means somebody has to run it
against a core that returns the new value.  **These tests move the failure
forward to CI**, where the enum and the table are compared directly.

THE THREE TESTS AND WHAT EACH HAS POWER OVER
---------------------------------------------
1.  status_name raises on an unknown code, on BOTH tables.  This is the guard on
    the repair itself.  It has no power over a table that is merely incomplete.
2.  Every C enum member has a row in the matching Python table.  THIS is the one
    with power over an append -- it fails the moment core/ gains a member,
    without needing the value to be produced.
3.  No Python row names a code the C enum does not declare, which catches the
    opposite drift: a table entry left behind after a member is renamed.

Test 2 is the reason the two CFG_ERR sites were routed through status_name: a
direct subscript raises KeyError with no diagnosis, so the failure said what was
missing but not why it mattered.

NO VENDOR DATA AND NO COMPILER.  The enums are parsed out of the headers as text
and the tables are imported.  So, like every file in tests/ since the six D275
checks moved to tools/check_d275_pins.py, this file does not skip, which is D366
clause 4's point: a skipping test is CI green over something that never ran.
"""

import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from tools.d275_fk_residuals import CFG_ERR, IK_STATUS, status_name  # noqa: E402

HEADERS = {
    "CFG_ERR": (ROOT / "core" / "include" / "hex_config.h", "HEX_CFG_OK", "hex_cfg_err_t"),
    "IK_STATUS": (ROOT / "core" / "include" / "ik_core.h", "IK_OK", "ik_status_t"),
}

TABLES = {"CFG_ERR": CFG_ERR, "IK_STATUS": IK_STATUS}

# A member line is the identifier at the head of it.  Comments in this project run
# to several lines and contain identifiers of their own, so the block is stripped
# of /* ... */ before anything is matched.
MEMBER = re.compile(r"^\s*([A-Z][A-Z0-9_]*)\s*(?:=\s*\d+\s*)?,?\s*$")


def parse_enum(path, first_member, typedef_name):
    """The enum members, in declaration order, from the header text.

    Values are positional: C assigns n+1 to a member with no initialiser, and every
    member here after the first is uninitialised.  The one initialiser present is
    `= 0` on the first, which is asserted rather than assumed.
    """
    text = path.read_text(encoding="utf-8")
    start = text.index(first_member)
    end = text.index(typedef_name, start)
    block = text[start:end]
    block = re.sub(r"/\*.*?\*/", "", block, flags=re.S)   # strip comments first
    block = block[: block.rindex("}")]

    members = []
    for line in block.splitlines():
        m = MEMBER.match(line)
        if m:
            members.append(m.group(1))
    assert members and members[0] == first_member, (
        "%s did not parse -- first member is %r" % (path.name, members[:1]))
    assert re.search(re.escape(first_member) + r"\s*=\s*0", text), (
        "%s does not start at 0, so positional values are not safe" % first_member)
    return members


@pytest.fixture(scope="module", params=sorted(TABLES))
def table_case(request):
    name = request.param
    path, first, typedef = HEADERS[name]
    return name, TABLES[name], parse_enum(path, first, typedef)


def test_the_enum_parsed_at_all(table_case):
    """A parse that finds nothing passes every comparison under it."""
    name, table, members = table_case
    assert len(members) >= 5, "%s: parsed only %d members -- %r" % (
        name, len(members), members)


def test_status_name_raises_on_an_unknown_code_with_a_diagnosis(table_case):
    name, table, members = table_case
    unknown = max(table) + 50
    with pytest.raises(ValueError) as exc:
        status_name(unknown, table)
    message = str(exc.value)
    assert str(unknown) in message, "%s: the message does not name the code" % name
    assert "enum" in message.lower(), "%s: the message does not say why" % name


def test_every_c_enum_member_has_a_row(table_case):
    """The append case.  Fails when core/ gains a member, before it is ever returned."""
    name, table, members = table_case
    missing = [(i, m) for i, m in enumerate(members) if i not in table]
    assert not missing, (
        "%s is behind its C enum -- %s. An appended member that this table does "
        "not name is the D342 failure again." % (name, missing))


def test_every_row_names_a_member_the_c_enum_declares(table_case):
    """The opposite drift: a row left behind after a rename or removal."""
    name, table, members = table_case
    stale = [(k, v) for k, v in sorted(table.items()) if k >= len(members)]
    assert not stale, "%s names codes the C enum does not declare -- %s" % (name, stale)

    mismatched = []
    for code, py_name in sorted(table.items()):
        c_name = members[code]
        # The Python tables drop the enum's prefix on the error members
        # (HEX_CFG_E_MEMBER -> E_MEMBER) and keep it on the others, so the C name
        # ending with the Python name is the relation that holds for all of them.
        if not c_name.endswith(py_name):
            mismatched.append((code, py_name, c_name))
    assert not mismatched, "%s: table name does not match the enum -- %s" % (
        name, mismatched)
