"""Load the hexapod constant table and refuse to hand out values that do not exist.

Every physical number used anywhere in this repository comes from
config/hexapod.json and passes through this module. Nothing hard-codes a
geometry, a rate, a budget or a quantisation step.

The rule this module enforces:

    A constant with status "unspecified" has no authoritative value.
    Reading it raises ConstantError. It never silently returns None,
    and it never falls back to a default.

Three constants moved in the three days before 21 August 2026. A fourth
(the coxa table) was invalidated outright. Code that reads its numbers from
one place survives that; code that bakes them in does not.
"""

import json
from pathlib import Path

# config/hexapod.json sits one level above sim/, so: sim/constants.py -> sim -> repo root
CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "hexapod.json"

REQUIRED_FIELDS = ("value", "unit", "status", "source")
VALID_STATUS = ("decided", "measured", "provisional", "surrogate", "unspecified")


class ConstantError(Exception):
    """Raised when a constant is missing, malformed, or deliberately unspecified."""


class Constants:
    """A read-only view over the constant table.

    Use .value(name) to get the number. Use .entry(name) to get the number
    together with its unit, status and provenance.
    """

    def __init__(self, table):
        self._table = table
        self._surrogates_read = set()

    def names(self):
        """Every constant name, sorted. Metadata keys starting with '_' are excluded."""
        return sorted(k for k in self._table if not k.startswith("_"))

    def entry(self, name):
        """The full record for one constant: value, unit, status, source, note."""
        if name not in self._table or name.startswith("_"):
            raise ConstantError(
                "No constant named '{}'. Known names: {}".format(name, ", ".join(self.names()))
            )
        return self._table[name]

    def status(self, name):
        return self.entry(name)["status"]

    def value(self, name):
        """The value. Raises if the constant has no authoritative value yet."""
        entry = self.entry(name)
        if entry["status"] == "unspecified":
            raise ConstantError(
                "Constant '{}' is unspecified and must not be guessed.\n"
                "  source: {}\n"
                "  note:   {}".format(name, entry["source"], entry.get("note", "-"))
            )
        if entry["status"] == "surrogate":
            self._surrogates_read.add(name)
        return entry["value"]

    def surrogates_read(self):
        """Every surrogate this object has actually handed out.

        A sweep runner calls this after producing a result table and stamps the
        names onto the output. A number that came from a surrogate is not a
        measurement and no report may present it as one.
        """
        return sorted(self._surrogates_read)

    def stamp(self):
        """One-line provenance stamp for any emitted result."""
        used = self.surrogates_read()
        if not used:
            return "no surrogate constants used"
        return "SURROGATE VALUES USED - NOT A MEASUREMENT: " + ", ".join(used)

    def __getitem__(self, name):
        return self.value(name)

    def with_status(self, status):
        """Every constant name carrying the given status."""
        return [n for n in self.names() if self._table[n]["status"] == status]


def _validate(table):
    """Reject a malformed table at load time rather than at first use."""
    for name, entry in table.items():
        if name.startswith("_"):
            continue

        if not isinstance(entry, dict):
            raise ConstantError("Constant '{}' is not an object.".format(name))

        for field in REQUIRED_FIELDS:
            if field not in entry:
                raise ConstantError("Constant '{}' is missing field '{}'.".format(name, field))

        if entry["status"] not in VALID_STATUS:
            raise ConstantError(
                "Constant '{}' has status '{}'; expected one of {}.".format(
                    name, entry["status"], VALID_STATUS
                )
            )

        if entry["status"] == "unspecified":
            if entry["value"] is not None:
                raise ConstantError(
                    "Constant '{}' is marked unspecified but carries a value.".format(name)
                )
        elif entry["value"] is None:
            raise ConstantError(
                "Constant '{}' has a null value but is not marked unspecified.".format(name)
            )


def load(path=CONFIG_PATH):
    """Read, validate and return the constant table."""
    text = Path(path).read_text(encoding="utf-8")
    table = json.loads(text)
    _validate(table)
    return Constants(table)
