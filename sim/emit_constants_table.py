"""Emit docs/constants.md from config/hexapod.json.

The README has to carry a constant table with every provisional value labelled.
Writing that table by hand creates a second source of truth that drifts the
first time a number moves - and numbers here move roughly daily. So it is
generated instead.

Run:  python -m sim.emit_constants_table
"""

import json
from pathlib import Path

from sim.constants import CONFIG_PATH, load

OUTPUT_PATH = Path(__file__).resolve().parent.parent / "docs" / "constants.md"

BADGE = {
    "decided": "decided",
    "measured": "measured",
    "provisional": "**PROVISIONAL**",
    "surrogate": "**SURROGATE**",
    "unspecified": "**BLOCKED**",
}


def format_value(value):
    if value is None:
        return "-"
    if isinstance(value, dict):
        return "; ".join("{}: {}".format(k, v) for k, v in value.items())
    if isinstance(value, float):
        # D147: 4 dp, always. Stripping trailing zeros is a precision claim
        # this project is not allowed to make.
        return "{:.4f}".format(value)
    return str(value)


def render():
    k = load()
    meta = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))["_schema"]

    lines = []
    lines.append("# Constant table")
    lines.append("")
    lines.append("**Generated from `config/hexapod.json` by `sim/emit_constants_table.py`. Do not edit by hand.**")
    lines.append("")
    lines.append("Platform: {}  ".format(meta["platform"]))
    lines.append("Precision: {}  ".format(meta["precision_convention"]))
    lines.append("Last updated: {}".format(meta["last_updated"]))
    lines.append("")

    surrogates = k.with_status("surrogate")
    if surrogates:
        lines.append("## Surrogate constants")
        lines.append("")
        lines.append("**Nothing below has been measured.** These numbers exist so that code can")
        lines.append("run. They carry no claim about the physical machine, and any result derived")
        lines.append("from one is stamped by `Constants.stamp()`.")
        lines.append("")
        for name in surrogates:
            entry = k.entry(name)
            lines.append("- **`{}`** = {} {} - {}".format(
                name, format_value(entry["value"]), entry["unit"], entry["source"]))
        lines.append("")

    blocked = k.with_status("unspecified")
    if blocked:
        lines.append("## Blocked constants")
        lines.append("")
        lines.append("These have no authoritative value. Reading one raises `ConstantError`.")
        lines.append("No code may proceed past them with a guessed number.")
        lines.append("")
        for name in blocked:
            entry = k.entry(name)
            lines.append("- **`{}`** ({}) - {}".format(name, entry["unit"], entry["source"]))
        lines.append("")

    lines.append("## All constants")
    lines.append("")
    lines.append("| Constant | Value | Unit | Status | Source |")
    lines.append("|---|---|---|---|---|")
    for name in k.names():
        entry = k.entry(name)
        lines.append("| `{}` | {} | {} | {} | {} |".format(
            name,
            format_value(entry["value"]),
            entry["unit"],
            BADGE[entry["status"]],
            entry["source"],
        ))
    lines.append("")

    notes = [(n, k.entry(n)["note"]) for n in k.names() if "note" in k.entry(n)]
    if notes:
        lines.append("## Notes")
        lines.append("")
        for name, note in notes:
            lines.append("**`{}`** - {}".format(name, note))
            lines.append("")

    return "\n".join(lines)


def main():
    OUTPUT_PATH.write_text(render(), encoding="utf-8")
    print("wrote {}".format(OUTPUT_PATH))


if __name__ == "__main__":
    main()
