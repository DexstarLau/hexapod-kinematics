# Third-party material

This repository is published. Nothing in it may be someone else's copyrighted
work republished under this project's name.

## The rule

**Vendor material is referenced, never redistributed, and never copied into
source.** That covers the kit manufacturer's manuals, schematics, parts lists,
example firmware, upper-computer software and action-group files.

Concretely, none of the following is in this repository and none may be added:

- the manufacturer's manuals, assembly guide, schematic or parts spreadsheet;
- the manufacturer's example source, in whole or in adapted form;
- the action-group file, or any table reconstructed from it.

The manuals carry an explicit reservation of rights. Treat the rest the same
way whether or not it carries a notice.

## What "never copied into source" means in practice

Reading a vendor document to learn a fact is fine and is how the constant table
was built. A single number that describes the hardware — a servo's travel, a
member length, a bus rate — is a fact about a physical object and lives in
`config/hexapod.json` with its provenance recorded.

Lifting a routine, a data table, or a file is not fine, and neither is
transcribing one with the identifiers changed. If a vendor implementation was
consulted while writing something here, the work has to be independently
written from the described behaviour, and the consultation belongs in the
commit message.

## Working with vendor data anyway

Some checks need the vendor's own data — `tools/vendor_poses.py` reads the
manufacturer's action-group file to test this project's constant table against
the poses the kit actually ships. That is done without redistributing anything:

1. The file is supplied by the operator at run time via `--actions`. It is not
   in the repository and is listed in `.gitignore`.
2. What is committed is the **report**: counts, ranges, derived structure and
   residual statistics. `docs/vendor_pose_check.md` contains no pose.
3. The report records the **SHA-256** of the input, so a published result can
   be traced to the exact file that produced it without that file being
   published.
4. Anyone who owns the kit already has the file and can reproduce the report.
   Anyone who does not, cannot — which is the correct outcome.

The same pattern applies to any future vendor data set: derived statistics may
be published, the source may not.

## Reporting

If you believe something here reproduces material it should not, open an issue
and it will be removed.
