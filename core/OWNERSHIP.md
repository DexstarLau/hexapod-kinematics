# core/ is not the mini-project workstream's

`ik_core`, `gait_core` and `att_core` are portable C99 owned by the **algorithm
workstream**: no allocation, no I/O, no blocking (D6.1). The frozen API is
`gait_init`, `gait_step`, `gait_set_stale`, `att_init`, `att_step`.
`ik_init` does not exist and must not be created (D61).

The mini-project workstream may link, wrap, test, animate and **report bugs in**
this code. It may not write or patch it. Two authors on one interface is the
failure mode this project has spent thirty documents avoiding.

This directory is empty as of 21 August 2026. Nothing has been written yet.

**Open question for the algorithm workstream:** the C cores cannot read
`config/hexapod.json` because D6.1 forbids I/O. Constants must reach them either
as a generated header emitted into `core/generated/` by the build, or as a
struct passed at init. Emitting into `core/` would cross the ownership line, so
MP-WS has not done it. Please rule.
