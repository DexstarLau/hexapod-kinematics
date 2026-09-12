"""The visualiser: MP1 scope component 8. Does gait_core WALK, or only RUN?

WHY IT EXISTS
-------------
The two bench harnesses in tests/bench/ establish that gait_core RUNS: it
returns 0, writes twelve finite angles every frame and keeps its own rules.
None of that says whether a stance foot stays where it was put down while the
body moves over it. That question is not a matter of taste here: D11 rejects
foot scuff, and D58 fixes the stance path as the no-slip straight line, "0 by
construction". So it has a measurable answer, and this module measures it and
draws it.

HOW, IN FOUR STEPS
------------------
1. Build core/ and call gait_step at the configured frame period with one
   constant command (vx, vy, omega). Only the frozen API is called. Nothing
   inside gait_core is read, copied or re-derived.
2. Turn each frame's twelve angles back into foot positions in the BODY frame,
   in double, from config/hexapod.json (foot_body_mm).
3. Move the body through the WORLD frame by the commanded twist itself -- the
   motion the engine was asked for -- one exact constant-twist step per frame
   (twist_step).
4. A foot is IN CONTACT when its height above the lowest foot in the same frame
   is within body_bob_budget_mm (D11). For each unbroken run of contact frames
   the DRIFT is the largest world-frame distance from where that run began.

WHAT IT HAS POWER OVER
  a stance foot that moves in the world while it is in contact, by any amount
  above the float32 floor of the angles it is given; fewer than three feet in
  contact (D208's tripod share); a non-zero gait_step return, which stops the
  run and is reported with its frame.

WHAT IT HAS NO POWER OVER
  **whether the robot walks on a floor.** The body is moved by the command, not
  by the feet, so this is the engine's self-consistency, not dynamics. A foot
  that slides in this picture slides on a real floor or drags the body; which
  one, only a measurement settles.
  **Lift-off and touchdown frames.** A swing foot that has risen less than
  body_bob_budget_mm counts as in contact, and what it travels there is
  included in the drift. That is the price of a contact rule that does not
  assume the body is held at constant height, which D11 does not require.

TWO CHOICES, STATED
-------------------
**The body height is not assumed.** Contact is judged against the lowest foot
in the same frame, not against a fixed stance height, so an engine that bobs
the body as D11 accepts is judged by the same rule as one that does not.

**foot_body_mm does not call sim.derive.foot_position_body.** That function
reads a COMMANDED coxa angle and subtracts beta_neutral_deg. gait_step writes
KINEMATIC angles (D262), where theta1 = 0 lies along beta_mount_deg and
beta_neutral_deg is never applied (ik_core.h). Every beta_neutral_deg in the
table is 0.0000 today, so the two agree to the last bit and no test could tell
them apart on the shipped constants -- which is the reason the difference is
written down rather than left to agree by accident.

CONSTANTS WITH NO VALUE
-----------------------
gait_step needs two fields that config/hexapod.json leaves unspecified today.
Nothing here supplies them. The caller does, by name, with --supply, and every
report carries them as stand-ins. **A supplied name the table already has a
value for is refused**: two sources for one number is the defect the constant
table exists to prevent.

Standard library only, like the rest of this repository's Python.
"""

import argparse
import ctypes
import hashlib
import html
import json
import math
import os
import subprocess
import sys
import tempfile
from pathlib import Path

from bindings.hexconfig import (HexConfig, LEGS, N_JOINTS, assert_layout, describe, fill,
                                hex_coxa, hex_femur, load_library)
from sim.constants import CONFIG_PATH, ConstantError, load
from sim.derive import derive

ROOT = Path(__file__).resolve().parent.parent
CORE = ROOT / "core"

# Body outline drawn through the coxa points in this order: down the right side,
# back up the left. Drawing order only; the frozen leg order is bindings.LEGS.
OUTLINE = ("R1", "R2", "R3", "L3", "L2", "L1")


class BuildError(Exception):
    """core/ could not be compiled. Never continue past this."""


class RunRefused(Exception):
    """The run cannot start honestly. The message names the reason."""


# ---------------------------------------------------------------------------
# 1. Build and bind
# ---------------------------------------------------------------------------

def build_core(out_dir):
    """Compile core/src/*.c into a shared library in out_dir and return its path.

    Raises BuildError when no compiler is found or the build fails. It does not
    return None: a caller that could carry on without the library would be a
    visualiser that draws nothing and says nothing.
    """
    out = Path(out_dir) / ("libhex_visualise" + (".dll" if os.name == "nt" else ".so"))
    cc = os.environ.get("CC", "gcc")
    sources = sorted(str(p) for p in (CORE / "src").glob("*.c"))
    if not sources:
        raise BuildError("core/src holds no .c files")
    cmd = [cc, "-std=c99", "-Wall", "-Wextra", "-pedantic", "-O2", "-shared", "-fPIC",
           "-I", str(CORE / "include"), *sources, "-o", str(out), "-lm"]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True)
    except FileNotFoundError:
        raise BuildError("no C compiler on PATH as %r" % cc)
    if r.returncode != 0:
        raise BuildError("core/ failed to build:\n" + r.stderr)
    return out


def bind(lib_path):
    """Load the library, assert the hex_config_t layout, declare the gait calls."""
    lib = load_library(lib_path)
    assert_layout(lib)
    lib.gait_init.argtypes = [ctypes.POINTER(HexConfig)]
    lib.gait_init.restype = None
    lib.gait_step.argtypes = [ctypes.c_float] * 4 + [ctypes.POINTER(ctypes.c_float)]
    lib.gait_step.restype = ctypes.c_int
    return lib


# ---------------------------------------------------------------------------
# 2. Configuration: the table, plus named stand-ins, never a default
# ---------------------------------------------------------------------------

def config_for_run(lib, constants, supplied):
    """hex_config_t from the table, with caller-supplied values for unspecified fields.

    supplied is {name: float}. Refuses, naming the field, when:
      - a field the struct needs is unspecified and not supplied;
      - a supplied name is not a field fill() leaves unspecified;
      - hex_config_validate does not return HEX_CFG_OK;
      - swing_clearance_mm does not exceed body_bob_budget_mm, which the contact
        rule depends on.
    Returns (cfg, stamp).
    """
    cfg, surrogates, disputed, deferred = fill(constants, strict=False)

    unknown = sorted(set(supplied) - set(deferred))
    if unknown:
        raise RunRefused(
            "--supply names %s, which config/hexapod.json already gives a value or "
            "which is not an unspecified hex_config_t field. Two sources for one "
            "number are refused." % ", ".join(unknown))
    missing = sorted(set(deferred) - set(supplied))
    if missing:
        raise RunRefused(
            "config/hexapod.json leaves %s unspecified and gait_step needs them. "
            "Supply each by name, e.g. --supply %s=<value>. No default exists."
            % (", ".join(missing), missing[0]))
    for name in deferred:
        setattr(cfg, name, float(supplied[name]))

    rc = lib.hex_config_validate(ctypes.byref(cfg))
    if rc != 0:
        raise RunRefused("hex_config_validate returned %s for this configuration"
                         % describe(rc))

    band = float(constants.value("body_bob_budget_mm"))
    clearance = float(constants.value("swing_clearance_mm"))
    if not clearance > band:
        raise RunRefused(
            "swing_clearance_mm %.4f does not exceed body_bob_budget_mm %.4f, so a "
            "foot at mid-swing cannot be told from a foot in contact" % (clearance, band))

    stamp = {
        "surrogates": sorted(surrogates),
        "disputed": sorted(disputed),
        "supplied": {name: float(supplied[name]) for name in sorted(deferred)},
        "contact_band_mm": band,
    }
    return cfg, stamp


# ---------------------------------------------------------------------------
# 3. Run the engine
# ---------------------------------------------------------------------------

def run_gait(lib, cfg, vx, vy, omega_deg_s, frames):
    """Call gait_init once, then gait_step `frames` times at the configured period.

    Returns (angles, dt_ms, stopped). angles[i] is the twelve floats written by
    call i+1. On the first non-zero return the run stops, because gait_step
    promises all twelve floats only on a 0 return; stopped is then
    {"frame": i+1, "rc": rc}, otherwise None.
    """
    dt_ms = ctypes.c_float(cfg.frame_period_us / 1000.0).value   # exactly what the core receives
    lib.gait_init(ctypes.byref(cfg))
    out = (ctypes.c_float * N_JOINTS)()
    angles, stopped = [], None
    for i in range(frames):
        rc = lib.gait_step(vx, vy, omega_deg_s, dt_ms, out)
        if rc != 0:
            stopped = {"frame": i + 1, "rc": rc}
            break
        angles.append([out[j] for j in range(N_JOINTS)])
    return angles, dt_ms, stopped


# ---------------------------------------------------------------------------
# 4. Reconstruct and measure, in double
# ---------------------------------------------------------------------------

def leg_geometry(constants):
    """Per-leg numbers foot_body_mm needs, read once from the table."""
    d = derive(constants)
    pos = constants.value("coxa_positions_mm")
    mount = constants.value("beta_mount_deg")
    return {
        "L1": float(constants.value("coxa_length_mm")),
        "R": d.rigid_len_mm,
        "psi": d.psi_deg,
        "coxa": {leg: (float(pos[leg][0]), float(pos[leg][1])) for leg in LEGS},
        "mount": {leg: float(mount[leg]) for leg in LEGS},
    }


def foot_body_mm(geom, leg, theta1_deg, theta2_deg):
    """Foot position in the BODY frame from two KINEMATIC angles (D262), in double.

    Mirrors ik_fk_leg then ik_leg_to_body: reach from the coxa axis is
    L1 + R*cos(theta2 + psi), height is -R*sin(theta2 + psi), and the leg points
    along beta_mount + theta1. beta_neutral_deg does not appear -- see the module
    docstring for why this is not sim.derive.foot_position_body.
    """
    theta = math.radians(theta2_deg + geom["psi"])
    reach = geom["L1"] + geom["R"] * math.cos(theta)
    yaw = math.radians(geom["mount"][leg] + theta1_deg)
    cx, cy = geom["coxa"][leg]
    return (cx + reach * math.cos(yaw), cy + reach * math.sin(yaw), -geom["R"] * math.sin(theta))


def twist_step(vx, vy, omega_rad_s, dt_s):
    """Body displacement over dt_s under a constant body-frame twist.

    Returns (dx, dy, dpsi) in the body frame at the START of the step. Exact for
    a constant twist: the body travels an arc, and a straight-line Euler step
    would itself read as foot drift on every turning run.
    """
    a = omega_rad_s * dt_s
    if a == 0.0:
        return vx * dt_s, vy * dt_s, 0.0
    s, c1 = math.sin(a), 1.0 - math.cos(a)
    return (s * vx - c1 * vy) / omega_rad_s, (c1 * vx + s * vy) / omega_rad_s, a


def to_world(pose, bx, by):
    x, y, psi = pose
    c, s = math.cos(psi), math.sin(psi)
    return x + c * bx - s * by, y + s * bx + c * by


def analyse(angles, dt_ms, vx, vy, omega_deg_s, geom, band_mm):
    """Everything the report shows, computed from the angles and the command alone."""
    dt_s = dt_ms / 1000.0
    w = math.radians(omega_deg_s)
    pose = (0.0, 0.0, 0.0)
    poses, feet_world, coxa_world, contact, height, drift = [], [], [], [], [], []
    anchor = {leg: None for leg in LEGS}
    legs = {leg: {"contact_frames": 0, "runs": 0, "worst_drift_mm": 0.0, "worst_frame": None}
            for leg in LEGS}
    min_contact, below_three = len(LEGS), 0

    for i, a in enumerate(angles):
        dx, dy, dpsi = twist_step(vx, vy, w, dt_s)
        pose = (*to_world(pose, dx, dy), pose[2] + dpsi)
        body = {leg: foot_body_mm(geom, leg, a[hex_coxa(k)], a[hex_femur(k)])
                for k, leg in enumerate(LEGS)}
        lowest = min(p[2] for p in body.values())

        f_world, f_contact, f_height, f_drift = {}, {}, {}, {}
        for leg in LEGS:
            bx, by, bz = body[leg]
            wx, wy = to_world(pose, bx, by)
            h = bz - lowest
            on = h <= band_mm
            if on:
                if anchor[leg] is None:
                    anchor[leg] = (wx, wy)
                    legs[leg]["runs"] += 1
                legs[leg]["contact_frames"] += 1
                dist = math.hypot(wx - anchor[leg][0], wy - anchor[leg][1])
                if dist > legs[leg]["worst_drift_mm"]:
                    legs[leg]["worst_drift_mm"], legs[leg]["worst_frame"] = dist, i + 1
            else:
                anchor[leg] = None
                dist = 0.0
            f_world[leg], f_contact[leg], f_height[leg], f_drift[leg] = (wx, wy), on, h, dist

        n_on = sum(f_contact.values())
        min_contact = min(min_contact, n_on)
        below_three += n_on < 3
        poses.append(pose)
        feet_world.append(f_world)
        coxa_world.append({leg: to_world(pose, *geom["coxa"][leg]) for leg in LEGS})
        contact.append(f_contact)
        height.append(f_height)
        drift.append(f_drift)

    return {
        "frames": len(angles), "dt_ms": dt_ms, "band_mm": band_mm,
        "legs": legs, "min_feet_in_contact": min_contact if angles else 0,
        "frames_below_three": below_three,
        "poses": poses, "feet_world": feet_world, "coxa_world": coxa_world,
        "contact": contact, "height": height, "drift": drift,
    }


# ---------------------------------------------------------------------------
# 5. Provenance
# ---------------------------------------------------------------------------

def lf_sha256(path):
    """SHA-256 over LF-normalised bytes, so a Windows checkout stamps the same digest."""
    return hashlib.sha256(Path(path).read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def provenance():
    files = [CONFIG_PATH] + sorted((CORE / "src").glob("*.c")) + sorted((CORE / "include").glob("*.h"))
    return [(str(Path(f).relative_to(ROOT)).replace("\\", "/"), lf_sha256(f)) for f in files]


# ---------------------------------------------------------------------------
# 6. The report
# ---------------------------------------------------------------------------

CSS = """
:root { --paper:#e9eef0; --ink:#1d2b36; --rule:#b9c6cc; --planted:#16735f;
        --airborne:#6c5aa8; --drift:#b3362a; --quiet:#54646e; }
* { box-sizing:border-box; }
body { margin:0; background:var(--paper); color:var(--ink);
       font:17px/1.55 Charter,"Bitstream Charter","Sitka Text",Cambria,Georgia,serif; }
main { max-width:980px; margin:0 auto; padding:32px 20px 64px; }
h1 { font-size:1.9rem; line-height:1.2; margin:0 0 6px; font-weight:600; }
h2 { font-size:1.15rem; margin:40px 0 8px; font-weight:600; }
p { max-width:72ch; margin:0 0 12px; }
.lede { color:var(--quiet); }
.num { font-variant-numeric:tabular-nums; }
.warn { border-left:4px solid var(--drift); padding:8px 14px; background:#f4f6f7; max-width:72ch; }
figure { margin:18px 0 0; }
svg { display:block; width:100%; height:auto; background:#f6f8f9; border:1px solid var(--rule); }
.controls { display:flex; gap:14px; align-items:center; margin:10px 0 0; flex-wrap:wrap; }
button { font:inherit; padding:6px 16px; border:1px solid var(--ink); background:var(--ink);
         color:var(--paper); border-radius:3px; cursor:pointer; }
button:focus-visible, input:focus-visible { outline:3px solid var(--airborne); outline-offset:2px; }
input[type=range] { flex:1; min-width:180px; accent-color:var(--ink); }
.key span { display:inline-block; width:12px; height:12px; border-radius:50%; margin:0 6px 0 14px;
            vertical-align:-1px; }
table { border-collapse:collapse; margin-top:8px; font-variant-numeric:tabular-nums; }
th, td { text-align:right; padding:5px 14px; border-bottom:1px solid var(--rule); }
th:first-child, td:first-child { text-align:left; }
th { font-weight:600; }
dl { display:grid; grid-template-columns:max-content 1fr; gap:4px 18px; margin:8px 0; font-size:.92rem; }
dt { color:var(--quiet); } dd { margin:0; overflow-wrap:anywhere; }
@media (max-width:600px) { body { font-size:16px; } th, td { padding:5px 8px; } }
"""

SCRIPT = """
(function () {
  var D = JSON.parse(document.getElementById('gait-data').textContent);
  var n = D.frames, i = 0, timer = null;
  var slider = document.getElementById('frame'), label = document.getElementById('frame-label');
  var play = document.getElementById('play'), body = document.getElementById('body');
  var cursor = document.getElementById('cursor');
  function show(k) {
    i = k; slider.value = k;
    var c = D.coxa[k], f = D.feet[k], on = D.contact[k];
    body.setAttribute('points', D.outline.map(function (j) { return c[j][0] + ',' + (-c[j][1]); }).join(' '));
    for (var j = 0; j < 6; j++) {
      var line = document.getElementById('leg' + j), foot = document.getElementById('foot' + j);
      line.setAttribute('x1', c[j][0]); line.setAttribute('y1', -c[j][1]);
      line.setAttribute('x2', f[j][0]); line.setAttribute('y2', -f[j][1]);
      foot.setAttribute('cx', f[j][0]); foot.setAttribute('cy', -f[j][1]);
      foot.setAttribute('class', on[j] ? 'planted' : 'airborne');
    }
    cursor.setAttribute('x1', D.chartX(k)); cursor.setAttribute('x2', D.chartX(k));
    label.textContent = 'Frame ' + (k + 1) + ' of ' + n + ', t = ' + ((k + 1) * D.dt_ms / 1000).toFixed(2) + ' s';
  }
  D.chartX = function (k) { return D.chart_left + k * D.chart_step; };
  function stop() { clearInterval(timer); timer = null; play.textContent = 'Play'; }
  play.addEventListener('click', function () {
    if (timer) { stop(); return; }
    play.textContent = 'Pause';
    timer = setInterval(function () { show((i + 1) % n); }, D.dt_ms);
  });
  slider.addEventListener('input', function () { stop(); show(parseInt(slider.value, 10)); });
  show(0);
})();
"""


def _fmt(v, places=3):
    return ("%%.%df" % places) % v


def render_html(command, run, stamp, prov, stopped):
    """A self-contained HTML page. Deterministic: same inputs, same bytes."""
    legs = LEGS
    n = run["frames"]
    esc = html.escape

    worst_leg = max(legs, key=lambda g: run["legs"][g]["worst_drift_mm"]) if n else None
    worst = run["legs"][worst_leg]["worst_drift_mm"] if n else 0.0

    # ---- top view geometry, world frame, y flipped for SVG
    xs = [p[0] for fr in run["feet_world"] for p in fr.values()] + \
         [p[0] for fr in run["coxa_world"] for p in fr.values()]
    ys = [p[1] for fr in run["feet_world"] for p in fr.values()] + \
         [p[1] for fr in run["coxa_world"] for p in fr.values()]
    if not xs:
        xs, ys = [0.0], [0.0]
    pad = 40.0
    vb = (min(xs) - pad, -(max(ys) + pad), (max(xs) - min(xs)) + 2 * pad, (max(ys) - min(ys)) + 2 * pad)

    trails = []
    for leg in legs:
        pts = [fr[leg] for fr in run["feet_world"]]
        trails.append('<polyline class="trail" points="%s"/>' % " ".join(
            "%s,%s" % (_fmt(x, 2), _fmt(-y, 2)) for x, y in pts))
        dots = [fr[leg] for fr, on in zip(run["feet_world"], run["contact"]) if on[leg]]
        trails.append("".join('<circle class="touch" r="1.2" cx="%s" cy="%s"/>'
                              % (_fmt(x, 2), _fmt(-y, 2)) for x, y in dots))
    path = " ".join("%s,%s" % (_fmt(p[0], 2), _fmt(-p[1], 2)) for p in run["poses"])
    legs_svg = "".join('<line id="leg%d" class="leg"/><circle id="foot%d" r="6"/>' % (j, j) for j in range(6))

    top = (
        '<svg viewBox="%s %s %s %s" role="img" aria-label="Top view in the world frame: body, legs, '
        'feet, and every foot position the run produced">'
        '<style>.trail{fill:none;stroke:#b9c6cc;stroke-width:1.2}.touch{fill:#16735f;opacity:.35}'
        '.path{fill:none;stroke:#1d2b36;stroke-width:1;stroke-dasharray:4 4}'
        '#body{fill:#1d2b36;fill-opacity:.12;stroke:#1d2b36;stroke-width:2}'
        '.leg{stroke:#1d2b36;stroke-width:3;stroke-linecap:round}'
        '.planted{fill:#16735f}.airborne{fill:#6c5aa8}</style>'
        '%s<polyline class="path" points="%s"/><polygon id="body"/>%s</svg>'
        % (_fmt(vb[0], 1), _fmt(vb[1], 1), _fmt(vb[2], 1), _fmt(vb[3], 1),
           "".join(trails), path, legs_svg))

    # ---- drift chart, one row per leg
    cw, row_h, left = 960.0, 64.0, 44.0
    step = (cw - left - 16.0) / max(n - 1, 1)
    top_drift = max(worst, 1e-9)
    rows = []
    for r, leg in enumerate(legs):
        y0 = 10.0 + r * row_h
        base = y0 + row_h - 14.0
        scale = (row_h - 24.0) / top_drift
        pts = " ".join("%s,%s" % (_fmt(left + k * step, 2), _fmt(base - run["drift"][k][leg] * scale, 2))
                       for k in range(n))
        rows.append('<text x="6" y="%s" class="lbl">%s</text>'
                    '<line x1="%s" x2="%s" y1="%s" y2="%s" class="axis"/>'
                    '<polyline class="drift" points="%s"/>'
                    % (_fmt(base - 4, 1), leg, _fmt(left, 1), _fmt(cw - 16, 1), _fmt(base, 1),
                       _fmt(base, 1), pts))
    chart_h = 10.0 + len(legs) * row_h
    chart = (
        '<svg viewBox="0 0 %s %s" role="img" aria-label="World-frame drift of each foot since it '
        'last touched down, per leg, against frame">'
        '<style>.lbl{font:600 14px Charter,Georgia,serif;fill:#1d2b36}.axis{stroke:#b9c6cc}'
        '.drift{fill:none;stroke:#b3362a;stroke-width:1.6}#cursor{stroke:#1d2b36;stroke-width:1}</style>'
        '%s<line id="cursor" y1="4" y2="%s"/></svg>'
        % (_fmt(cw, 0), _fmt(chart_h, 0), "".join(rows), _fmt(chart_h - 4, 0)))

    # ---- tables
    trs = "".join(
        "<tr><td>%s</td><td>%d</td><td>%d</td><td>%s</td><td>%s</td></tr>"
        % (leg, run["legs"][leg]["contact_frames"], run["legs"][leg]["runs"],
           _fmt(run["legs"][leg]["worst_drift_mm"], 4),
           run["legs"][leg]["worst_frame"] if run["legs"][leg]["worst_frame"] else "none")
        for leg in legs)

    supplied = stamp["supplied"]
    warn = ""
    if supplied:
        warn = ('<p class="warn">This run uses stand-in values that config/hexapod.json does not '
                'contain: %s. They were supplied by whoever ran it. Nothing drawn here is a '
                'measurement of the robot.</p>'
                % ", ".join("%s = %s" % (esc(k), _fmt(v, 4)) for k, v in supplied.items()))
    stop_note = ""
    if stopped:
        stop_note = ('<p class="warn">gait_step returned %d on frame %d. The run stopped there, '
                     'because the engine promises all twelve angles only on a 0 return.</p>'
                     % (stopped["rc"], stopped["frame"]))

    prov_rows = "".join("<dt>%s</dt><dd class=\"num\">%s</dd>" % (esc(f), h) for f, h in prov)
    listed = lambda xs: esc(", ".join(xs)) if xs else "none"

    data = {
        "frames": n, "dt_ms": run["dt_ms"], "outline": [LEGS.index(g) for g in OUTLINE],
        "coxa": [[[round(fr[g][0], 3), round(fr[g][1], 3)] for g in legs] for fr in run["coxa_world"]],
        "feet": [[[round(fr[g][0], 3), round(fr[g][1], 3)] for g in legs] for fr in run["feet_world"]],
        "contact": [[1 if fr[g] else 0 for g in legs] for fr in run["contact"]],
        "chart_left": left, "chart_step": round(step, 6),
    }

    headline = ("Worst stance-foot drift: %s mm, on %s" % (_fmt(worst, 1), worst_leg)) if n \
        else "No frame completed"

    page = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>gait_core walk check: vx %(vx)s, vy %(vy)s, omega %(om)s</title>
<style>%(css)s</style></head><body><main>
<h1>%(headline)s</h1>
<p class="lede">D58 puts a stance foot on the no-slip straight line, so a foot in contact should
not move in the world at all. This page drives gait_core with one constant command and shows
where its feet actually go.</p>
%(warn)s%(stop)s
<h2>Top view, world frame</h2>
<p>Dashed line: the body's commanded path. Grey: every position each foot took. Green dots: positions
while in contact. A planted foot that stays put leaves one dot per step, not a streak.</p>
<figure>%(top)s
<div class="controls"><button id="play" type="button">Play</button>
<input id="frame" type="range" min="0" max="%(last)d" value="0" aria-label="Frame">
<span id="frame-label" class="num"></span></div>
<p class="key">Foot<span style="background:#16735f"></span>in contact<span style="background:#6c5aa8"></span>in the air</p>
</figure>
<h2>Drift while in contact, per leg</h2>
<p>Distance from where the foot last touched down, while it stays in contact; zero in the air.
Rows share one vertical scale, topped by the worst value, %(worst)s mm.</p>
<figure>%(chart)s</figure>
<table><thead><tr><th>Leg</th><th>Contact frames</th><th>Contact runs</th><th>Worst drift, mm</th>
<th>At frame</th></tr></thead><tbody>%(trs)s</tbody></table>
<p>Fewest feet in contact in any frame: %(minc)d. Frames with fewer than three: %(below)d.</p>
<h2>How contact is decided</h2>
<p>A foot is in contact when it is within %(band)s mm (body_bob_budget_mm, D11) of the lowest foot
in the same frame. The body height is not assumed. A swing foot that has risen less than that
counts as in contact, so lift-off and touchdown frames add to the drift.</p>
<h2>What produced this page</h2>
<dl>
<dt>Command</dt><dd class="num">vx %(vx)s mm/s, vy %(vy)s mm/s, omega %(om)s deg/s</dd>
<dt>Frames</dt><dd class="num">%(n)d completed of %(req)d requested, %(dt)s ms each</dd>
<dt>Stand-ins supplied</dt><dd>%(sup)s</dd>
<dt>Surrogate constants read</dt><dd>%(sur)s</dd>
<dt>Disputed constants read</dt><dd>%(dis)s</dd>
%(prov)s
</dl>
<script type="application/json" id="gait-data">%(data)s</script>
<script>%(script)s</script>
</main></body></html>
""" % {
        "vx": _fmt(command["vx"], 4), "vy": _fmt(command["vy"], 4), "om": _fmt(command["omega"], 4),
        "css": CSS, "headline": esc(headline), "warn": warn, "stop": stop_note, "top": top,
        "last": max(n - 1, 0), "chart": chart, "worst": _fmt(worst, 4), "trs": trs,
        "minc": run["min_feet_in_contact"], "below": run["frames_below_three"],
        "band": _fmt(run["band_mm"], 4), "n": n, "req": command["frames"], "dt": _fmt(run["dt_ms"], 4),
        "sup": esc(", ".join("%s = %s" % (k, _fmt(v, 4)) for k, v in supplied.items())) or "none",
        "sur": listed(stamp["surrogates"]), "dis": listed(stamp["disputed"]), "prov": prov_rows,
        "data": json.dumps(data, separators=(",", ":")).replace("</", "<\\/"), "script": SCRIPT,
    }
    return page


# ---------------------------------------------------------------------------
# 7. Command line
# ---------------------------------------------------------------------------

def parse_supply(items):
    out = {}
    for item in items:
        name, sep, value = item.partition("=")
        if not sep or not name:
            raise RunRefused("--supply takes NAME=VALUE, got %r" % item)
        try:
            v = float(value)
        except ValueError:
            raise RunRefused("--supply %s: %r is not a number" % (name, value))
        if not math.isfinite(v):
            raise RunRefused("--supply %s: %r is not finite" % (name, value))
        if name in out:
            raise RunRefused("--supply names %s twice" % name)
        out[name] = v
    return out


def visualise(vx, vy, omega, seconds, supplied, constants=None, lib_path=None):
    """Run once and return (page, run, stamp, stopped). The CLI and the tests share this."""
    k = constants if constants is not None else load()
    if not seconds > 0.0:
        raise RunRefused("--seconds must be positive")
    # ignore_cleanup_errors: Windows will not delete a DLL that is still loaded, and a
    # failed cleanup of a temporary build is not a failed run.
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        lib = bind(lib_path if lib_path is not None else build_core(tmp))
        cfg, stamp = config_for_run(lib, k, supplied)
        frames = int(round(seconds * 1.0e6 / cfg.frame_period_us))
        if frames < 1:
            raise RunRefused("--seconds %.4f is shorter than one frame" % seconds)
        angles, dt_ms, stopped = run_gait(lib, cfg, vx, vy, omega, frames)
    run = analyse(angles, dt_ms, vx, vy, omega, leg_geometry(k), stamp["contact_band_mm"])
    command = {"vx": vx, "vy": vy, "omega": omega, "frames": frames}
    page = render_html(command, run, stamp, provenance(), stopped)
    return page, run, stamp, stopped


def main(argv=None):
    p = argparse.ArgumentParser(
        prog="python -m sim.visualise",
        description="Drive gait_core with one constant command and draw where its feet go.")
    p.add_argument("--vx", type=float, required=True, help="body-frame forward speed, mm/s")
    p.add_argument("--vy", type=float, required=True, help="body-frame leftward speed, mm/s")
    p.add_argument("--omega", type=float, required=True, help="yaw rate, deg/s, positive to the left")
    p.add_argument("--seconds", type=float, required=True, help="run length")
    p.add_argument("--supply", action="append", default=[], metavar="NAME=VALUE",
                   help="a value for a field config/hexapod.json leaves unspecified")
    p.add_argument("--out", required=True, help="HTML file to write")
    a = p.parse_args(argv)
    try:
        page, run, stamp, stopped = visualise(a.vx, a.vy, a.omega, a.seconds, parse_supply(a.supply))
    except (RunRefused, BuildError, ConstantError) as e:
        print("refused: %s" % e, file=sys.stderr)
        return 2
    Path(a.out).write_text(page, encoding="utf-8", newline="\n")

    print("frames %d, %.4f ms each%s" % (run["frames"], run["dt_ms"],
          "" if not stopped else "; STOPPED at frame %d, rc %d" % (stopped["frame"], stopped["rc"])))
    print("contact band %.4f mm (body_bob_budget_mm)" % run["band_mm"])
    print("leg  contact_frames  runs  worst_drift_mm")
    for leg in LEGS:
        s = run["legs"][leg]
        print("%-4s %14d %5d %15.4f" % (leg, s["contact_frames"], s["runs"], s["worst_drift_mm"]))
    print("fewest feet in contact %d; frames with fewer than three %d"
          % (run["min_feet_in_contact"], run["frames_below_three"]))
    if stamp["supplied"]:
        print("STAND-INS, not in config/hexapod.json: %s"
              % ", ".join("%s=%.4f" % kv for kv in stamp["supplied"].items()))
    print("wrote %s" % a.out)
    return 0 if not stopped else 1


if __name__ == "__main__":
    sys.exit(main())
