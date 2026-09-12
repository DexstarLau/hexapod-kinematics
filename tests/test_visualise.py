"""sim/visualise.py -- the visualiser, MP1 scope component 8.

WHAT THESE TESTS PIN, AND WHAT THEY DELIBERATELY DO NOT
-------------------------------------------------------
They pin the INSTRUMENT: that it refuses what it must refuse, that its drift
figure moves when a foot moves and stays at zero when nothing moves, that its
body path is exact on a turn, and that its report is reproducible.

They do NOT pin what the instrument currently reads off gait_core. On 12
September it reads tens of millimetres of stance-foot drift on a straight walk,
where D58 puts zero. That is a defect report to the algorithm workstream, not a
number to freeze: a test asserting today's drift would go red on the fix.

The one property of the engine asserted here is D208's: at least
tripod_support_legs feet in contact in every frame. A correct fix keeps that.

NO SKIP. No compiler fails, as tests/test_binding.py does and for its reason.

STAND-INS. Two hex_config_t fields are unspecified in config/hexapod.json.
These tests supply them the way a caller must, by name. The values are chosen
not to be round numbers (RULE: a stand-in must never be a neat number).
"""

import copy
import json
import math

import pytest

from sim import visualise as V
from sim.constants import Constants, load
from sim.derive import foot_position_body

STAND_INS = {"swing_eps_mm_s": 3.7, "stale_ramp_ms": 437.3}
FORWARD = dict(vx=83.7, vy=0.0, omega=0.0)
TURN = dict(vx=0.0, vy=0.0, omega=17.9)


@pytest.fixture(scope="module")
def lib_path(tmp_path_factory):
    try:
        return V.build_core(tmp_path_factory.mktemp("visualise"))
    except V.BuildError as e:
        pytest.fail("%s. This test does not skip: the visualiser is the only check on "
                    "whether gait_core's feet stay put." % e)


@pytest.fixture(scope="module")
def k():
    return load()


def table_copy(k):
    return copy.deepcopy(k._table)


def run(lib_path, k, cmd, seconds=4.0, supplied=None):
    return V.visualise(cmd["vx"], cmd["vy"], cmd["omega"], seconds,
                       STAND_INS if supplied is None else supplied,
                       constants=k, lib_path=lib_path)


# ---------------------------------------------------------------- refusals

def test_an_unspecified_field_that_is_not_supplied_is_refused_by_name(lib_path, k):
    t = table_copy(k)
    for name in STAND_INS:                       # force it, so this holds after the table fills
        t[name]["status"], t[name]["value"] = "unspecified", None
    with pytest.raises(V.RunRefused) as e:
        run(lib_path, Constants(t), FORWARD, supplied={})
    for name in STAND_INS:
        assert name in str(e.value)


def test_a_supplied_value_the_table_already_has_is_refused(lib_path, k):
    t = table_copy(k)
    t["swing_eps_mm_s"]["status"], t["swing_eps_mm_s"]["value"] = "decided", 4.1
    with pytest.raises(V.RunRefused) as e:
        run(lib_path, Constants(t), FORWARD, supplied=dict(STAND_INS))
    assert "swing_eps_mm_s" in str(e.value) and "Two sources" in str(e.value)


def test_a_configuration_hex_config_validate_rejects_is_refused(lib_path, k):
    bad = dict(STAND_INS, stale_ramp_ms=0.37)    # far below one swing: E_RAMP
    with pytest.raises(V.RunRefused) as e:
        run(lib_path, k, FORWARD, supplied=bad)
    assert "E_RAMP" in str(e.value)


@pytest.mark.parametrize("item", ["swing_eps_mm_s", "=3.7", "swing_eps_mm_s=abc",
                                  "swing_eps_mm_s=nan", "swing_eps_mm_s=inf"])
def test_a_malformed_supply_is_refused(item):
    with pytest.raises(V.RunRefused):
        V.parse_supply([item])


# ---------------------------------------------------------------- the engine, D208 only

@pytest.mark.parametrize("cmd", [FORWARD, TURN], ids=["forward", "turn"])
def test_every_frame_returns_zero_and_keeps_a_tripod_in_contact(lib_path, k, cmd):
    _page, r, _stamp, stopped = run(lib_path, k, cmd)
    assert stopped is None
    assert r["frames"] == int(round(4.0 * 1.0e6 / (1.0e6 / k.value("update_rate_hz"))))
    assert r["min_feet_in_contact"] >= k.value("tripod_support_legs")
    assert r["frames_below_three"] == 0


# ---------------------------------------------------------------- the instrument has power

def held_stance(lib_path, k, frames):
    """Angles for a robot told to stand still, straight from the engine."""
    lib = V.bind(lib_path)
    cfg, _ = V.config_for_run(lib, k, STAND_INS)
    angles, dt_ms, stopped = V.run_gait(lib, cfg, 0.0, 0.0, 0.0, frames)
    assert stopped is None
    return angles, dt_ms


def test_nothing_moving_reads_exactly_zero_drift(lib_path, k):
    angles, dt_ms = held_stance(lib_path, k, 40)
    r = V.analyse(angles, dt_ms, 0.0, 0.0, 0.0, V.leg_geometry(k), k.value("body_bob_budget_mm"))
    assert all(r["legs"][g]["worst_drift_mm"] == 0.0 for g in V.LEGS)
    assert all(r["legs"][g]["runs"] == 1 for g in V.LEGS)


def test_a_foot_turned_in_contact_reads_exactly_the_distance_it_moved(lib_path, k):
    """Positive control. Turn R2's coxa by a non-round angle from frame 20 on, which
    moves the foot sideways without lifting it. The drift must equal the distance
    between the two foot positions, and no other leg may read anything."""
    angles, dt_ms = held_stance(lib_path, k, 40)
    geom = V.leg_geometry(k)
    leg = V.LEGS.index("R2")
    c = V.hex_coxa(leg)
    before = V.foot_body_mm(geom, "R2", angles[0][c], angles[0][V.hex_femur(leg)])
    for a in angles[20:]:
        a[c] += 0.73
    after = V.foot_body_mm(geom, "R2", angles[20][c], angles[20][V.hex_femur(leg)])

    r = V.analyse(angles, dt_ms, 0.0, 0.0, 0.0, geom, k.value("body_bob_budget_mm"))
    expected = math.hypot(after[0] - before[0], after[1] - before[1])
    assert expected > 1.0                                    # the control is not vacuous
    assert r["legs"]["R2"]["worst_drift_mm"] == pytest.approx(expected, rel=1e-12)
    assert r["legs"]["R2"]["worst_frame"] == 21
    assert all(r["legs"][g]["worst_drift_mm"] == 0.0 for g in V.LEGS if g != "R2")


def test_a_foot_lifted_past_the_band_starts_a_new_contact_run(lib_path, k):
    """The anchor must reset on lift-off, or a foot put down somewhere new reads as
    drift from where it was before. Lift L3 well past the band for five frames."""
    angles, dt_ms = held_stance(lib_path, k, 40)
    leg = V.LEGS.index("L3")
    f = V.hex_femur(leg)
    for a in angles[10:15]:
        a[f] -= 11.3                           # z = -R*sin(theta2 + psi): smaller theta2, higher foot
    for a in angles[15:]:
        a[V.hex_coxa(leg)] += 2.9                            # and comes down elsewhere
    r = V.analyse(angles, dt_ms, 0.0, 0.0, 0.0, V.leg_geometry(k), k.value("body_bob_budget_mm"))
    assert r["legs"]["L3"]["runs"] == 2
    assert r["legs"]["L3"]["worst_drift_mm"] == 0.0


def test_the_body_path_is_exact_on_a_turn():
    """A straight-line (Euler) body step on a turning run would itself read as foot
    drift. Two hundred small exact steps must land where one big exact step does."""
    vx, vy, w, T, n = 61.3, -23.9, math.radians(29.7), 2.0, 200
    pose = (0.0, 0.0, 0.0)
    for _ in range(n):
        dx, dy, dpsi = V.twist_step(vx, vy, w, T / n)
        pose = (*V.to_world(pose, dx, dy), pose[2] + dpsi)
    dx, dy, dpsi = V.twist_step(vx, vy, w, T)
    assert pose[0] == pytest.approx(dx, abs=1e-9)
    assert pose[1] == pytest.approx(dy, abs=1e-9)
    assert pose[2] == pytest.approx(dpsi, abs=1e-12)
    euler = (vx * T, vy * T)
    assert math.hypot(euler[0] - dx, euler[1] - dy) > 10.0   # and the difference is not small


def test_the_reconstruction_does_not_read_beta_neutral(k):
    """gait_step writes kinematic angles; beta_neutral_deg is a command offset that
    ik_core never applies. Every entry is 0.0000 in the table, which hides the
    difference, so set them to non-round values: foot_body_mm must not move, and
    sim.derive.foot_position_body -- which reads commanded angles -- must."""
    t = table_copy(k)
    t["beta_neutral_deg"]["value"] = {g: v for g, v in zip(V.LEGS, (7.3, -4.1, 2.9, -7.3, 4.1, -2.9))}
    shifted = Constants(t)
    for g in V.LEGS:
        a = V.foot_body_mm(V.leg_geometry(k), g, 6.1, 37.9)
        b = V.foot_body_mm(V.leg_geometry(shifted), g, 6.1, 37.9)
        assert a == b
        assert foot_position_body(k, g, 6.1, 37.9) != foot_position_body(shifted, g, 6.1, 37.9)


# ---------------------------------------------------------------- the report

def test_the_report_is_reproducible_and_carries_its_provenance(lib_path, k):
    page1, _r, stamp, _s = run(lib_path, k, FORWARD, seconds=1.0)
    page2, _r2, _s2, _st = run(lib_path, k, FORWARD, seconds=1.0)
    assert page1 == page2
    for name, value in STAND_INS.items():
        assert "%s = %.4f" % (name, value) in page1
    for name in stamp["surrogates"] + stamp["disputed"]:
        assert name in page1
    for path, digest in V.provenance():
        assert path in page1 and digest in page1
    data = page1.split('id="gait-data">', 1)[1].split("</script>", 1)[0]
    assert json.loads(data)["frames"] == 50
