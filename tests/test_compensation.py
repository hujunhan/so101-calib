"""S1c: the servo model fit, the compensated commands, and the tracking metrics on a virtual servo."""

import numpy as np
import pytest
from so101_kinematics.models import PinArm

from so101_calib import compensation as cp
from so101_calib import statics as st

MODEL = cp.ServoModel(
    c=[0.0, 1.6, 1.8, 1.8, 0.0],
    b=[-0.14, -0.07, -0.07, -0.14, -0.15],
    k=[0.0, -0.7, -1.0, -0.6, 0.0],
    o=[0.0, 0.1, 0.15, 0.1, 0.0],
    runs=[],
    position_p=32,
)
HZ = 30.0


def sine_path(seconds=4.0, amp_deg=4.0, f=0.5):
    t = np.arange(0, seconds, 1 / HZ)
    q = np.tile(np.radians([0.0, -30.0, 40.0, 20.0, 0.0, 0.0]), (len(t), 1))
    q[:, :5] += np.radians(amp_deg) * np.sin(2 * np.pi * f * t)[:, None]
    return q


def test_side_follows_the_motion_and_keeps_it_at_rest():
    v = np.array([[-1.0], [-1.0], [0.0], [0.0], [1.0], [0.0]])
    side = cp.motion_side(v)
    assert side[0, 0] == pytest.approx(1.0)  # moving down: approached from above, as in S1b
    assert side[2, 0] == side[3, 0] == side[1, 0]  # at rest: the side it arrived from
    assert side[4, 0] == pytest.approx(-1.0) and side[5, 0] == side[4, 0]


def test_variants_add_their_parts_only():
    target, grav = sine_path(), st.Gravity()
    none = cp.compensate(target, HZ, MODEL, grav, cp.VARIANTS["none"])
    assert np.allclose(none, target)
    spring = cp.compensate(target, HZ, MODEL, grav, cp.VARIANTS["spring"])
    tau = np.array([grav.tau(q) for q in target[:, :5]])
    assert np.allclose(np.degrees(spring - target)[:, :5], MODEL.error_deg(tau, 0 * tau, ("spring",)))
    assert np.allclose(spring[:, 5], target[:, 5])  # the gripper is left alone


def test_fit_recovers_a_known_model():
    rng = np.random.default_rng(0)
    tau = rng.uniform(-1, 0.3, (64, 5))
    side = np.tile([1.0, -1.0], 32)
    err = MODEL.error_deg(tau, side[:, None])
    for j in range(5):
        f = st.fit_joint(err[:, j], side, tau[:, j] if j in st.GRAVITY_JOINTS else None)
        if j in st.GRAVITY_JOINTS:
            assert f["c"] == pytest.approx(MODEL.c[j]) and f["k"] == pytest.approx(MODEL.k[j])
        assert f["b"] == pytest.approx(MODEL.b[j]) and f["o"] == pytest.approx(MODEL.o[j], abs=1e-9)


def test_full_compensation_tracks_a_virtual_servo():
    """On a servo that behaves exactly like the model, the full variant should remove most of the error;
    with no compensation the error is the delay and the model error."""
    target, grav, fk = sine_path(), st.Gravity(), PinArm().fk
    out = {}
    for name in ("none", "full"):
        cmd = cp.compensate(target, HZ, MODEL, grav, cp.VARIANTS[name], lead_s=cp.SIM_DELAY_S + 0.5 / HZ)
        t_sent, t_dense, q_dense = cp.simulate(cmd, HZ, MODEL, grav)
        out[name] = cp.Run.make(fk, t_sent, target, t_dense, q_dense)
    none, full = out["none"].tracking(), out["full"].tracking()
    assert full["tool_rms_mm"] < 0.25 * none["tool_rms_mm"]
    assert np.abs(full["joint_mean_deg"]).max() < 0.1


def test_residual_delay_of_a_pure_delay():
    """Without spring and band, the uncompensated path lags by the delay plus half a tick (the held command).
    With the band it would look longer: a direction-dependent error lines up with the velocity, like a lag."""
    zero = cp.ServoModel(c=[0.0] * 5, b=[0.0] * 5, k=[0.0] * 5, o=[0.0] * 5, runs=[], position_p=32)
    target, grav = sine_path(), st.Gravity()
    t_sent, t_dense, q_dense = cp.simulate(target, HZ, zero, grav)
    run = cp.Run.make(PinArm().fk, t_sent, target, t_dense, q_dense)
    assert run.residual_delay_s() == pytest.approx(cp.SIM_DELAY_S + 0.5 / HZ, abs=0.006)


def test_model_round_trip(tmp_path):
    MODEL.save(tmp_path / "m.json")
    assert cp.ServoModel.load(tmp_path / "m.json") == MODEL
