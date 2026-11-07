"""S1b: torque model, pose choice, collision check and the fit."""

import numpy as np
import pinocchio as pin
import pytest
from so101_core.safety import SafetyLayer

from so101_calib import statics as st


def test_payload_torque_matches_a_mass_added_to_the_model():
    q = np.radians([0, -30, 40, 20, 0])
    with_payload = st.Gravity(0.2).tau(q)
    # Same mass added to the last body at the tool frame, the Pinocchio way.
    arm = st.Gravity().arm
    frame = arm.model.frames[arm.fid]
    arm.model.inertias[frame.parentJoint] += pin.Inertia(0.2, frame.placement.translation, np.zeros((3, 3)))
    ref = pin.computeGeneralizedGravity(arm.model, arm.model.createData(), arm._full(q))[arm.idx]
    assert np.allclose(with_payload, ref, atol=1e-9)


def test_gravity_has_no_torque_on_pan_and_roll_at_pan_roll_zero():
    tau = st.Gravity().tau(np.radians([0, -34, 34, 34, 0]))
    assert abs(tau[0]) < 1e-4 and abs(tau[4]) < 1e-4 and tau[2] < -0.3  # the elbow holds the forearm


def test_chosen_poses_are_distinct_spread_and_collision_free():
    s, g = SafetyLayer(), st.Gravity()
    cands = st.candidate_poses(s, g)
    chosen = st.spread_poses(cands, np.radians([0, -34, 34, 34, 0]), st.N_POSES)
    assert len({tuple(c.q) for c in chosen}) == st.N_POSES
    taus = np.array([c.tau for c in chosen])
    all_taus = np.array([c.tau for c in cands])
    for k in st.GRAVITY_JOINTS:  # covers most of each joint's torque range
        assert np.ptp(taus[:, k]) > 0.8 * np.ptp(all_taus[:, k])
    coll = st.Collisions()
    assert not any(coll.hits(np.append(c.q, 0.0)) for c in chosen)


def test_route_check_finds_a_folded_wrist():
    s = SafetyLayer()
    rest = np.radians([9, -102, 96, 54, 0, -9])
    folded = np.radians([0, -80, 90, 90, 0, 0])
    path = st.route([np.radians([0, -30, 30, 30, 0, 0]), folded], rest, s, np.radians(2))
    assert st.route_problems(path, st.Collisions(), rest)


def test_fit_recovers_compliance_band_and_offset():
    rng = np.random.default_rng(0)
    tau = rng.uniform(-0.7, 0.3, 64)
    side = np.tile([1.0, -1.0], 32)
    err = 1.6 * tau + 0.25 * side + 0.05 + rng.normal(0, 0.02, 64)
    f = st.fit_joint(err, side, tau)
    assert f["c"] == pytest.approx(1.6, abs=0.05) and f["b"] == pytest.approx(0.25, abs=0.02)
    assert f["o"] == pytest.approx(0.05, abs=0.02) and f["rms_resid_deg"] < 0.03
    f0 = st.fit_joint(0.1 * side, side, None)
    assert f0["c"] is None and f0["b"] == pytest.approx(0.1)
