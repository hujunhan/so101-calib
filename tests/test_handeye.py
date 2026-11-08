"""S4: the hand-eye fit on views simulated with a known camera mount, board placement and joint offsets."""

import cv2
import numpy as np
import pytest

from so101_calib import charuco as ch
from so101_calib import handeye as he
from so101_calib.touch import Pointer

PTR = Pointer()
INTR = ch.Intrinsics(
    (1280, 720),
    np.array([[732.0, 0, 680.0], [0, 731.0, 340.0], [0, 0, 1.0]]),
    np.array([0.136, -0.222, 0.0, -0.001, 0.06]),
)
# camera on the gripper, looking along the gripper (-z of the gripper frame), 5 cm back, 3 cm to the side
X_TRUE = np.eye(4)
X_TRUE[:3, :3] = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
X_TRUE[:3, 3] = [0.03, 0.0, -0.05]
Y_TRUE = np.eye(4)
Y_TRUE[:3, :3] = he.rot_z(np.radians(4.0)) @ he.FLAT_R
Y_TRUE[:3, 3] = [0.32, 0.10, 0.007]
OFF = np.array([0.0, 2.0, -3.0, 2.5, 0.0])


def simulate(n: int, rng, noise_px=0.3, joint_deg=0.05):
    """Random arm poses whose camera sees most of the board; readings = true angle - offset."""
    out = []
    while len(out) < n:
        q = np.radians(rng.uniform([-35, -60, 0, 20, -90], [35, 20, 90, 100, 90]))
        frac, angle, dist = he.predicted_view(q + np.radians(OFF), X_TRUE, Y_TRUE, INTR, PTR)
        if frac < 0.9 or angle > 55 or not 0.12 < dist < 0.45:
            continue
        T = he.inv(X_TRUE) @ he.inv(PTR.gripper(q + np.radians(OFF))) @ Y_TRUE
        obj = ch.board().getChessboardCorners().astype(np.float64)
        rv, t = he.rvec_t(T)
        p, _ = cv2.projectPoints(obj, rv, t, INTR.K, INTR.D)
        img = p.reshape(-1, 2) + rng.normal(0, noise_px, (len(obj), 2))
        q_read = q + np.radians(rng.normal(0, joint_deg, 5))
        out.append(he.Sample(np.append(q_read, 0.0), ch.View(np.arange(len(obj)), img, obj)))
    return out


@pytest.fixture(scope="module")
def samples():
    return simulate(24, np.random.default_rng(0))


def test_fit_with_offsets_recovers_everything(samples):
    # 0.05 deg of joint-reading noise is ~1-2 px at the board: the floor of the fit
    f = he.fit(samples, INTR, PTR, offsets=True)
    assert f.rms_px < 2.5
    assert np.allclose(f.offsets_deg[1:4], OFF[1:4], atol=0.5)
    assert np.allclose(f.X[:3, 3], X_TRUE[:3, 3], atol=0.002)
    assert np.allclose(f.Y[:3, 3], Y_TRUE[:3, 3], atol=0.002)


def test_without_offsets_held_out_views_are_worse(samples):
    without = he.heldout_px(samples, INTR, PTR, offsets=False)
    with_ = he.heldout_px(samples, INTR, PTR, offsets=True)
    assert with_ < 2.5 and with_ < 0.7 * without


def test_zed_pose_composes():
    T_zed_board = he.se3(np.array([0.1, -0.2, 0.05]), np.array([0.05, 0.02, 0.6]))
    Z = he.zed_in_base(Y_TRUE, T_zed_board)
    assert np.allclose(Z @ T_zed_board, Y_TRUE)


def test_corners_under_the_jaws_do_not_count():
    pts = np.array([[640.0, 600.0], [640.0, 300.0], [100.0, 700.0], [300.0, 470.0]])
    assert he.under_jaws(pts).tolist() == [True, False, False, False]
