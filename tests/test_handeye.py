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


def simulate(n: int, rng, noise_px=0.3, joint_deg=0.05, Y=Y_TRUE):
    """Random arm poses whose camera sees most of the board; readings = true angle - offset."""
    out = []
    while len(out) < n:
        q = np.radians(rng.uniform([-35, -60, 0, 20, -90], [35, 20, 90, 100, 90]))
        frac, angle, dist = he.predicted_view(q + np.radians(OFF), X_TRUE, Y, INTR, PTR)
        if frac < 0.9 or angle > 55 or not 0.12 < dist < 0.45:
            continue
        T = he.inv(X_TRUE) @ he.inv(PTR.gripper(q + np.radians(OFF))) @ Y
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
    f = he.fit(samples, INTR, PTR, he.MODELS["lift+elbow+wrist"])
    assert f.rms_px < 2.5
    assert np.allclose(f.offsets_deg[1:4], OFF[1:4], atol=0.5)
    assert np.allclose(f.X[:3, 3], X_TRUE[:3, 3], atol=0.002)
    assert np.allclose(f.Y[:3, 3], Y_TRUE[:3, 3], atol=0.002)


def test_without_offsets_held_out_views_are_worse(samples):
    without = he.heldout_px(samples, INTR, PTR, ())
    with_ = he.heldout_px(samples, INTR, PTR, he.MODELS["lift+elbow+wrist"])
    assert with_ < 2.5 and with_ < 0.7 * without


def test_zed_pose_composes():
    T_zed_board = he.se3(np.array([0.1, -0.2, 0.05]), np.array([0.05, 0.02, 0.6]))
    Z = he.zed_in_base(Y_TRUE, T_zed_board)
    assert np.allclose(Z @ T_zed_board, Y_TRUE)


def test_corners_under_the_jaws_do_not_count():
    pts = np.array([[640.0, 600.0], [640.0, 300.0], [100.0, 700.0], [300.0, 470.0]])
    assert he.under_jaws(pts).tolist() == [True, False, False, False]


def test_two_runs_share_the_camera_and_offsets():
    """Two placements of the board: one X and one set of offsets, a board per run; the offsets fitted on one
    run predict the other run (only its board refitted) at the noise level."""
    Y2 = Y_TRUE.copy()
    Y2[:3, :3] = he.rot_z(np.radians(-10.0)) @ he.FLAT_R
    Y2[:3, 3] = [0.27, -0.05, 0.007]
    run2 = simulate(16, np.random.default_rng(5), Y=Y2)
    run1 = simulate(16, np.random.default_rng(6))
    for s in run2:
        s.board = 1
    both = run1 + run2
    j = he.MODELS["lift+elbow+wrist"]
    f = he.fit(both, INTR, PTR, j)
    # 16 views per run: one run alone lands 0.7 deg off on the elbow (joint-reading noise is shared by all
    # corners of a view, so the fit's own +- 0.1 deg is too small); together within 0.75 deg
    assert np.allclose(f.offsets_deg[1:4], OFF[1:4], atol=0.75)
    assert np.allclose(f.Ys[1][:3, 3], Y2[:3, 3], atol=0.002)
    assert he.other_run_px(both, INTR, PTR, j) < 2.5 < he.other_run_px(both, INTR, PTR, ())


def test_the_simplest_good_model_is_chosen():
    from so101_calib.handeye_cli import choose

    assert choose({"none": 18.0, "elbow": 4.1, "lift+elbow": 4.05, "lift+elbow+wrist": 4.0}) == "elbow"
    assert choose({"none": 5.0, "elbow": 4.5, "lift+elbow": 4.4, "lift+elbow+wrist": 4.4}) == "none"


def test_zed_from_two_placements():
    import cv2

    zed = ch.Intrinsics((1280, 720), np.array([[535.0, 0, 640], [0, 535, 350], [0, 0, 1.0]]), np.zeros(5))
    Z = he.se3(np.radians([-150.0, 10.0, 80.0]), np.array([0.39, 0.28, 0.38]))
    Y2 = Y_TRUE.copy()
    Y2[:3, 3] = [0.25, -0.02, 0.007]
    rng = np.random.default_rng(3)
    views = []
    for Y in (Y_TRUE, Y2):
        obj = ch.board().getChessboardCorners().astype(np.float64)
        rv, t = he.rvec_t(he.inv(Z) @ Y)
        p, _ = cv2.projectPoints(obj, rv, t, zed.K, zed.D)
        img = p.reshape(-1, 2) + rng.normal(0, 0.2, (len(obj), 2))
        views.append((ch.View(np.arange(len(obj)), img, obj), Y))
    T, rms = he.zed_fit(views, zed)
    assert rms < 0.5 and np.allclose(T[:3, 3], Z[:3, 3], atol=0.002)


def test_relocate_fits_the_board_with_a_known_mount():
    # After a workspace change: the camera mount X and the joint offsets are known, the board is new.
    rng = np.random.default_rng(3)
    Y_new = np.eye(4)
    Y_new[:3, :3] = he.rot_z(np.radians(-20.0)) @ he.FLAT_R
    Y_new[:3, 3] = [0.25, -0.05, 0.011]
    views = simulate(8, rng, Y=Y_new)
    for s in views:  # model angles = readings + offsets (the joint map applied, as SafeArm reads)
        s.q = s.q.copy()
        s.q[:5] += np.radians(OFF)
    Y, rms, per = he.fit_board(views, X_TRUE, INTR, PTR)
    assert rms < 2.5 and len(per) == 8
    assert np.allclose(Y[:3, 3], Y_new[:3, 3], atol=0.002)
    assert np.degrees(np.arccos((np.trace(Y[:3, :3].T @ Y_new[:3, :3]) - 1) / 2)) < 0.3

    # A camera mount 1 cm off shows up as a much larger error.
    X_bad = X_TRUE.copy()
    X_bad[:3, 3] += [0.01, 0.0, 0.0]
    _, rms_bad, _ = he.fit_board(views, X_bad, INTR, PTR)
    assert rms_bad > 2 * rms
