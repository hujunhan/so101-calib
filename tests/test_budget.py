"""S5: the ZED chain (ray to plane, leave-one-placement-out) and pose planning, on synthetic data."""

import cv2
import numpy as np

from so101_calib import budget as bu
from so101_calib import charuco as ch
from so101_calib import handeye as he
from so101_calib.budget_cli import board_at, ik_tool
from so101_calib.touch import Pointer

ZED = ch.Intrinsics(
    (1280, 720),
    np.array([[535.0, 0, 640], [0, 535, 350], [0, 0, 1.0]]),
    np.array([-0.05, 0.02, 0.0, 0.0, -0.01]),
)
# the ZED as S4 found it (orthonormalised)
_R = np.array([[-0.1004, 0.8941, -0.4365], [0.9933, 0.0647, -0.096], [-0.0576, -0.4432, -0.8946]])
_U, _, _Vt = np.linalg.svd(_R)
Z_TRUE = np.eye(4)
Z_TRUE[:3, :3], Z_TRUE[:3, 3] = _U @ _Vt, [0.3859, 0.2733, 0.3867]
PLACES = [(0.28, -0.08), (0.18, -0.14), (0.30, 0.10), (0.16, 0.12), (0.34, 0.0)]


def zed_views(Y, rng, noise=0.2):
    obj = ch.board().getChessboardCorners().astype(np.float64)
    p, _ = cv2.projectPoints(obj, *he.rvec_t(he.inv(Z_TRUE) @ Y), ZED.K, ZED.D)
    return [ch.View(np.arange(len(obj)), p.reshape(-1, 2) + rng.normal(0, noise, (len(obj), 2)), obj)]


def test_ray_meets_the_plane_where_the_point_is():
    Y = board_at(0.25, 0.05)
    v = zed_views(Y, np.random.default_rng(0), noise=0.0)[0]
    err = bu.ray_plane_errors_mm([v], Y, ZED, Z_TRUE)
    assert np.abs(err).max() < 1e-6
    assert 0.5 < bu.mm_per_px(Y, ZED, Z_TRUE) < 1.5  # ~0.9 mm per px at ~50 cm


def test_leave_one_out_is_small_with_true_boards_and_grows_with_a_wrong_one():
    rng = np.random.default_rng(1)
    Ys = [board_at(x, y) for x, y in PLACES]
    views = [zed_views(Y, rng) for Y in Ys]
    _, errs = bu.zed_leave_one_out(views, Ys, ZED)
    assert max(np.sqrt((e**2).sum(1).mean()) for e in errs) < 0.5
    Ys_bad = [Y.copy() for Y in Ys]
    Ys_bad[0][:3, 3] += [0.004, 0.0, 0.0]  # the wrist chain put one board 4 mm off
    _, errs = bu.zed_leave_one_out(views, Ys_bad, ZED)
    assert np.sqrt((errs[0] ** 2).sum(1).mean()) > 3.0


def test_tool_reaches_down_above_a_board():
    ptr = Pointer()
    target = bu.board_centre(board_at(0.25, 0.0)) + [0, 0, 0.01]
    lo, hi = np.radians([-100, -95, -90, -90, -150]), np.radians([100, 95, 85, 90, 150])
    q = ik_tool(ptr, target, np.radians([0, -20, 40, 60, 0]), lo, hi)
    assert q is not None and np.linalg.norm(ptr.tip(q, bu.TOOL) - target) < 1e-3


def test_views_are_planned_for_a_suggested_placement():
    from so101_core.model import JOINTS, urdf_limits
    from so101_core.safety import SafetyLayer

    from so101_calib.handeye_cli import wrist_intrinsics

    X = np.eye(4)
    X[:3, :3] = cv2.Rodrigues(np.radians([155.5, 5.9, 1.3]))[0]
    X[:3, 3] = [-0.004, 0.071, -0.003]
    safety = SafetyLayer(limits=urdf_limits())
    rest = safety.clip(np.radians([8.6, -102.4, 89.3, 76.0, -92.5, -9.2])[: len(JOINTS)])[0]
    vp = bu.plan_views(
        board_at(0.28, -0.08),
        X,
        wrist_intrinsics(),
        Pointer(),
        safety,
        rest,
        np.zeros(5),
        np.random.default_rng(0),
    )
    assert len(vp.poses) >= 3 and not vp.problems
