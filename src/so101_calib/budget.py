"""S5: how far a point picked in the ZED image is from where the arm goes, and where that error comes from.

The iPad board is put down at several places on the table. At each, the ZED sees it, and the arm looks at it
with the wrist camera from a few poses. Two independent estimates of the board in the base frame follow:

    wrist chain:  Y_p = G(q + delta) X T_cam_board      (S4's X and elbow offset, held fixed)
    ZED chain:    T_base_zed T_zed_board                 (T_base_zed fitted to the other placements)

The ZED chain is used the way the demo uses it: a ZED pixel's ray meets the board's plane, and the point is
compared with where the wrist chain puts that corner. Leaving the placement out of the ZED fit makes the
number honest. The error budget splits the pointing error into the ZED pixel, the ZED pose, the arm's
kinematics and the servos.
"""

from dataclasses import dataclass

import cv2
import numpy as np

from so101_calib import charuco as ch
from so101_calib import handeye as he
from so101_calib import statics as st
from so101_calib.touch import Pointer

# The model's tool point (MuJoCo site "gripperframe", between the jaw tips), in the gripper (wrist_roll)
# frame.
TOOL = np.array([-0.0079, -0.0002, -0.0981])
IPAD_SIZE_M = (0.25, 0.18, 0.011)  # iPad Pro 11" in its case (11 mm measured), for collision checks
IPAD_MARGIN_M = 0.015  # the board's first estimate (from the ZED) can be off by several mm


def flatten(Y: np.ndarray) -> np.ndarray:
    """The nearest board pose lying flat on the table (yaw and position kept)."""
    out = np.eye(4)
    out[:3, :3] = he.rot_z(he.flat_yaw(Y[:3, :3])) @ he.FLAT_R
    out[:3, 3] = Y[:3, 3]
    return out


def board_centre(Y: np.ndarray) -> np.ndarray:
    return Y[:3, :3] @ ch.board().getChessboardCorners().mean(0) + Y[:3, 3]


def ipad_box(Y: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The iPad as a collision box (centre, rotation, size) under the board, with a margin."""
    size = np.array(IPAD_SIZE_M) + [2 * IPAD_MARGIN_M, 2 * IPAD_MARGIN_M, 0.0]
    return board_centre(Y) - [0.0, 0.0, IPAD_SIZE_M[2] / 2], Y[:3, :3], size


def tool_clearance(q_true: np.ndarray, ptr: Pointer, z: float) -> float:
    """Height (m) above z of the lowest gripper point tracked: jaw tips and the wrist."""
    G = ptr.gripper(q_true[:5])
    pts = [TOOL, np.array([-0.025, 0.0, -0.10]), np.zeros(3)]
    return min(float((G[:3, :3] @ p + G[:3, 3])[2]) for p in pts) - z


# -- choosing wrist-camera poses for a placement -------------------------------------------------------------
@dataclass
class ViewPlan:
    poses: list[np.ndarray]  # commands (model angles, joint map applied)
    problems: list[str]
    n_candidates: int


def plan_views(
    Y: np.ndarray,
    X: np.ndarray,
    intr: ch.Intrinsics,
    ptr: Pointer,
    safety,
    q_from: np.ndarray,
    extra_off: np.ndarray,
    rng: np.random.Generator,
    n: int = 5,
    n_samples: int = 40000,
    speed_rad_s: float = np.radians(12.0),
) -> ViewPlan:
    """Arm poses whose wrist camera should see the whole board (jaws counted) from 18-35 cm, at most 40 deg
    off face-on, well clear of the iPad: random over the joint ranges, the n most different kept, in an order
    that keeps moves short, every move from q_from and back collision-checked. extra_off (5,): the zero
    offsets not yet in the joint map (true angle = command + extra_off)."""
    z = float(Y[2, 3])
    coll = st.Collisions(clearance_m=0.01, world_boxes=[ipad_box(Y)])

    def good(q) -> bool:
        if not np.allclose(safety.clip(q)[0][:5], q[:5]):
            return False
        qt = q.copy()
        qt[:5] += extra_off
        frac, angle, dist = he.predicted_view(qt, X, Y, intr, ptr)
        if frac < 0.9 or angle > 40 or not 0.18 < dist < 0.35:
            return False
        return tool_clearance(qt, ptr, z) >= 0.04 and not coll.hits(qt)

    # good poses are a thin slice of joint space: random seeds, then the neighbourhood of each seed found
    cands = []
    for _ in range(n_samples):
        q = rng.uniform(safety.lo, safety.hi)
        q[5] = q_from[5]
        if good(q):
            cands.append(q)
    for seed in list(cands)[:60]:
        for _ in range(40):
            q = seed.copy()
            q[:5] += np.radians(rng.normal(0, 10, 5))
            if good(q):
                cands.append(q)
    if len(cands) < 3:
        return ViewPlan([], [f"only {len(cands)} poses see this placement well"], len(cands))
    step = speed_rad_s / safety.cfg.rate_hz
    pool = list(cands)
    # the most different poses; a pose whose move in or out touches something leaves the pool, and the
    # choice is made again
    for _ in range(20):
        if len(pool) < 3:
            break
        poses = [pool[i] for i in he.spread(pool, X, ptr, n, q_from)]
        path = st.route([*poses, q_from], q_from, safety, step)
        bad = route_contacts(path, coll, ptr, z, extra_off, q_from)
        if not bad:
            return ViewPlan(poses, [], len(cands))
        ends = np.cumsum(
            [0] + [len(st.ramp_path(a, b, step)) for a, b in zip([q_from, *poses], poses, strict=False)]
        )
        k = min(max(int(np.searchsorted(ends, bad[0], side="right")) - 1, 0), len(poses) - 1)
        pool = [q for q in pool if q is not poses[k]]
    return ViewPlan([], ["no clean route through the views"], len(cands))


def route_contacts(path, coll, ptr, z, extra_off, rest, near_rest_rad: float = 0.35) -> list[int]:
    """Indices of commands on the path that touch something or come within 2 cm of the board's plane (the
    folded rest pose itself is in contact in the model and is skipped)."""
    out = []
    for i, q in enumerate(path):
        if np.abs(q[:5] - rest[:5]).max() <= near_rest_rad:
            continue
        qt = q.copy()
        qt[:5] += extra_off
        if coll.hits(qt) or tool_clearance(qt, ptr, z) < 0.02:
            out.append(i)
    return out


# -- the ZED chain -------------------------------------------------------------------------------------------
def rays(px: np.ndarray, intr: ch.Intrinsics) -> np.ndarray:
    """Unit-z ray directions (n, 3) in the camera frame for pixels (n, 2)."""
    u = cv2.undistortPoints(np.asarray(px, float).reshape(-1, 1, 2), intr.K, intr.D).reshape(-1, 2)
    return np.column_stack([u, np.ones(len(u))])


def ray_plane(px: np.ndarray, intr: ch.Intrinsics, T_base_cam: np.ndarray, z: float) -> np.ndarray:
    """Base-frame points (n, 3) where the pixels' rays meet the horizontal plane at height z."""
    d = (T_base_cam[:3, :3] @ rays(px, intr).T).T
    o = T_base_cam[:3, 3]
    s = (z - o[2]) / d[:, 2]
    return o + s[:, None] * d


def ray_plane_errors_mm(views: list[ch.View], Y: np.ndarray, intr, T_base_zed) -> np.ndarray:
    """For every corner the ZED saw: (ray-plane point - the corner where Y puts it), x and y in mm (n, 2)."""
    out = []
    for v in views:
        p = ray_plane(v.img, intr, T_base_zed, float(Y[2, 3]))
        true = (Y[:3, :3] @ v.obj.T).T + Y[:3, 3]
        out.append((p - true)[:, :2])
    return np.vstack(out) * 1000


def mm_per_px(Y: np.ndarray, intr, T_base_zed) -> float:
    """How far (mm) on the board's plane one ZED pixel moves the ray-plane point, at the board's centre."""
    c = board_centre(Y)
    pc = he.inv(T_base_zed) @ np.append(c, 1.0)
    px, _ = cv2.projectPoints(pc[:3].reshape(1, 3), np.zeros(3), np.zeros(3), intr.K, intr.D)
    px = px.reshape(1, 2)
    pts = ray_plane(np.vstack([px, px + [1, 0], px + [0, 1]]), intr, T_base_zed, float(Y[2, 3]))
    return float(np.mean(np.linalg.norm(pts[1:] - pts[0], axis=1)) * 1000)


def zed_leave_one_out(
    zed_views: list[list[ch.View]], Ys: list[np.ndarray], intr
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """Per placement: the ZED fitted to all the other placements, and its ray-plane errors there (mm)."""
    Ts, errs = [], []
    for p in range(len(Ys)):
        train = [(v, Ys[k]) for k in range(len(Ys)) if k != p for v in zed_views[k]]
        T, _ = he.zed_fit(train, intr)
        Ts.append(T)
        errs.append(ray_plane_errors_mm(zed_views[p], Ys[p], intr, T))
    return Ts, errs


# -- servos --------------------------------------------------------------------------------------------------
def servo_tip_mm(q_true: np.ndarray, model, grav, ptr: Pointer, compensated_deg: float = 0.15) -> tuple:
    """Tool-point error (mm, RMS) from the servos at a pose: without compensation (the S1b model's error, both
    approach sides), and with it (a residual of compensated_deg per joint, S1b's 0.08-0.21 deg RMS)."""
    tau = grav.tau(q_true[:5])
    tip = ptr.tip(q_true, TOOL)
    raw = []
    for side in (1.0, -1.0):
        e = np.radians(model.error_deg(tau, np.full(5, side)))
        q = q_true.copy()
        q[:5] -= e  # commanded q_true, the joint stops short by the error
        raw.append(np.linalg.norm(ptr.tip(q, TOOL) - tip))
    J = np.column_stack(
        [(ptr.tip(q_true + np.eye(len(q_true))[j] * 1e-4, TOOL) - tip) / 1e-4 for j in range(5)]
    )
    comp = np.sqrt(np.trace(J @ J.T)) * np.radians(compensated_deg)  # 3-D RMS for independent joint errors
    return float(np.sqrt(np.mean(np.square(raw))) * 1000), float(comp * 1000)
