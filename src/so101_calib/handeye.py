"""S4: where the cameras are. The iPad board lies still on the table; the arm moves the wrist camera over it.

For each arm pose i (joint angles q_i, read after the arm settled) the wrist camera sees board corners P_j:

    pixel_ij = project(K, D, T_cam_board_i P_j),   T_cam_board_i = X^-1 G(q_i + delta)^-1 Y

G: the gripper frame in the base (FK; the wrist_roll joint frame), X: the camera in the gripper frame,
Y: the board in the base frame (one per run: the iPad is placed anew), delta: zero offsets of a chosen set of
joints (MODELS). One least-squares fit of all pixels (reprojection error); OpenCV's closed-form hand-eye only
starts it.

The ZED sees the same board: T_base_zed = Y T_zed_board^-1.
"""

from dataclasses import dataclass

import cv2
import numpy as np
from scipy.optimize import least_squares

from so101_calib import charuco as ch
from so101_calib.touch import Pointer

# Joint offset models compared by S4 (joint indices: 1 shoulder_lift, 2 elbow_flex, 3 wrist_flex).
MODELS: dict[str, tuple[int, ...]] = {
    "none": (),
    "elbow": (2,),
    "lift+elbow": (1, 2),
    "lift+elbow+wrist": (1, 2, 3),
}


def se3(rvec: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(rvec, float).reshape(3, 1))[0]
    T[:3, 3] = t
    return T


def rvec_t(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return cv2.Rodrigues(T[:3, :3])[0].ravel(), T[:3, 3].copy()


def inv(T: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def average_pose(Ts: list[np.ndarray]) -> np.ndarray:
    """Mean of rigid transforms: rotations by SVD of their sum, translations by the mean."""
    U, _, Vt = np.linalg.svd(sum(T[:3, :3] for T in Ts))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    out = np.eye(4)
    out[:3, :3] = R
    out[:3, 3] = np.mean([T[:3, 3] for T in Ts], axis=0)
    return out


@dataclass
class Sample:
    q: np.ndarray  # (5+,) joint angles, rad (as read)
    view: ch.View  # wrist-camera detection
    board: int = 0  # which placement of the board (run)


def pnp(view: ch.View, intr: ch.Intrinsics) -> np.ndarray:
    ok, rv, tv = cv2.solvePnP(view.obj, view.img, intr.K, intr.D)
    return se3(rv.ravel(), tv.ravel())


def initial(samples: list[Sample], intr: ch.Intrinsics, ptr: Pointer) -> tuple[np.ndarray, np.ndarray]:
    """Closed-form start: X by OpenCV's hand-eye (Park), then Y as the mean of G X T_cam_board."""
    G = [ptr.gripper(s.q[:5]) for s in samples]
    C = [pnp(s.view, intr) for s in samples]
    R, t = cv2.calibrateHandEye(
        [g[:3, :3] for g in G],
        [g[:3, 3] for g in G],
        [c[:3, :3] for c in C],
        [c[:3, 3] for c in C],
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    X = np.eye(4)
    X[:3, :3], X[:3, 3] = R, t.ravel()
    Y = average_pose([g @ X @ c for g, c in zip(G, C, strict=True)])
    return X, Y


def boards(samples: list[Sample]) -> list[int]:
    return sorted({s.board for s in samples})


def initial_multi(samples: list[Sample], intr, ptr) -> tuple[np.ndarray, list[np.ndarray]]:
    """X from the board with the most views; each board then as the mean of G X T_cam_board."""
    bs = boards(samples)
    main = max(bs, key=lambda b: sum(s.board == b for s in samples))
    X, _ = initial([s for s in samples if s.board == main], intr, ptr)
    Ys = []
    for b in bs:
        sb = [s for s in samples if s.board == b]
        Ys.append(average_pose([ptr.gripper(s.q[:5]) @ X @ pnp(s.view, intr) for s in sb]))
    return X, Ys


@dataclass
class Fit:
    X: np.ndarray  # camera in gripper
    Ys: list[np.ndarray]  # board in base, per placement (sample.board)
    offsets_deg: np.ndarray  # (5,)
    offsets_std_deg: np.ndarray  # (5,)
    rms_px: float
    per_sample_px: np.ndarray

    @property
    def Y(self) -> np.ndarray:
        return self.Ys[0]


def _project(sample: Sample, X, Y, off, intr, ptr) -> np.ndarray:
    G = ptr.gripper(sample.q[:5] + off)
    T = inv(X) @ inv(G) @ Y
    rv, t = rvec_t(T)
    p, _ = cv2.projectPoints(sample.view.obj, rv, t, intr.K, intr.D)
    return p.reshape(-1, 2)


# A board lying face up on the table: its z axis (into the board) points down, -z of the base. Its rotation is
# then FLAT_R rotated about the vertical by a yaw.
FLAT_R = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def flat_yaw(R: np.ndarray) -> float:
    """The yaw that brings FLAT_R closest to R."""
    M = R @ FLAT_R.T
    return float(np.arctan2(M[1, 0] - M[0, 1], M[0, 0] + M[1, 1]))


def _flat(y) -> np.ndarray:
    Y = np.eye(4)
    Y[:3, :3], Y[:3, 3] = rot_z(y[0]) @ FLAT_R, y[1:4]
    return Y


def fit(
    samples: list[Sample],
    intr: ch.Intrinsics,
    ptr: Pointer,
    joints: tuple[int, ...] = (),
    flat: bool = True,
    start: tuple[np.ndarray, list[np.ndarray]] | None = None,
    fixed_X_off: tuple[np.ndarray, np.ndarray] | None = None,
    shared_z: bool = False,
    fixed_z: float | None = None,
) -> Fit:
    """flat: the board lies on the table the base stands on, so only its yaw and position are free (as in
    S2, this keeps a shoulder_lift offset from hiding in a tilted board). fixed_X_off: (X, offsets in rad)
    held fixed, only the boards fitted (to test X and the offsets on a run they were not fitted on).
    shared_z (flat): one height for all placements (the same iPad on the same table; S5 showed a placement
    seen from few poses drifting 8-10 mm in height otherwise); fixed_z: that height given."""
    shared_z = flat and (shared_z or fixed_z is not None)
    bs = boards(samples)
    if start is None and fixed_X_off is not None:
        # X known: each board straight from the views (no hand-eye start, which needs 3+ views)
        Xf, offf = fixed_X_off
        start = (
            Xf,
            [
                average_pose(
                    [ptr.gripper(s.q[:5] + offf) @ Xf @ pnp(s.view, intr) for s in samples if s.board == b]
                )
                for b in bs
            ],
        )
    X0, Y0s = start if start is not None else initial_multi(samples, intr, ptr)
    if fixed_X_off is not None:
        X0, joints = fixed_X_off[0], ()
    nx = 0 if fixed_X_off is not None else 6
    nj = len(joints)
    ny = (3 if shared_z else 4) if flat else 6

    def unpack(x):
        X = X0 if fixed_X_off is not None else se3(x[0:3], x[3:6])
        off = np.zeros(5) if fixed_X_off is None else fixed_X_off[1].copy()
        off[list(joints)] = x[nx : nx + nj]
        Ys = []
        for k in range(len(bs)):
            y = x[nx + nj + ny * k : nx + nj + ny * (k + 1)]
            if shared_z:
                y = np.append(y, fixed_z if fixed_z is not None else x[-1])
            Ys.append(_flat(y) if flat else se3(y[:3], y[3:]))
        return X, Ys, off

    def res(x):
        X, Ys, off = unpack(x)
        return np.concatenate(
            [(_project(s, X, Ys[bs.index(s.board)], off, intr, ptr) - s.view.img).ravel() for s in samples]
        )

    ys = [
        np.concatenate([[flat_yaw(Y[:3, :3])], Y[:3, 3]]) if flat else np.concatenate(rvec_t(Y)) for Y in Y0s
    ]
    if shared_z:
        ys = [y[:3] for y in ys] + ([[np.mean([Y[2, 3] for Y in Y0s])]] if fixed_z is None else [])
    x0 = np.concatenate(([*rvec_t(X0)] if nx else []) + [np.zeros(nj)] + ys)
    r = least_squares(res, x0, x_scale="jac", loss="soft_l1", f_scale=1.0)
    X, Ys, off = unpack(r.x)
    resid = res(r.x)
    per = []
    for s in samples:
        e = _project(s, X, Ys[bs.index(s.board)], off, intr, ptr) - s.view.img
        per.append(np.sqrt((e**2).sum(1).mean()))
    dof = max(len(resid) - len(r.x), 1)
    cov = float(resid @ resid) / dof * np.linalg.pinv(r.jac.T @ r.jac)
    off_std = np.zeros(5)
    off_std[list(joints)] = np.sqrt(np.diag(cov)[nx : nx + nj])
    # boards indexed by sample.board (missing placements get None)
    Ys_by_id = [Ys[bs.index(b)] if b in bs else None for b in range(max(bs) + 1)]
    return Fit(
        X,
        Ys_by_id,
        np.degrees(off),
        np.degrees(off_std),
        float(np.sqrt((resid**2).mean() * 2)),
        np.array(per),
    )


def heldout_px(
    samples, intr, ptr, joints: tuple[int, ...], folds: int = 4, flat: bool = True, shared_z: bool = False
) -> float:
    """RMS reprojection error of poses left out of the fit (k-fold): the fair test of whether the offsets
    help or only fit noise."""
    idx = np.arange(len(samples))
    errs = []
    for f in range(folds):
        test = idx[f::folds]
        ft = fit([samples[i] for i in np.setdiff1d(idx, test)], intr, ptr, joints, flat, shared_z=shared_z)
        off = np.radians(ft.offsets_deg)
        for i in test:
            e = _project(samples[i], ft.X, ft.Ys[samples[i].board], off, intr, ptr) - samples[i].view.img
            errs.append((e**2).sum(1).mean())
    return float(np.sqrt(np.mean(errs)))


def other_run_px(
    samples, intr, ptr, joints: tuple[int, ...], flat: bool = True, shared_z: bool = False
) -> float | None:
    """Leave one run out: X and the offsets fitted on the other runs, then on the left-out run only its board
    placement is fitted (4 numbers). RMS px over all runs; None with a single run. The strictest test:
    another day's placement of the board, poses never seen."""
    bs = boards(samples)
    if len(bs) < 2:
        return None
    errs = []
    for b in bs:
        train = [s for s in samples if s.board != b]
        test = [s for s in samples if s.board == b]
        ft = fit(train, intr, ptr, joints, flat, shared_z=shared_z)
        z = next(Y for Y in ft.Ys if Y is not None)[2, 3] if shared_z else None
        fb = fit(test, intr, ptr, flat=flat, fixed_X_off=(ft.X, np.radians(ft.offsets_deg)), fixed_z=z)
        errs.append(fb.per_sample_px**2)
    return float(np.sqrt(np.mean(np.concatenate(errs))))


def board_mm_error(fit_: Fit, samples, intr, ptr) -> np.ndarray:
    """Per sample: how far (mm) the board centre seen by the camera (PnP) lies from where the fit puts it
    (through FK): the error a grasp guided by this camera would see, in millimetres."""
    centre = ch.board().getChessboardCorners().mean(0)
    off = np.radians(fit_.offsets_deg)
    out = []
    for s in samples:
        seen = ptr.gripper(s.q[:5] + off) @ fit_.X @ pnp(s.view, intr)
        Y = fit_.Ys[s.board]
        out.append(np.linalg.norm((seen[:3, :3] @ centre + seen[:3, 3]) - (Y[:3, :3] @ centre + Y[:3, 3])))
    return np.array(out) * 1000


# -- choosing poses ------------------------------------------------------------------------------------------
# Where the gripper jaws cover the wrist image (1280 x 720): a trapezoid from the jaw tips (y 450, x 560-720)
# widening to the bottom edge (x 280-940), drawn round the jaws in the first S4 run's 24 views, with a margin.
# Corners there are not seen; the first run lost 9 of 24 views mostly to it.
JAW_TOP_Y, JAW_TOP_X, JAW_BOTTOM_X = 450.0, (560.0, 720.0), (280.0, 940.0)


def under_jaws(p: np.ndarray, size: tuple[int, int] = (1280, 720)) -> np.ndarray:
    """For image points (n, 2): inside the jaw trapezoid (scaled to the image size)."""
    sx, sy = size[0] / 1280, size[1] / 720
    x, y = p[:, 0] / sx, p[:, 1] / sy
    a = np.clip((y - JAW_TOP_Y) / (720 - JAW_TOP_Y), 0, 1)
    left = JAW_TOP_X[0] + a * (JAW_BOTTOM_X[0] - JAW_TOP_X[0])
    right = JAW_TOP_X[1] + a * (JAW_BOTTOM_X[1] - JAW_TOP_X[1])
    return (y >= JAW_TOP_Y) & (x >= left) & (x <= right)


def predicted_view(
    q, X, Y, intr: ch.Intrinsics, ptr: Pointer, margin_px: float = 40.0, jaws: bool = True
) -> tuple[float, float, float]:
    """For an arm pose: the fraction of board corners predicted inside the image (with a margin; not under
    the gripper jaws if jaws), the angle
    (deg) between the camera axis and the board normal, and the distance (m) to the board centre."""
    T = inv(X) @ inv(ptr.gripper(q[:5])) @ Y
    obj = ch.board().getChessboardCorners()
    pc = (T[:3, :3] @ obj.T).T + T[:3, 3]
    if (pc[:, 2] <= 0.02).any():
        return 0.0, 90.0, 0.0
    rv, t = rvec_t(T)
    p, _ = cv2.projectPoints(obj, rv, t, intr.K, intr.D)
    p = p.reshape(-1, 2)
    w, h = intr.size
    inside = (
        (p[:, 0] > margin_px) & (p[:, 0] < w - margin_px) & (p[:, 1] > margin_px) & (p[:, 1] < h - margin_px)
    )
    if jaws:
        inside &= ~under_jaws(p, intr.size)
    normal = T[:3, 2]
    angle = float(np.degrees(np.arccos(abs(normal[2]))))
    return float(inside.mean()), angle, float(np.linalg.norm(pc.mean(0)))


def spread(qs: list[np.ndarray], X, ptr: Pointer, n: int, first: np.ndarray) -> list[int]:
    """Pick n poses whose camera orientations and positions differ most (farthest-point sampling over the
    camera's rotation vector, in rad, and position, in units of 0.1 m)."""
    feats = []
    for q in qs:
        T = ptr.gripper(q[:5]) @ X
        rv, t = rvec_t(T)
        feats.append(np.concatenate([rv, t / 0.1]))
    feats = np.array(feats)
    chosen = [int(np.argmin([np.abs(q[:5] - first[:5]).max() for q in qs]))]
    d = np.linalg.norm(feats - feats[chosen[0]], axis=1)
    while len(chosen) < min(n, len(qs)):
        k = int(np.argmax(d))
        chosen.append(k)
        d = np.minimum(d, np.linalg.norm(feats - feats[k], axis=1))
    order = [chosen[0]]
    left = set(chosen[1:])
    while left:
        here = qs[order[-1]]
        k = min(left, key=lambda i: np.abs(qs[i][:5] - here[:5]).max())
        order.append(k)
        left.remove(k)
    return order


# -- the ZED -------------------------------------------------------------------------------------------------
def zed_in_base(Y: np.ndarray, T_zed_board: np.ndarray) -> np.ndarray:
    """The ZED left eye's frame in the arm's base frame."""
    return Y @ inv(T_zed_board)


def zed_fit(views: list[tuple[ch.View, np.ndarray]], intr: ch.Intrinsics) -> tuple[np.ndarray, float]:
    """The ZED in the base frame from its views of the board in several placements (view, board in base):
    one pose for all their corners (reprojection error). One placement alone fixes the ZED's tilt only
    through the board's small extent; placements apart tie it down. Returns (T_base_zed, RMS px)."""
    T0 = average_pose([zed_in_base(Y, pnp(v, intr)) for v, Y in views])

    def res(x):
        T = inv(se3(x[:3], x[3:]))  # base -> ZED
        out = []
        for v, Y in views:
            rv, t = rvec_t(T @ Y)
            p, _ = cv2.projectPoints(v.obj, rv, t, intr.K, intr.D)
            out.append((p.reshape(-1, 2) - v.img).ravel())
        return np.concatenate(out)

    r = least_squares(res, np.concatenate(rvec_t(T0)), x_scale="jac", loss="soft_l1")
    return se3(r.x[:3], r.x[3:]), float(np.sqrt((r.fun**2).mean() * 2))
