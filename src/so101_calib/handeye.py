"""S4: where the cameras are. The iPad board lies still on the table; the arm moves the wrist camera over it.

For each arm pose i (joint angles q_i, read after the arm settled) the wrist camera sees board corners P_j:

    pixel_ij = project(K, D, T_cam_board_i P_j),   T_cam_board_i = X^-1 G(q_i + delta)^-1 Y

G: the gripper frame in the base (FK; the wrist_roll joint frame), X: the camera in the gripper frame,
Y: the board in the base frame, delta: optional zero offsets of shoulder_lift, elbow_flex and wrist_flex.
One least-squares fit of all pixels (reprojection error); OpenCV's closed-form hand-eye only starts it.

The ZED sees the same board: T_base_zed = Y T_zed_board^-1.
"""

from dataclasses import dataclass

import cv2
import numpy as np
from scipy.optimize import least_squares

from so101_calib import charuco as ch
from so101_calib.touch import Pointer

FITTED = (1, 2, 3)  # shoulder_lift, elbow_flex, wrist_flex


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


@dataclass
class Fit:
    X: np.ndarray  # camera in gripper
    Y: np.ndarray  # board in base
    offsets_deg: np.ndarray  # (5,)
    rms_px: float
    per_sample_px: np.ndarray
    std: np.ndarray  # parameter std: X (rvec, t), Y (rvec, t), offsets (rad) as fitted


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


def fit(
    samples: list[Sample],
    intr: ch.Intrinsics,
    ptr: Pointer,
    offsets: bool = False,
    flat: bool = True,
    start: tuple[np.ndarray, np.ndarray] | None = None,
) -> Fit:
    """flat: the board lies on the table the base stands on, so only its yaw and position are free (as in
    S2, this keeps a shoulder_lift offset from hiding in a tilted board)."""
    X0, Y0 = start if start is not None else initial(samples, intr, ptr)
    nf = len(FITTED) if offsets else 0
    ny = 4 if flat else 6

    def unpack(x):
        X = se3(x[0:3], x[3:6])
        if flat:
            Y = np.eye(4)
            Y[:3, :3], Y[:3, 3] = rot_z(x[6]) @ FLAT_R, x[7:10]
        else:
            Y = se3(x[6:9], x[9:12])
        off = np.zeros(5)
        off[list(FITTED)[:nf]] = x[6 + ny : 6 + ny + nf]
        return X, Y, off

    def res(x):
        X, Y, off = unpack(x)
        return np.concatenate([(_project(s, X, Y, off, intr, ptr) - s.view.img).ravel() for s in samples])

    y0 = np.concatenate([[flat_yaw(Y0[:3, :3])], Y0[:3, 3]]) if flat else np.concatenate(rvec_t(Y0))
    x0 = np.concatenate([*rvec_t(X0), y0, np.zeros(nf)])
    r = least_squares(res, x0, x_scale="jac", loss="soft_l1", f_scale=1.0)
    X, Y, off = unpack(r.x)
    per = []
    for s in samples:
        e = _project(s, X, Y, off, intr, ptr) - s.view.img
        per.append(np.sqrt((e**2).sum(1).mean()))
    resid = res(r.x)
    dof = max(len(resid) - len(r.x), 1)
    cov = float(resid @ resid) / dof * np.linalg.pinv(r.jac.T @ r.jac)
    return Fit(
        X, Y, np.degrees(off), float(np.sqrt((resid**2).mean() * 2)), np.array(per), np.sqrt(np.diag(cov))
    )


def heldout_px(samples, intr, ptr, offsets: bool, folds: int = 4, flat: bool = True) -> float:
    """RMS reprojection error of poses left out of the fit (k-fold): the fair test of whether the offsets
    help or only fit noise."""
    idx = np.arange(len(samples))
    errs = []
    for f in range(folds):
        test = idx[f::folds]
        train = [samples[i] for i in np.setdiff1d(idx, test)]
        ft = fit(train, intr, ptr, offsets, flat)
        off = np.radians(ft.offsets_deg)
        for i in test:
            e = _project(samples[i], ft.X, ft.Y, off, intr, ptr) - samples[i].view.img
            errs.append((e**2).sum(1).mean())
    return float(np.sqrt(np.mean(errs)))


def board_mm_error(fit_: Fit, samples, intr, ptr) -> np.ndarray:
    """Per sample: how far (mm) the board centre seen by the camera (PnP) lies from where the fit puts it
    (through FK): the error a grasp guided by this camera would see, in millimetres."""
    centre = ch.board().getChessboardCorners().mean(0)
    off = np.radians(fit_.offsets_deg)
    out = []
    for s in samples:
        seen = ptr.gripper(s.q[:5] + off) @ fit_.X @ pnp(s.view, intr)
        out.append(
            np.linalg.norm((seen[:3, :3] @ centre + seen[:3, 3]) - (fit_.Y[:3, :3] @ centre + fit_.Y[:3, 3]))
        )
    return np.array(out) * 1000


# -- choosing poses ------------------------------------------------------------------------------------------
def predicted_view(
    q, X, Y, intr: ch.Intrinsics, ptr: Pointer, margin_px: float = 40.0
) -> tuple[float, float, float]:
    """For an arm pose: the fraction of board corners predicted inside the image (with a margin), the angle
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
