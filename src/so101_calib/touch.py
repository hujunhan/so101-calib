"""S2: joint zero offsets from touches on an iPad dot grid.

Torque off, the user guides a pointer fixed to the gripper onto dots of a grid shown on an iPad; each touch
records the joint angles. If the model is right, the pointer tip (forward kinematics) is at the dot:

    FK(q + delta, p) = R d_i + t

delta: zero offsets of shoulder_lift, elbow_flex and wrist_flex (to find); p: the pointer tip in the gripper
frame; R, t: where the iPad lies (both nuisance). Least squares over all touches.

What the touches can and cannot tell apart (the null space of the Jacobian):
- a shoulder_pan offset turns the arm about the vertical axis, like a turned iPad: not fitted (S4);
- a wrist_roll offset turns the pointer about the roll axis, like a pointer fixed elsewhere: not fitted;
- a wrist_flex offset looks like a moved pointer as long as wrist_roll is constant: so some touches turn the
  gripper (roll -60 and +60 deg);
- a shoulder_lift offset tilts the whole arm about a horizontal axis, like a tilted iPad: the iPad lies flat
  on the table the base stands on, so its plane is fixed parallel to the base (only yaw and position fitted).
"""

from dataclasses import dataclass

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from so101_kinematics.models import HI, LO, PinArm

# The grid shown on the iPad (tools/ipad_grid.html draws the same): columns A-D 50 mm apart along the long
# side, rows 1-3 50 mm apart, row 1 nearest the arm. Grid frame: x along the rows (A -> D), y along the
# columns (1 -> 3), origin at the centre, z out of the screen.
COLS, ROWS, PITCH_M = "ABCD", (1, 2, 3), 0.050
DOTS = {
    f"{c}{r}": np.array([(i - 1.5) * PITCH_M, (j - 1.0) * PITCH_M, 0.0])
    for i, c in enumerate(COLS)
    for j, r in enumerate(ROWS)
}
FITTED = (1, 2, 3)  # shoulder_lift, elbow_flex, wrist_flex (ARM_JOINTS indices)
POINTER_GUESS = np.array([-0.025, 0.0, -0.125])  # taped to the outside of the fixed jaw, ~2 cm past its tip
STEEP, SHALLOW = 70.0, 40.0  # deg the pointer points below horizontal
ROLL_TURN = 60.0  # deg


@dataclass(frozen=True)
class Touch:
    dot: str
    pitch_deg: float  # how the user should hold the pointer (a guide: the recorded joints are what counts)
    roll_deg: float


def touch_list(kind: str) -> list[Touch]:
    """'fit': three dots, each touched four ways (pointer steep and shallow, gripper turned to roll -60
    and +60 deg), 12 touches; in simulation as precise as 27 touches with level grippers added, because the
    turned and tilted touches are what separate the offsets from the pointer. 'check': six dots straight,
    for the held-out check."""
    if kind == "check":
        return [Touch(d, STEEP, 0.0) for d in ("A1", "A3", "B2", "C2", "D1", "D3")]
    return [
        Touch(d, pitch, roll)
        for d in ("A1", "B3", "D2")
        for pitch in (STEEP, SHALLOW)
        for roll in (-ROLL_TURN, ROLL_TURN)
    ]


class Pointer:
    """Forward kinematics of a point fixed to the gripper (the wrist_roll joint frame, = MuJoCo "gripper")."""

    def __init__(self):
        self.arm = PinArm()
        self.data = self.arm.model.createData()
        self.jid = self.arm.model.getJointId("wrist_roll")

    def gripper(self, q: np.ndarray) -> np.ndarray:
        pin.forwardKinematics(self.arm.model, self.data, self.arm._full(q[:5]))
        return self.data.oMi[self.jid].homogeneous

    def tip(self, q: np.ndarray, p: np.ndarray) -> np.ndarray:
        T = self.gripper(q)
        return T[:3, :3] @ p + T[:3, 3]

    def pitch_deg(self, q: np.ndarray) -> float:
        """Angle of the gripper axis (-z of the gripper frame) below horizontal."""
        return float(np.degrees(np.arcsin(np.clip(self.gripper(q)[2, 2], -1, 1))))


def rot_z(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


@dataclass
class Placement:
    """Where the iPad grid lies: flat (parallel to the base's bottom face), turned by yaw, origin at t."""

    yaw: float
    t: np.ndarray

    def world(self, d: np.ndarray) -> np.ndarray:
        return rot_z(self.yaw) @ d + self.t


# -- fitting -------------------------------------------------------------------------------------------------
@dataclass
class Fit:
    offsets_deg: np.ndarray  # (5,): only FITTED nonzero
    offsets_std_deg: np.ndarray
    pointer: np.ndarray  # m, gripper frame
    placement: Placement
    resid_mm: np.ndarray  # (n, 3)

    @property
    def rms_mm(self) -> float:
        return float(np.sqrt((self.resid_mm**2).sum(1).mean()))

    @property
    def max_mm(self) -> float:
        return float(np.linalg.norm(self.resid_mm, axis=1).max())


def _residuals(ptr: Pointer, qs, dots, offsets, p, pl: Placement) -> np.ndarray:
    return np.concatenate(
        [(ptr.tip(q + offsets, p) - pl.world(d)) * 1000 for q, d in zip(qs, dots, strict=True)]
    )


def fit(
    ptr: Pointer, qs: np.ndarray, dots: np.ndarray, fit_offsets: bool = True, guess: Placement | None = None
) -> Fit:
    """Least squares for the offsets (if fit_offsets), the pointer and the placement, from touches (joint
    angles qs (n, 5+), dot positions in the grid frame (n, 3))."""
    if guess is None:
        guess = guess_placement(ptr, qs, dots, POINTER_GUESS)
    nf = len(FITTED) if fit_offsets else 0

    def unpack(x):
        off = np.zeros(5)
        off[list(FITTED)[:nf]] = x[:nf]
        return off, x[nf : nf + 3], Placement(x[nf + 3], x[nf + 4 : nf + 7])

    def res(x):
        return _residuals(ptr, qs[:, :5], dots, *unpack(x))

    x0 = np.concatenate([np.zeros(nf), POINTER_GUESS, [guess.yaw], guess.t])
    r = least_squares(res, x0, x_scale="jac")
    off, p, pl = unpack(r.x)
    # parameter std from the Jacobian and the residual scatter (sigma^2 (J^T J)^-1)
    dof = max(len(r.fun) - len(r.x), 1)
    sigma2 = float(r.fun @ r.fun) / dof
    cov = sigma2 * np.linalg.pinv(r.jac.T @ r.jac)
    std = np.zeros(5)
    std[list(FITTED)[:nf]] = np.sqrt(np.diag(cov)[:nf])
    return Fit(np.degrees(off), np.degrees(std), p, pl, r.fun.reshape(-1, 3))


def guess_placement(ptr: Pointer, qs, dots, p) -> Placement:
    """A starting placement: the best rigid yaw + shift of the dots onto the touched tips (2-D Procrustes)."""
    tips = np.array([ptr.tip(q[:5], p) for q in qs])
    a, b = dots[:, :2] - dots[:, :2].mean(0), tips[:, :2] - tips[:, :2].mean(0)
    h = a.T @ b
    yaw = float(np.arctan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1]))
    t = tips.mean(0) - rot_z(yaw) @ dots.mean(0)
    return Placement(yaw, t)


def check(ptr: Pointer, qs, dots, offsets_deg: np.ndarray, pointer: np.ndarray) -> Fit:
    """Held-out touches with the offsets and the pointer fixed: only their placement is fitted (4 numbers)."""
    off = np.radians(offsets_deg)
    g = guess_placement(ptr, qs + np.append(off, np.zeros(qs.shape[1] - 5)), dots, pointer)

    def res(x):
        return _residuals(ptr, qs[:, :5], dots, off, pointer, Placement(x[0], x[1:4]))

    r = least_squares(res, np.concatenate([[g.yaw], g.t]))
    return Fit(offsets_deg, np.zeros(5), pointer, Placement(r.x[0], r.x[1:4]), r.fun.reshape(-1, 3))


def distance_errors_mm(ptr: Pointer, qs, dots, offsets_deg, pointer) -> np.ndarray:
    """For every pair of touches on different dots: FK distance - true distance (mm). Needs no placement:
    the ruler check of M1, on every pair."""
    off = np.radians(offsets_deg)
    tips = np.array([ptr.tip(q[:5] + off, pointer) for q in qs])
    out = []
    for i in range(len(qs)):
        for j in range(i + 1, len(qs)):
            if np.linalg.norm(dots[i] - dots[j]) > 1e-6:
                out.append(np.linalg.norm(tips[i] - tips[j]) - np.linalg.norm(dots[i] - dots[j]))
    return np.array(out) * 1000


# -- simulation ----------------------------------------------------------------------------------------------
def ik_touch(ptr: Pointer, target, p, pitch_deg, roll_deg, q0) -> tuple[np.ndarray, float]:
    """Joint angles that put the pointer tip at `target` with the gripper pitched and rolled as asked."""

    def f(q):
        return np.concatenate(
            [
                (ptr.tip(q, p) - target) * 1000,
                [0.05 * (ptr.pitch_deg(q) - pitch_deg), 0.5 * (np.degrees(q[4]) - roll_deg)],
            ]
        )

    r = least_squares(f, q0, bounds=(LO + 0.05, HI - 0.05))
    return r.x, float(np.linalg.norm(r.fun[:3]))


def simulate(
    ptr: Pointer,
    touches: list[Touch],
    placement: Placement,
    offsets_deg: np.ndarray,
    pointer: np.ndarray,
    rng: np.random.Generator,
    touch_mm: float = 0.5,
    joint_deg: float = 0.1,
) -> tuple[np.ndarray, np.ndarray, list[Touch]]:
    """Joint readings for the touches on an arm with these zero offsets (reading = true angle - offset),
    with touch and encoder noise. Unreachable touches (> 0.5 mm from the dot) are dropped."""
    qs, dots, kept = [], [], []
    q0 = np.radians([0.0, -30.0, 40.0, 50.0, 0.0])
    for tc in touches:
        d = DOTS[tc.dot]
        q, err = ik_touch(ptr, placement.world(d), pointer, tc.pitch_deg, tc.roll_deg, q0)
        if err > 0.5:
            continue
        q = q - np.radians(offsets_deg) + np.radians(rng.normal(0.0, joint_deg, 5))
        qs.append(q)
        dots.append(d + np.append(rng.normal(0.0, touch_mm / 1000, 2), 0.0))
        kept.append(tc)
    return np.array(qs), np.array(dots), kept
