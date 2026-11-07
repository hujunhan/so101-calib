"""S1b: the servos' static error over the workspace.

Model (from S1a): a position servo with gain P is a torsion spring. At rest, holding a gravity torque tau,

    error = commanded - measured = c * tau + s * b + o

c is the compliance (deg per N m, set by P), s = +1 when the joint arrived from above the target and -1 from
below, b is the half-width of the band the servo can stop anywhere in (friction against P x error), and o a
constant offset (expected ~0). tau comes from the URDF masses (Pinocchio), plus an optional payload at the
tool.
"""

from dataclasses import dataclass

import numpy as np
import pinocchio as pin
from so101_core.model import ARM_JOINTS
from so101_core.safety import SafetyLayer
from so101_kinematics.models import PinArm

from so101_calib.servo_id import ramp_path

APPROACH_DEG = 3.0  # each approach starts this far above or below the target, on every arm joint
APPROACH_SPEED = np.radians(10.0)
HOLD_S = 1.0  # read the arm this long after it settles; the mean of the last half is the measurement
REPEATS = 2  # each pose: (+, -) approaches, this many times
N_POSES = 16
MIN_TOOL_Z = 0.08  # m above the base-frame origin (the base's bottom face)
MIN_TOOL_X = 0.12  # m in front of the base
GRAVITY_JOINTS = (1, 2, 3)  # shoulder_lift, elbow_flex, wrist_flex; pan and roll carry no gravity torque


class Gravity:
    """Joint torques (N m) that hold the arm still, from the URDF masses, plus a point payload at the tool."""

    def __init__(self, payload_kg: float = 0.0):
        self.arm = PinArm()
        self.payload_kg = payload_kg

    def frames_z(self, q: np.ndarray) -> np.ndarray:
        """Heights (m) of every body frame (joint origins and the tool)."""
        m, d = self.arm.model, self.arm.data
        pin.framesForwardKinematics(m, d, self.arm._full(q[:5]))
        return np.array(
            [d.oMf[i].translation[2] for i, f in enumerate(m.frames) if f.type == pin.FrameType.BODY]
        )

    def tau(self, q: np.ndarray) -> np.ndarray:
        """(5,) holding torques of the arm joints for joint angles q (rad, 5 or 6 values)."""
        a = self.arm
        g = pin.computeGeneralizedGravity(a.model, a.data, a._full(q[:5]))[a.idx]
        if self.payload_kg:
            # A point mass at the tool: the joints hold J^T times its weight (pointing down).
            weight = np.array([0.0, 0.0, -self.payload_kg * 9.81])
            g = g - a.jacobian(q[:5])[:3].T @ weight
        return g


@dataclass
class Candidate:
    q: np.ndarray  # (5,) rad
    tau: np.ndarray  # (5,) N m


def candidate_poses(safety: SafetyLayer, grav: Gravity) -> list[Candidate]:
    """A grid over lift, elbow and wrist (pan = roll = 0) inside the safe range, tool in front and above the
    table, with room for the approach offsets."""
    out = []
    margin = np.radians(APPROACH_DEG)
    for lift in range(-80, 31, 10):
        for elbow in range(-40, 91, 10):
            for wrist in range(-60, 91, 15):
                q = np.radians([0.0, lift, elbow, wrist, 0.0])
                if np.any(q - margin < safety.lo[:5]) or np.any(q + margin > safety.hi[:5]):
                    continue
                tool = grav.arm.fk(q)[:3, 3]
                if tool[2] < MIN_TOOL_Z or tool[0] < MIN_TOOL_X or grav.frames_z(q).min() < -1e-3:
                    continue
                out.append(Candidate(q, grav.tau(q)))
    return out


def spread_poses(cands: list[Candidate], start: np.ndarray, n: int) -> list[Candidate]:
    """Pick n poses that spread the gravity torques of lift, elbow and wrist (farthest-point sampling,
    torques scaled by their range), starting from the candidate nearest to `start`."""
    taus = np.array([c.tau[list(GRAVITY_JOINTS)] for c in cands])
    scaled = (taus - taus.min(0)) / np.ptp(taus, axis=0)
    first = int(np.argmin([np.abs(c.q - start[:5]).max() for c in cands]))
    chosen = [first]
    dist = np.linalg.norm(scaled - scaled[first], axis=1)
    while len(chosen) < n:
        nxt = int(np.argmax(dist))
        chosen.append(nxt)
        dist = np.minimum(dist, np.linalg.norm(scaled - scaled[nxt], axis=1))
    # Visit them in a short order (greedy nearest neighbour in joint space, from the first).
    order = [chosen[0]]
    left = set(chosen[1:])
    while left:
        here = cands[order[-1]].q
        nxt = min(left, key=lambda i: np.abs(cands[i].q - here).max())
        order.append(nxt)
        left.remove(nxt)
    return [cands[i] for i in order]


def approach_targets(q: np.ndarray, safety: SafetyLayer) -> list[tuple[int, np.ndarray]]:
    """(side, start) pairs for one pose: start APPROACH_DEG above (side +1) or below (-1), every arm joint."""
    out = []
    for _ in range(REPEATS):
        for side in (1, -1):
            start = q.copy()
            start[:5] += side * np.radians(APPROACH_DEG)
            out.append((side, safety.clip(start)[0]))
    return out


class Collisions:
    """Contacts between the arm's collision meshes, and with the floor plane at the base's bottom face, in
    so101-core's MuJoCo model. The folded rest pose itself overlaps in the model (the meshes are coarse and
    the real arm rests on the table), so routes are checked once they have left the rest pose."""

    def __init__(self, clearance_m: float = 0.005):
        from so101_core.twin import Twin

        self.twin = Twin()
        self.clearance = clearance_m

    def hits(self, q: np.ndarray) -> list[tuple[str, str]]:
        m, d = self.twin.model, self.twin.data
        self.twin.set(q)
        out = []
        for c in d.contact[: d.ncon]:
            if c.dist < self.clearance:
                out.append((m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name))
        return out


def route(poses: list[np.ndarray], q_from: np.ndarray, safety: SafetyLayer, step: float) -> np.ndarray:
    """The `move_to` commands that visit `poses` in order from q_from (every `step` rad per joint)."""
    out = [safety.clip(q_from)[0][None]]
    q = out[0][0]
    for p in poses:
        out.append(ramp_path(q, p, step))
        q = p
    return np.vstack(out)


def route_problems(
    path: np.ndarray, coll: "Collisions", rest: np.ndarray, near_rest_rad: float = 0.35
) -> list[str]:
    """Contacts on the route, except next to the rest pose (which is in contact in the model): contacts are
    allowed only while every arm joint is within `near_rest_rad` of rest."""
    problems = []
    for i, q in enumerate(path):
        hits = coll.hits(q)
        if hits and np.abs(q[:5] - rest[:5]).max() > near_rest_rad:
            problems.append(f"command {i}: {sorted(set(hits))} at {np.round(np.degrees(q[:5])).tolist()} deg")
    return problems


# -- analysis ------------------------------------------------------------------------------------------------
def fit_joint(err_deg: np.ndarray, side: np.ndarray, tau: np.ndarray | None) -> dict:
    """Least squares err = c * tau + b * side + o; without tau (pan, roll): err = b * side + o."""
    cols = [side, np.ones_like(side)] if tau is None else [tau, side, np.ones_like(side)]
    m = np.column_stack(cols)
    coef, *_ = np.linalg.lstsq(m, err_deg, rcond=None)
    resid = err_deg - m @ coef
    out = {"rms_resid_deg": float(np.sqrt(np.mean(resid**2))), "max_resid_deg": float(np.abs(resid).max())}
    if tau is None:
        out |= {"c": None, "b": float(coef[0]), "o": float(coef[1])}
    else:
        out |= {"c": float(coef[0]), "b": float(coef[1]), "o": float(coef[2])}
    out["rms_before_deg"] = float(np.sqrt(np.mean(err_deg**2)))
    return out


def joint_names() -> tuple[str, ...]:
    return ARM_JOINTS
