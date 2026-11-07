"""S1a: servo system identification, one joint at a time: test programs (steps and sines) and their analysis.

A program holds every joint at the test pose and moves one joint: small steps up and down (delay, rise
time, deadband, the error left after settling), then sines (gain and phase against frequency). Commands go
out at the safety rate (30 Hz); `SafeArm.follow(dense=True)` reads the arm about every millisecond in
between, so the delay is measured to ~1 ms instead of one tick (33 ms) as in M1.
"""

from dataclasses import dataclass

import numpy as np
from so101_core.model import ARM_JOINTS, JOINTS

# Arm forward and slightly down, both upper links loaded by gravity: the tool is ~29 cm in front of the
# base and ~13 cm up (FK). Gripper: kept where the arm is.
TEST_POSE_DEG = np.array([0.0, -34.0, 34.0, 34.0, 0.0])

STEP_SIZES_DEG = (0.5, 1.0, 1.5)  # one tick each: below the 2 deg step cap, so they stay real steps
STEP_HOLD_S = 1.0
SINE_AMP_DEG = 4.0
SINE_FREQS_HZ = (0.2, 0.5, 0.75, 1.0)  # 1.25 Hz would come within 1 deg of the stall threshold
SINE_CYCLES = (3, 4, 4, 5)
SETTLE_S = 1.0  # hold before the first step and after each sine

# Model used only for the preflight (M1's fit): what following error to expect, against the stall threshold.
EXPECTED_DELAY_S = 0.10
EXPECTED_SAG_DEG = 1.0

KINDS = ("hold", "step", "sine")


@dataclass
class Segment:
    kind: str  # "hold", "step" or "sine"
    start: int  # first tick
    end: int  # one past the last tick
    value: float  # step: the step (deg, signed); sine: frequency (Hz); hold: 0


def program(q0: np.ndarray, joint: int, hz: float) -> tuple[np.ndarray, list[Segment]]:
    """Ticks (n, 6) that move `joint` around `q0` and hold every other joint at `q0`."""
    offsets: list[float] = []
    segments: list[Segment] = []

    def add(kind: str, values: np.ndarray, value: float = 0.0) -> None:
        segments.append(Segment(kind, len(offsets), len(offsets) + len(values), value))
        offsets.extend(values)

    def hold(seconds: float, at: float = 0.0) -> np.ndarray:
        return np.full(round(seconds * hz), at)

    add("hold", hold(SETTLE_S))
    level = 0.0
    for size in STEP_SIZES_DEG:
        for target in (size, 0.0, -size, 0.0):
            add("step", hold(STEP_HOLD_S, target), target - level)
            level = target
    for f, cycles in zip(SINE_FREQS_HZ, SINE_CYCLES, strict=True):
        t = np.arange(round(cycles / f * hz)) / hz
        add("sine", SINE_AMP_DEG * np.sin(2 * np.pi * f * t), f)
        add("hold", hold(SETTLE_S))
    ticks = np.tile(q0, (len(offsets), 1))
    ticks[:, joint] += np.radians(offsets)
    return ticks, segments


def segments_to_array(segments: list[Segment]) -> np.ndarray:
    return np.array([(KINDS.index(s.kind), s.start, s.end, s.value) for s in segments], dtype=float)


def segments_from_array(a: np.ndarray) -> list[Segment]:
    return [Segment(KINDS[int(k)], int(s), int(e), float(v)) for k, s, e, v in a]


def expected_following_error(ticks: np.ndarray, hz: float) -> np.ndarray:
    """Per-joint worst |command - measured| (rad) if the arm were a pure delay plus sag (M1's model)."""
    lag = max(1, round(EXPECTED_DELAY_S * hz))
    delayed = np.vstack([np.repeat(ticks[:1], lag, axis=0), ticks[:-lag]])
    return np.max(np.abs(ticks - delayed), axis=0) + np.radians(EXPECTED_SAG_DEG)


def ramp_path(q_from: np.ndarray, goal: np.ndarray, step: float) -> np.ndarray:
    """The commands `SafeArm.move_to` sends: every joint moves at most `step` per tick until it arrives."""
    path = [q_from]
    q = q_from.copy()
    while not np.allclose(q, goal):
        q = q + np.clip(goal - q, -step, step)
        path.append(q)
    return np.array(path)


# -- analysis ------------------------------------------------------------------------------------------------
COUNT_DEG = 360 / 4096  # one encoder count


def step_metrics(t_sent, q_cmd, t_dense, q_dense, seg: Segment, joint: int, hz: float) -> dict:
    """Times from the step command to 10/50/90 % of the final change, and the error left after settling."""
    t0 = t_sent[seg.start]
    t_end = t_sent[seg.end - 1] + 1 / hz
    y = np.degrees(q_dense[:, joint])
    before = y[(t_dense > t0 - 0.2) & (t_dense < t0)]
    inside = (t_dense >= t0) & (t_dense < t_end)
    t, yy = t_dense[inside] - t0, y[inside]
    base = np.mean(before)
    final = np.mean(yy[t > t[-1] - 0.3])
    change = final - base
    out = {
        "step_deg": seg.value,
        "moved_deg": change,
        # Commanded minus measured after settling: sag plus whatever depends on the direction of approach.
        "error_deg": np.degrees(q_cmd[seg.start, joint]) - final,
        # Step minus the change it produced: the constant sag cancels, the direction-dependent part stays.
        "shortfall_deg": seg.value - change,
    }
    moved = np.abs(yy - base) >= 1.5 * COUNT_DEG
    out["t_first_ms"] = 1000 * t[np.argmax(moved)] if moved.any() else None
    for p in (10, 50, 90):
        reached = (yy - base) * np.sign(change) >= p / 100 * abs(change)
        ok = reached.any() and abs(change) >= 2 * COUNT_DEG
        out[f"t{p}_ms"] = 1000 * t[np.argmax(reached)] if ok else None
    return out


def _fit_sine(t: np.ndarray, y: np.ndarray, f: float) -> tuple[float, float]:
    """Amplitude and phase (rad) of y = A sin(2 pi f t + phi) + c, by least squares."""
    w = 2 * np.pi * f
    m = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)])
    (a, b, _), *_ = np.linalg.lstsq(m, y, rcond=None)
    return float(np.hypot(a, b)), float(np.arctan2(b, a))


def sine_metrics(t_sent, q_cmd, t_dense, q_dense, seg: Segment, joint: int) -> dict:
    """Gain and phase lag of the measured joint against the commanded sine; the first cycle is skipped."""
    f = seg.value
    t_start = t_sent[seg.start] + 1 / f
    t_stop = t_sent[seg.end - 1]
    c = (t_sent >= t_start) & (t_sent <= t_stop)
    m = (t_dense >= t_start) & (t_dense <= t_stop)
    a_cmd, p_cmd = _fit_sine(t_sent[c], np.degrees(q_cmd[c, joint]), f)
    a_meas, p_meas = _fit_sine(t_dense[m], np.degrees(q_dense[m, joint]), f)
    lag = (p_cmd - p_meas) % (2 * np.pi)
    return {
        "freq_hz": f,
        "gain": a_meas / a_cmd,
        "phase_deg": np.degrees(lag),
        "delay_ms": 1000 * lag / (2 * np.pi * f),
    }


def coupling_deg(q_cmd, q_dense, t_dense, t_sent, joint: int) -> dict:
    """How far the other joints moved (peak to peak, deg) while this one was tested."""
    m = t_dense >= t_sent[0] + SETTLE_S / 2
    return {JOINTS[k]: float(np.ptp(np.degrees(q_dense[m, k]))) for k in range(len(ARM_JOINTS)) if k != joint}
