"""S1a: the test program is safe as planned, and the analysis recovers a known delay and sag."""

import numpy as np
import pytest
from so101_core.safety import SafetyLayer

from so101_calib import servo_cli
from so101_calib import servo_id as sid

HZ = 30.0


def q0():
    return servo_cli.test_pose(SafetyLayer(), 0.0)


def test_program_moves_only_one_joint_and_passes_the_preflight():
    ticks, segs = sid.program(q0(), 2, HZ)
    others = np.delete(ticks, 2, axis=1)
    assert np.allclose(others, others[0])
    assert segs[0].start == 0 and segs[-1].end == len(ticks)
    assert all(a.end == b.start for a, b in zip(segs, segs[1:], strict=False))
    assert servo_cli.check_program(SafetyLayer(), q0(), list(range(5))) == []


def test_segments_round_trip():
    _, segs = sid.program(q0(), 1, HZ)
    assert sid.segments_from_array(sid.segments_to_array(segs)) == segs


def synthetic_run(delay_s, sag_deg, joint=2):
    """A servo that follows the held (staircase) command after a pure delay, minus a constant sag."""
    ticks, segs = sid.program(q0(), joint, HZ)
    t_sent = np.arange(len(ticks)) / HZ
    t_dense = np.arange(0, t_sent[-1] + 1 / HZ, 0.001)
    held = np.clip(np.floor((t_dense - delay_s) * HZ).astype(int), 0, len(ticks) - 1)
    q_dense = ticks[held] - np.radians(sag_deg) * (np.arange(6) == joint)
    return t_sent, ticks, t_dense, q_dense, segs


def test_analysis_recovers_delay_and_sag():
    t_sent, q_cmd, t_dense, q_dense, segs = synthetic_run(0.080, 0.9)
    steps = [sid.step_metrics(t_sent, q_cmd, t_dense, q_dense, s, 2, HZ) for s in segs if s.kind == "step"]
    for m in steps:
        assert m["t50_ms"] == pytest.approx(80, abs=2)
        assert m["error_deg"] == pytest.approx(0.9, abs=0.01)
        assert m["shortfall_deg"] == pytest.approx(0, abs=0.01)
    for s in segs:
        if s.kind == "sine":
            m = sid.sine_metrics(t_sent, q_cmd, t_dense, q_dense, s, 2)
            assert m["gain"] == pytest.approx(1, abs=0.02)
            assert m["delay_ms"] == pytest.approx(80 + 1000 / HZ / 2, abs=3)  # + half a tick (staircase)


def test_expected_following_error_is_delay_times_speed_plus_sag():
    ticks, _ = sid.program(q0(), 0, HZ)
    peak_speed = 2 * np.pi * max(sid.SINE_FREQS_HZ) * sid.SINE_AMP_DEG  # deg/s
    expected = np.degrees(sid.expected_following_error(ticks, HZ)[0])
    assert expected == pytest.approx(peak_speed * sid.EXPECTED_DELAY_S + sid.EXPECTED_SAG_DEG, rel=0.1)
