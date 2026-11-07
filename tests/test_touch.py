"""S2: the touch model, the fit and the held-out check on a simulated arm."""

from pathlib import Path

import numpy as np
import pytest

from so101_calib import touch as tc

PTR = tc.Pointer()
OFF = np.array([0.0, 2.0, -3.0, 2.5, 0.0])
P = np.array([-0.022, 0.003, -0.127])
TABLE = tc.Placement(np.radians(-88.0), np.array([0.24, 0.01, 0.007]))
BOX = tc.Placement(np.radians(-93.0), np.array([0.235, -0.01, 0.12]))


@pytest.fixture(scope="module")
def touches():
    rng = np.random.default_rng(0)
    exact = dict(touch_mm=0.0, joint_deg=0.0)
    fit = tc.simulate(PTR, tc.touch_list("fit"), TABLE, OFF, P, rng, **exact)
    check = tc.simulate(PTR, tc.touch_list("check"), BOX, OFF, P, rng, **exact)
    return fit, check


def test_every_planned_touch_is_reachable(touches):
    (_, _, kept), (_, _, kept_b) = touches
    assert len(kept) == len(tc.touch_list("fit")) and len(kept_b) == len(tc.touch_list("check"))


def test_fit_recovers_the_offsets_and_the_pointer(touches):
    (qs, dots, _), _ = touches
    f = tc.fit(PTR, qs, dots)
    assert np.allclose(f.offsets_deg, OFF, atol=0.02)
    assert np.allclose(f.pointer, P, atol=1e-4)
    assert f.rms_mm < 0.05
    assert f.offsets_deg[0] == 0.0 and f.offsets_deg[4] == 0.0  # pan and roll are not fitted


def test_without_offsets_the_fit_is_worse_and_the_check_shows_it(touches):
    (qs, dots, _), (qb, db, _) = touches
    before = tc.fit(PTR, qs, dots, fit_offsets=False)
    after = tc.fit(PTR, qs, dots)
    assert before.rms_mm > 10 * after.rms_mm
    good = tc.check(PTR, qb, db, after.offsets_deg, after.pointer)
    bad = tc.check(PTR, qb, db, before.offsets_deg, before.pointer)
    assert good.rms_mm < 0.05 < 0.5 < bad.rms_mm
    assert np.abs(tc.distance_errors_mm(PTR, qb, db, after.offsets_deg, after.pointer)).max() < 0.05


def test_a_pan_offset_is_absorbed_by_the_placement():
    """A pan offset turns the arm like a turned grid: it must not change the fit (it is not observable)."""
    rng = np.random.default_rng(1)
    exact = dict(touch_mm=0.0, joint_deg=0.0)
    qs, dots, _ = tc.simulate(PTR, tc.touch_list("fit"), TABLE, OFF + [4.0, 0, 0, 0, 0], P, rng, **exact)
    f = tc.fit(PTR, qs, dots)
    assert f.rms_mm < 0.05 and np.allclose(f.offsets_deg[1:4], OFF[1:4], atol=0.02)


def test_dots_match_the_grid_page():
    html = (Path(__file__).parents[1] / "tools" / "ipad_grid.html").read_text()
    assert f"PITCH_MM = {tc.PITCH_M * 1000:g}" in html and f'COLS = "{tc.COLS}"' in html
