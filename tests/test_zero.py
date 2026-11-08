"""so101-zero: the zero pose tells a 7 deg elbow error by the wrist axis' height."""

import numpy as np

from so101_calib import zero_cli


def test_zero_pose_heights_tell_the_elbow_error():
    q = np.zeros(6)
    h = zero_cli.axis_heights_mm(q)
    assert abs(h["wrist_flex"] - h["elbow_flex"] - 5.2) < 1.0  # forearm nearly level at zero
    q[2] = np.radians(7.28)  # the elbow 7.3 deg further: the forearm droops
    h2 = zero_cli.axis_heights_mm(q)
    assert (h2["wrist_flex"] - h2["elbow_flex"]) - (h["wrist_flex"] - h["elbow_flex"]) < -15
