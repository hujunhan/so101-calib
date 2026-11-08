# so101-calib: theory

The theory behind M2: what each method does, why it works, and where it stops working on this arm. Read it with
the code. [DESIGN.md](DESIGN.md) explains how the code is built, the [README](../README.md) has the numbers, and
[DECISIONS.md](../DECISIONS.md) records when and why each decision was made.

Notation: $q \in \mathbb{R}^5$ are the arm joints (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
wrist_roll). $T_{a\,b}$ is the pose of frame $b$ in frame $a$ (a 4x4 rigid transform), so
$T_{a\,c} = T_{a\,b} T_{b\,c}$. $G(q)$ is the gripper (wrist_roll joint) frame in the base frame, by forward
kinematics on the URDF. Lengths in metres and angles in radians unless stated otherwise.

Contents:

1. [A position servo is a spring with a friction band](#1-a-position-servo-is-a-spring-with-a-friction-band)
2. [Feed-forward: commanding past the target](#2-feed-forward-commanding-past-the-target)
3. [Kinematic calibration and what it can identify](#3-kinematic-calibration-and-what-it-can-identify)
4. [Where LeRobot puts zero, and why it was 7 degrees off](#4-where-lerobot-puts-zero-and-why-it-was-7-degrees-off)
5. [The camera model and its calibration](#5-the-camera-model-and-its-calibration)
6. [Hand-eye calibration](#6-hand-eye-calibration)
7. [The ZED through a shared board](#7-the-zed-through-a-shared-board)
8. [From a pixel to a point on the table](#8-from-a-pixel-to-a-point-on-the-table)
9. [The error budget](#9-the-error-budget)
10. [Choosing between models: held-out data](#10-choosing-between-models-held-out-data)
11. [Gravity, backlash and the folded arm](#11-gravity-backlash-and-the-folded-arm)
12. [What the measurements can and cannot see](#12-what-the-measurements-can-and-cannot-see)
13. [References](#13-references)

---

## 1. A position servo is a spring with a friction band

An STS3215 is a DC motor, a gearbox and a magnetic encoder on the output shaft, closed by a proportional
position loop inside the servo. To first order the motor torque is proportional to the position error
$e = q_\text{cmd} - q$:

$$\tau_\text{motor} = K\,e, \qquad K \propto P .$$

At rest the motor torque balances the load. For a joint holding the arm against gravity, the load is the
gravity torque $\tau(q)$ of everything beyond that joint, which the URDF masses give (Pinocchio's generalized
gravity). So the joint stops short by

$$e = \tau / K = c\,\tau, \qquad c = 1/K \ \ \text{(compliance, deg per N m)}.$$

The servo behaves like a **torsion spring** whose stiffness is set by the gain $P$. Doubling $P$ halves $c$:
S1a measured the elbow's error against gravity fall from 3.0 to 1.55 deg when $P$ went from 16 to 32.

Friction adds a **band**. Static friction can hold the joint anywhere the motor torque $K e$ is smaller than the
friction torque $\tau_f$; so the joint can stop anywhere within $\pm \tau_f / K$ of the spring's rest point, and
on which side depends on where it came from. Writing $s = +1$ for a joint that arrived from above (moving down)
and $-1$ from below:

$$e = c\,\tau + s\,(b + k\,|\tau|) + o .$$

$b$ is the friction of an unloaded gear train. $k|\tau|$ is the part that grows with load: gear-tooth friction
is proportional to the force the teeth carry (Coulomb friction, $\mu N$). The data demand this term: with a
constant band the fit leaves 0.14-0.18 deg RMS on lift and elbow, with it 0.08-0.09 deg, about one encoder count
(0.088 deg). Because the band is friction divided by $K$, it also shrinks as $P$ rises (2-4x from $P$ = 16 to
32), which rules out a fixed electronic deadband as its cause.

The model has no memory of the pose: given $\tau$ and $s$ it predicts the error anywhere. That is why it carries
over to a 100 g payload it was not fitted on (lift and elbow 0.84-0.91 deg RMS down to 0.14-0.21 deg), and why
a 16-pose survey is enough. $o$ is a small constant (0.1-0.2 deg): a torque the URDF lacks (cables), or a
residual zero error. $c$ fitted on both runs is 1.62 / 1.79 / 1.81 deg per N m for lift, elbow and wrist.

Timing is separate. A command is answered after 50-55 ms (the servo's own loop and bus latency) and the joint
is halfway after about 70 ms. Moving slowly adds a speed-dependent lag on top: the command must get one band
ahead before the joint moves, which takes $b/\dot q$: 90 ms at 5 deg/s, 18 ms at 25 deg/s. M1's single
"100 ms delay" was these together with half a 30 Hz command tick.

## 2. Feed-forward: commanding past the target

Inverting the model gives the command that puts the joint on the target $q^*$:

$$q_\text{cmd} = q^* + c\,\tau(q^*) + s\,(b + k|\tau(q^*)|) + o .$$

On a moving path $s$ comes from the direction of motion ($s = -\tanh(\dot q / v_0)$, so it turns over smoothly
at a reversal), and the whole command can be sent ahead by the servo's latency (the **lead**, 80 ms here).
Each term fixes something different: the spring removes the gravity sag (most visible after stopping), the band
removes the error that flips with direction, and only the lead removes the lag, which dominates while moving.
On M1's circle the three together cut the median tracking error from 2.0 / 3.8 mm to 0.6 / 0.7 mm (2 / 5 cm/s).

This is open-loop: it can only remove what the model predicts. What is left (0.6 mm) is what the model does
not have; a "virtual servo" (the model plus a 65 ms delay) predicted the ranking and the size of the other
variants beforehand, but 0.2-0.5 mm for the full one. The S5 demo uses the static part (spring and band) when
it stops over a point.

## 3. Kinematic calibration and what it can identify

FK uses the URDF's geometry and the joint readings. Its error has two kinds of source: **geometry** (link
lengths, axis directions) and **joint zeros** (the reading that means 0 rad). On a printed arm assembled by
hand, joint zeros are the first suspect: a few degrees of zero error on the elbow moves the tool by 2-3 cm.

The classic method (S2) touches known points with a tool and solves

$$\min_{\delta,\,p,\,R,\,t}\ \sum_i \big\| FK(q_i + \delta,\ p) - (R\,d_i + t) \big\|^2$$

for the zero offsets $\delta$, the pointer tip $p$ in the gripper frame, and the placement $(R, t)$ of the
points. The pointer and the placement are **nuisance parameters**: unknown, needed, not wanted.

**Identifiability.** A parameter can be fitted only if changing it changes the residuals in a way no other
parameter can copy. Formally: the Jacobian of the residuals with respect to all parameters must have full
column rank; a null-space direction is a combination of parameters the data cannot see. On this arm:

- a shoulder_pan offset turns the arm about the vertical axis, exactly like a turned target: pan $\equiv$ the
  placement's yaw;
- a wrist_roll offset turns the pointer about the roll axis, exactly like a pointer fixed elsewhere: roll
  $\equiv$ the pointer;
- a wrist_flex offset is a moved pointer as long as wrist_roll is constant, so touches must vary the roll;
- a shoulder_lift offset tilts the whole arm about a horizontal axis, which a tilted target would copy. The
  target lies flat on the table the base stands on, so only its yaw and position are fitted. **This
  constraint is what makes the lift offset observable.**

The same reasoning sets the parameters of the hand-eye fit (section 6).

**Why S2 stopped.** The residual after the fit was 3.5-3.8 mm against ~1 mm expected, random per touch,
and two runs disagreed on the pointer by 8.7 mm. The pointer (a toothpick in tape) slid, and a limp arm
guided by hand is bent by the hand: printed parts flex and gears open their play where the encoders cannot see
it (section 12). A touch measures the arm under an unknown load. A camera that looks while the arm holds itself
(S4) measures it under its own weight only, which the servo model already describes.

## 4. Where LeRobot puts zero, and why it was 7 degrees off

LeRobot's calibration has two steps: the user poses the arm in a "middle" pose (the servo's homing offset is
set so that pose reads half a turn), then moves every joint through its full range while LeRobot records the
minimum and maximum. In degrees mode a reading is

$$\theta = (\text{ticks} - m) \cdot \frac{360}{4095}, \qquad m = \frac{\text{min} + \text{max}}{2} .$$

So **zero is the middle of the recorded range**, not the posed middle. The URDF ("new calibration") puts zero
in the middle of each joint's nominal range. The two agree only if the physical range is symmetric about the
URDF's zero.

On this arm the elbow's recorded ends are, in true angles, +89.3 deg (folded: the forearm against the upper
arm, the rest pose, a hard stop) and -103.9 deg (stretched). The middle is 7.3 deg off the URDF's zero: a range
that is not symmetric (printing and assembly), read through a convention that assumes it is. The user did
nothing wrong, and recalibrating would give the same number.

The fix belongs one layer up. so101-core's joint map turns readings into model angles,
$q = \text{sign}\cdot\theta + \text{offset}$, so the measured offset (-0.127 rad) goes there and LeRobot's
calibration, and every dataset recorded through it, stays as it is. One consequence: the safety limits are in
model angles, and with an offset a model angle inside the URDF limits can mean a reading outside the range the
servo was calibrated over. `JointMap.model_limits` intersects the two, so the elbow's upper limit (with the 5
deg margin) fell from 91.8 to 84.5 deg: the same physical stop as before.

## 5. The camera model and its calibration

**Pinhole plus distortion.** A point $P_c = (X, Y, Z)$ in the camera frame projects to normalized coordinates
$(x, y) = (X/Z, Y/Z)$, which lens distortion moves (Brown-Conrady, the 5-parameter model OpenCV and the ZED
factory file use):

$$
\begin{aligned}
x_d &= x(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + 2p_1 xy + p_2(r^2 + 2x^2), \\
y_d &= y(1 + k_1 r^2 + k_2 r^4 + k_3 r^6) + p_1(r^2 + 2y^2) + 2p_2 xy, \qquad r^2 = x^2 + y^2,
\end{aligned}
$$

then to pixels $u = f_x x_d + c_x$, $v = f_y y_d + c_y$. The intrinsics are $K$ ($f_x, f_y, c_x, c_y$) and
$D$ ($k_1, k_2, p_1, p_2, k_3$).

**Zhang's method.** A planar target with known points, seen in several poses: each view's homography
constrains $K$; a closed form starts, and a nonlinear least squares over all views refines $K$, $D$ and every
view's pose by minimizing the **reprojection error** (pixels between detected and projected points). OpenCV's
`calibrateCamera` does this.

**ChArUco.** A chessboard whose white squares carry ArUco markers. The markers identify each corner, so a
partial view (board cut by the image edge, or by the gripper) still gives labelled corners; the corners
themselves are chessboard saddle points, refined to subpixel accuracy. Shown on an iPad pixel for pixel
(264 ppi), the 25 mm squares are as exact as the screen's pixel pitch; a ruler agreed to ~0.5 %. Intrinsics do
not depend on the square size at all, only lengths (baseline, triangulated sizes) do.

**What makes a calibration good.** Distortion grows with $r$; it is only measured where corners were seen.
Views that cover only the centre leave the corners extrapolated: in simulation ~10x worse there, and a
partial-coverage set biased $f_x$ by 1.2 %. 15 views covering the whole image did as well as 30. Corners on one
line fix no pose (a line is a 1-D family of solutions), so views whose corners do not span two rows and two
columns are rejected. Honest accuracy is the **cross-validated** reprojection error: calibrate on half the
views, measure on the other half (wrist camera: 0.23 px fitted, 0.24 px held out, so it does not overfit).

**Checking a factory calibration.** The ZED's factory parameters cannot be checked against a truth, but they
can be checked against data they were not fitted on: they reproject our views at 0.43 px against 0.37 px for
our own fit (held out), and triangulating the board with the factory stereo pair measures its 25 mm squares
as 25.08 mm (+0.3 %). That makes ZED depth metric to about 0.3 %, and the factory calibration the one to use.

## 6. Hand-eye calibration

The wrist camera is fixed to the gripper at an unknown pose $X = T_{\text{grip}\,\text{cam}}$. The board lies
still at an unknown $Y = T_{\text{base}\,\text{board}}$. For each arm pose $i$ the camera sees the board at
$C_i = T_{\text{cam}\,\text{board}}$ (by PnP: the board pose from its corners and the intrinsics). The chain
closes:

$$G_i\,X\,C_i = Y \quad \text{for every } i .$$

**The classic form.** For two poses $i, j$: $G_j^{-1}G_i\,X = X\,C_j C_i^{-1}$, that is

$$A X = X B, \qquad A = G_j^{-1} G_i \ \text{(gripper motion)}, \quad B = C_j C_i^{-1} \ \text{(camera motion)} .$$

The camera moves as the gripper does, seen through the fixed $X$. The rotation part, $R_A R_X = R_X R_B$,
says $R_A$ and $R_B$ are the same rotation in different frames, so their axes are related by $R_X$; Park and
Martin solve it in closed form on the rotation group (least squares over the axis pairs), then the
translation linearly. At least two motions with non-parallel rotation axes are needed. OpenCV's
`calibrateHandEye` provides it.

**Why only as a start.** The closed form first compresses each view into one pose $C_i$ and treats $G_i$ as
exact. Pixels are what the camera measures, and the arm's joints have errors the closed form cannot model.
Comparisons of hand-eye methods find a nonlinear fit of the reprojection error, with $X$ and $Y$ together,
the most accurate (Ali et al. 2019). S4 does that:

$$\min_{X,\,Y,\,\delta}\ \sum_i \sum_j \rho\Big( \big\| \pi\big(K, D,\ X^{-1} G(q_i + \delta)^{-1} Y\, P_j\big) - u_{ij} \big\|^2 \Big)$$

over every corner $P_j$ detected at pixel $u_{ij}$, with $\pi$ the camera model of section 5 and $\delta$ the
joint zero offsets of a chosen set of joints. $\rho$ is the soft-L1 loss ($2(\sqrt{1 + s} - 1)$), quadratic for
small residuals and linear for large ones, so a few bad corners cannot pull the fit. The parameters:

| Parameter | Count | Why this many |
|---|---:|---|
| $X$ | 6 | the camera mount |
| $Y$ per placement of the board | 3 + shared height | flat on the table: yaw and $x, y$; one height for all placements (same iPad, same table) |
| $\delta$ | 0-3 | lift, elbow, wrist_flex; pan $\equiv$ the board's yaw and roll $\equiv X$, so they cannot be fitted (section 3) |

**The flat board** is what makes a lift offset distinguishable from a tilted board (section 3). **The shared
height** came from S5: a placement seen from only two poses drifted 8-10 mm in height in its own fit, which
moved its corners on the table by several mm when seen through the ZED. With one height for all, a few bad
views can no longer hide in it.

**Uncertainty.** The covariance $\hat\sigma^2 (J^\top J)^{-1}$ at the solution gives standard errors. They
assume independent pixel errors; but a joint-reading error moves all corners of a view together, so the real
spread is larger than the formula says (simulated: 0.1 deg claimed, 0.7 deg off for 16 views). The honest
check is to fit on some data and predict other data (section 10).

**Offsets and the camera mount trade.** A wrist_flex offset rotates the camera about the wrist axis, which a
slightly different $X$ almost reproduces; lift and wrist offsets also trade against each other. Fitted per run,
lift and wrist swung by up to 3 deg while predicting the other run equally well; the elbow alone, known to
0.1 deg, predicted as well as all three. The simplest model that predicts as well is kept.

## 7. The ZED through a shared board

The ZED is fixed to the world, not to the arm (eye-to-hand). The usual way is a marker on the gripper seen by
the ZED in many poses. Here the board is already in the base frame ($Y$, from section 6), and the ZED sees it:

$$T_{\text{base}\,\text{zed}} = Y\, T_{\text{zed}\,\text{board}}^{-1} .$$

No marker on the gripper is needed. The weak part is the ZED's view of one board: a 20 x 12 cm board at
50 cm fixes its position well but its tilt only through its small extent, and the tilt error, multiplied by
the 50 cm lever, moves the ZED. One placement per run gave ZED positions 8 mm and 1.3 deg apart. The fix is
several placements spread over the table, fitted together: one ZED pose for all their corners
(`handeye.zed_fit`), each placement's board from the wrist chain.

## 8. From a pixel to a point on the table

A pixel gives a ray, not a point: every point along it projects to the same pixel. Undistorting the pixel
gives the normalized direction $d_c = (x, y, 1)$; in the base frame the ray is $o + s\,R\,d_c$ with
$(R, o)$ from $T_{\text{base}\,\text{zed}}$. If the point lies on a known horizontal plane $z = h$ (the
table, or the iPad's screen), $s = (h - o_z)/(R d_c)_z$ and the point follows.

**Sensitivity.** One pixel moves the point by about $Z/f$ divided by the cosine of the viewing angle: ~0.9 mm
per pixel here ($Z \approx 0.5$ m, $f \approx 535$ px). A plane height wrong by $\Delta h$ moves the point
horizontally by $\Delta h \tan\theta$, $\theta$ the ray's angle from vertical (~27 deg here): 1 cm of height,
5 mm on the table. The iPad's screen and the table are 11 mm apart, so the two planes are kept apart.

**Points off the plane** need depth: stereo triangulation, with a depth error

$$\delta Z = \frac{Z^2}{f\,B}\,\delta d$$

($B$ = 0.12 m baseline, $\delta d$ the disparity error): ~4 mm per pixel of disparity at 50 cm, ~1.2 mm for a
0.3 px match. Worse than the plane for points on the table, but the only option for points above it.

## 9. The error budget

The pointing error (the tool versus the point clicked in the ZED image) is split into terms, each measured
separately, and combined as a root sum of squares:

$$\sigma_\text{total}^2 = \sigma_\text{click}^2 + \sigma_\text{ZED pose}^2 + \sigma_\text{kinematics}^2 + \sigma_\text{servo}^2 .$$

| Term | Measured by |
|---|---|
| click | mm per ZED pixel at each placement, for a 1 px click error |
| ZED pose | the ZED's ray-plane points against the wrist chain's corners, the placement left out of the ZED fit |
| kinematics | each wrist view's board centre against its placement's fit (the arm's pose error at one pose) |
| servo | S1's residual (0.15 deg per joint) through the Jacobian at the pointing pose; and without compensation, S1's model error |

Adding squares assumes the terms are independent; they are not entirely (the ZED-pose term is measured against
the wrist chain, so it contains part of the kinematic error), which makes the total pessimistic. The terms
also depend on where the arm is: measured from S5's wide views they add to 6.5 mm, near the calibrated region
the first two are ~1 and ~2 mm.

## 10. Choosing between models: held-out data

Every extra parameter lowers the fitted error, so the fitted error cannot choose a model. M2 chooses by
prediction on data the fit did not see:

- **k-fold over views** (S3, S4): fit on 3/4 of the views, predict the rest, rotate.
- **Leave one run out** (S4) and **leave one placement out** (S4 over all runs, S5): fit on the other runs,
  then on the left-out run fit only its board placement (3 numbers) and measure the reprojection error. This
  asks the question that matters: does the model hold where the board lies elsewhere and the poses are new?

The rule (fixed before looking): the simplest model within 5 % of the best held-out error; extra parameters
only if they cut the error by 20 % or more. It kept the elbow offset alone (19.4 -> 4.7 px on the left-out
run), and it rejected link lengths, joint compliance and a gravity-dependent roll error in S5 (none improved
the left-out error).

## 11. Gravity, backlash and the folded arm

A gear train has backlash: a small angle over which the output can turn without the motor turning. Under
load the teeth press against one side and the backlash does not show. With no load nothing presses them; the
output sits somewhere in the gap and the position loop, seeing small errors either way, can hunt.

Folded at rest, the arm's centre of mass sits almost over the shoulder axis: the gravity torque on
shoulder_lift is ~0.03 N m, against 0.3-1 N m in working poses. On the way to rest the shoulder's torque passes
through this region. The dense recording (`so101-park approach`) found the shoulder oscillating at ~7 Hz with
0.1 deg there, and nothing larger: held still, at rest or standing up, no joint moved by one encoder count in
6 s. A pose that loads the shoulder in one direction (standing: upper arm back, forearm up, ~0.6 N m) is the
remedy if it ever matters.

## 12. What the measurements can and cannot see

The encoder sits on the servo's output shaft. It sees the gear train's backlash and the servo's own spring
(sections 1, 11). It does not see anything beyond the shaft:

- the horn and its screws, the printed brackets and links bending under load;
- the wrist camera's mount;
- a hand pushing the arm (S2).

These are why FK from the encoders can be off in a way the joint model cannot fix. S5 found single views off
by up to ~7 mm in far poses (shoulder folded far back, camera looking sideways), every corner shifted the same
way. The models tried (section 10 and the README) were offsets, link lengths, joint compliance and a roll error
from the camera's weight; only letting the camera tilt on its mount with gravity helped, and only partly. Rigid
by hand and with a fixed-focus camera, the cause stays open. The practical answer is to calibrate where the arm
works and to measure the error there.

Other limits of the measurements:

- **Model torques** come from the URDF masses; the wrist camera and its cable are not in them.
- **The iPad's scale** is the screen's pixel pitch, checked with a ruler to ~0.5 %.
- **Reprojection errors are in pixels**; a millimetre figure needs a distance (a pixel at 25 cm from the wrist
  camera is ~0.35 mm, at 50 cm from the ZED ~0.9 mm).

## 13. References

- Z. Zhang, "A flexible new technique for camera calibration", IEEE TPAMI 22(11), 2000.
- D. C. Brown, "Decentering distortion of lenses", Photogrammetric Engineering 32(3), 1966.
- S. Garrido-Jurado et al., "Automatic generation and detection of highly reliable fiducial markers under
  occlusion", Pattern Recognition 47(6), 2014 (ArUco); OpenCV's ChArUco documentation.
- R. Hartley, A. Zisserman, *Multiple View Geometry in Computer Vision*, 2nd ed., Cambridge, 2004 (PnP,
  triangulation, stereo depth).
- R. Y. Tsai, R. K. Lenz, "A new technique for fully autonomous and efficient 3D robotics hand/eye
  calibration", IEEE Trans. Robotics and Automation 5(3), 1989.
- F. C. Park, B. J. Martin, "Robot sensor calibration: solving AX = XB on the Euclidean group", IEEE Trans.
  Robotics and Automation 10(5), 1994.
- I. Ali, O. Suominen, A. Gotchev, E. R. Morales, "Methods for simultaneous robot-world-hand-eye calibration: a
  comparative study", Sensors 19(12), 2019.
- J. M. Hollerbach, C. W. Wampler, "The calibration index and taxonomy for robot kinematic calibration
  methods", IJRR 15(6), 1996 (identifiability).
- B. W. Mooring, Z. S. Roth, M. R. Driels, *Fundamentals of Manipulator Calibration*, Wiley, 1991.
- LeRobot, `lerobot/motors/motors_bus.py` (degrees normalization: zero at the middle of the recorded range).
