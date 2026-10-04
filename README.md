# Planar VTOL control

The normalized nonlinear plant has state [x, vx, y, vy, theta, omega],
with xdd = -u1 sin(theta), ydd = u1 cos(theta) - 1, theta_dd = u2.
The dynamic controller adds thrust and thrust rate as internal states.
It implements fourth-order tracking with gains [16, 32, 24, 8].

Run `regression` in MATLAB from this folder. MATLAB Coder and a supported
C compiler are required. Only controller.m is converted to C; the plant,
trajectory, and RK4 integration remain MATLAB. The generated C controller
is compiled into a MEX module and executed in the same simulation harness
for software-in-the-loop comparison. This is a MATLAB Coder workflow,
not a Simulink model or Simulink SIL block.

| Requirement | Test | Acceptance |
| --- | --- | --- |
| REQ-01 | Test_Setpoint | Final position error < 0.01 at t=16 s |
| REQ-02 | Test_Circle | Position RMS error < 0.01 over 0-16 s |
| REQ-03 | Test_Disturbance | Position error stays < 0.02 from 8-16 s |

All requirements apply to both MiL and SiL. Control signals u1/u2 and
all augmented states must agree to absolute tolerance 1e-8.
Point motion uses a seventh-degree polynomial from [1,1] to [3,2] in
6 seconds, then holds. The circle has center [2,1.5], radius 0.5, and
angular speed 0.35 rad/s, initialized on its nominal trajectory.
The disturbance adds [0.3,-0.2] position offset and 0.1 rad attitude offset.
Simulation uses continuous-feedback RK4 with step 0.005 s and ideal state
feedback. No actuator limits, estimator, sensor noise, or physical units
beyond the normalized model are included.

The GitHub workflow needs a MATLAB batch licensing token because it uses
MATLAB Coder. Configure repository secret `MLM_LICENSE_TOKEN`, or adapt
the workflow to a licensed self-hosted runner. See the official setup guide:
https://github.com/matlab-actions/setup-matlab#use-matlab-batch-licensing-token

Verified locally with MATLAB R2026b: all three tests passed, with MiL/SiL
control differences zero. Generated controller C is in
`build/controller/controller.c`. Regression saves traces and plots in `results/`.
