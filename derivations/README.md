# Exercise 8.2 symbolic derivations

Two files derive the planar VTOL mathematics independently of Simulink:

- `derive_flatness.m`: plant flat-output derivatives, all six state
  parameterizations, thrust u1, thrust rate and angular acceleration u2.
- `derive_dynamic_tracking.m`: the eight-state thrust extension,
  fourth-order decoupling matrix, and inversion for [d2u1; u2], including
  substitution of snap commands [v1; v2].

First run `regression` from the project root to create the two trajectory
datasets. Then run `run_derivations` in this folder using Symbolic Math Toolbox. This
optional runner checks identities, exports numeric MATLAB evaluators into
`generated/`, and compares feedforward against the existing Simulink source
using both saved trajectories. Results are saved to `symbolic_results.mat`.

The model uses epsilon=0 and normalized gravity g=1. Derivations retain g
as a positive constant. Thrust must be positive and nonzero. Angle
derivatives apply on a continuous atan2 branch, away from angle wrapping.

For dynamic tracking, xb1=u1, xb2=du1 and ub1=d2u1. Only thrust is extended.
The state-based inversion is:

    ub1 = -sin(theta)*v1 + cos(theta)*v2 + u1*omega^2
    u2 = (-cos(theta)*v1 - sin(theta)*v2 - 2*du1*omega)/u1

Here vi = reference_snap_i - 8*jerk_error_i - 24*acceleration_error_i
- 32*velocity_error_i - 16*position_error_i. These files prepare the
dynamic tracking mathematics; they do not modify the feedforward-only SLX.

The original four-file derivation layout is retained locally under `archive/`
for reference and is not published or used by the current runner. Generated MATLAB evaluators
are separate from Simulink Coder controller-only C generation.
