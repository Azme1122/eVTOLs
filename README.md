# eVTOL Flight Control: Model-Based Design, MiL/SiL and CI

An in-progress flight-control project, starting with the nonlinear planar VTOL
from Exercise 8.2. The current implementation is a **2D, three-degree-of-freedom
plant with six states**, not a full 6-DOF multicopter. The long-term goal is a
multicopter simulation and verification workflow with actuator modeling,
requirements traceability, generated C, automated testing and PX4 SITL.

## Implemented

| Area | Current implementation |
| --- | --- |
| Simulink plant | `eVTOL.slx`: nonlinear planar dynamics and a six-state Integrator for position, velocity, attitude and angular rate. |
| Reference generation | Separate point-to-point and circular trajectory blocks, with derivatives through fourth order. Point-to-point motion is from `[1,1]` to `[3,2]` over 10 time units. |
| Controller | Flatness-based open-loop feedforward for thrust `u1` and angular acceleration `u2`, with matched initial states and a singularity guard. |
| Visualization | Actual and desired paths, state plots and aircraft motion visualization. |
| Requirements | One editable position-error requirement in Requirements Toolbox, linked to a MATLAB unit test with verification status. |
| Generated C | Controller-only C generation using Simulink Coder; the plant is not generated. |
| MiL/SiL comparison | Custom host SiL runs the compiled controller C against the same Simulink plant and compares `u1`, `u2` and all six states with MiL. |
| Regression | One `regression` entry point, 14 checks, PASS/FAIL output, comparison plots and machine-readable reports. All 14 checks passed locally. |
| GitHub Actions | Workflow configured for pushes, pull requests and manual runs. Full hosted execution is pending licensing configuration; it is not yet a passing hosted regression. |

**Tools used:** MATLAB R2026b, Simulink, Requirements Toolbox, Simulink Coder,
C, a host C compiler, Git and GitHub Actions workflow configuration.
**Planned tools:** UAV Toolbox, Embedded Coder and PX4 SITL/Gazebo.
These planned tools have not yet been used in the implemented workflow.

## Next Steps and Roadmap

1. **Dynamic flatness-based tracking:** integrate the two thrust-extension
   states (`u1` and its derivative), feedback gains `16, 32, 24, 8`, and
   initial-disturbance tests. Symbolic dynamic-extension derivations already
   exist, but this feedback controller is not yet integrated into the model.
2. **Broader requirements and tests:** add measurable convergence, disturbance
   rejection and actuator-limit criteria, extending requirement-to-test links.
3. **Complete hosted CI:** configure eligible MATLAB licensing or a licensed
   self-hosted runner, then confirm code generation and regression on GitHub.
4. **6-DOF multicopter model:** add 3D translational and rotational dynamics,
   rotor thrust/torque, motor dynamics, saturation and control allocation.
   The controller must also be adapted to the 3D plant; the planar controller
   is not a drop-in multicopter controller.
5. **Embedded Coder verification:** when licensed, add production-oriented
   controller C generation and MathWorks' built-in SIL back-to-back testing.
6. **UAV Toolbox and PX4 SITL/Gazebo:** introduce UAV scenarios and compatible
   controller interfaces, followed by hover, trajectory and wind-gust tests.

The chosen controller direction is flatness-based control. The cascaded PID/LQR
controller and EKF from the original project vision are **not implemented and
are outside the immediate scope**. Current tests use an ideal, dimensionless
plant with exact state knowledge and no motor dynamics, wind or sensor noise.

## Run

Open the repository folder in MATLAB and run:

```matlab
regression
```

The runner executes the linked requirement test, generates controller-only C,
compiles it with a C compiler, runs both point-to-point and complete-circle
simulations, compares MiL/SiL outputs and states, and prints PASS/FAIL.
It throws an error when a build or any check fails, so CI fails too.

For the requirement alone, run `verify_evtol_requirements`.
For the original aircraft plots/animation, open and run `eVTOL.slx`.

## Requirement

REQ-01: at every logged simulation time, the Euclidean actual-to-desired
position error must be strictly below `PositionErrorLimit`, initially 0.01.
The attribute is stored in `eVTOL_requirements.slreqx` and is editable under
Custom Attributes when REQ-01 is selected in Requirements Editor.

`test_evtol_position.m` is the linked MATLAB unit test. Run it through
`verify_evtol_requirements` or Requirements Editor's Run Tests, and enable
the Verification Status column. The regression also applies the same position
limit to its compiled-controller simulations.

Both paths use matched initial position, velocity, attitude and angular rate,
with no disturbances. All exercise quantities are dimensionless. The current
controller is feedforward, not dynamic tracking feedback or an estimator.

## Code Generation Boundary

`verification/build_controller_c.m` copies only `eVTOL/Flatness Feedforward`
into a temporary controller-only model and uses Simulink Coder's `grt.tlc`.
The plant, six-state Integrator, reference generators and plotting blocks
are never part of this code-generation target.

- Readable controller C and headers: `generated/controller/`.
- Build outputs and transient controller/test models: `build/` (ignored).
- Main simulation model: `eVTOL.slx` (not modified by regression).

GRT is a prototyping target, not an Embedded Coder production target. Generated
files retain the licensing notices emitted by the installed MATLAB license.

## What Custom Host SiL Means

MathWorks' built-in SIL/PIL execution modes require Embedded Coder, which is
not available here. Instead, `verification/controller_mex_adapter.c` wraps
the actual Simulink-generated controller C in a host MEX binary.
`verification/evtol_sil_sfunction.m` calls that binary at each solver evaluation
in a temporary copy of the same Simulink VTOL plant.

MiL uses the model's feedforward block; custom host SiL uses compiled controller C.
Both use the same references, initial states, ode4 solver and 0.005 fixed step.
No plant code is generated. The adapter retains the zero-thrust singularity guard.
The controller is stateless and initialized independently at each call.

The absolute maximum differences for u1, u2 and all six states must each
be below 1e-10. Position error must remain below REQ-01 in both executions.
Hover and singularity-guard checks are included. No disturbance rejection,
hardware deployment, timing verification or certification is claimed.

## Reports

`results/regression.csv`, `regression.mat`, and `regression.xml` contain the
summary. `Point_mil_sil.*` and `Circle_mil_sil.*` contain time histories and
comparison plots. JUnit XML can be consumed by CI reporting tools.

## GitHub Actions

One workflow is configured to run `regression` on every push, pull request and
manual dispatch. After licensing is available, it regenerates controller C,
runs all checks and uploads reports/source artifacts.

This repository is **public**. Public visibility enables automatic licensing
for supported non-transformation products, but does not by itself provide
licensing for the code-generation products used by the full workflow. See the
[MATLAB Actions licensing guidance](https://github.com/matlab-actions/setup-matlab#licensing).

The current workflow requires a MathWorks batch licensing token. Add it as the
repository Actions secret `MLM_LICENSE_TOKEN`; do not put it in a file or commit.
Without it, CI deliberately fails at the licensing check rather than claiming
that tests passed. No successful hosted regression is claimed yet.
Consult your license administrator or MathWorks about
[batch licensing availability](https://www.mathworks.com/support/batch-tokens.html)
or a licensed self-hosted runner. Licensing must cover MATLAB, Simulink,
Simulink Coder, Requirements Toolbox and their required dependencies, including
MATLAB Coder. Availability depends on the license entitlement.

## Reference Material

`derivations/` and `simulink_sources/` keep the symbolic work and readable block
sources. The local `NCS2SIM/` folder is the user's original crane reference and
is not part of the published VTOL regression. Historical work was archived
outside the project during cleanup, not required to run regression.
