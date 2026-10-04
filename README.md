# Planar VTOL: Feedforward, Requirements, MiL / Custom Host SiL

MATLAB R2026b, Simulink, Requirements Toolbox, Simulink Coder, generated C,
Git and GitHub Actions. Embedded Coder and UAV Toolbox are not used.

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

One workflow runs `regression` on every push, pull request and manual dispatch.
It regenerates controller C, runs all checks and uploads reports/source artifacts.

This repository is private, and code generation uses a transformation product.
Hosted MATLAB therefore needs a MathWorks batch licensing token. Add it as the
repository Actions secret `MLM_LICENSE_TOKEN`; do not put it in a file or commit.
Without it, CI deliberately fails at the licensing check rather than claiming
that tests passed. Consult your license administrator or MathWorks about an
eligible batch token or a licensed self-hosted runner. The
[batch-token pilot](https://www.mathworks.com/support/batch-tokens.html)
is currently not accepting new requests.
The token must cover MATLAB, Simulink, Simulink Coder and Requirements Toolbox
and their dependencies. Availability depends on the license entitlement.

## Reference Material

`derivations/` and `simulink_sources/` keep the symbolic work and readable block
sources. The local `NCS2SIM/` folder is the user's original crane reference and
is not part of the published VTOL regression. Historical work was archived
outside the project during cleanup, not required to run regression.
