/*
 * vtol_controller.c
 *
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * Code generation for model "vtol_controller".
 *
 * Model version              : 1.1
 * Simulink Coder version : 26.2 (R2026b) 22-May-2026
 * C source code generated on : Sun Oct  4 20:13:43 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "vtol_controller.h"
#include <math.h>
#include "rtwtypes.h"
#include <string.h>

/* External inputs (root inport signals with default storage) */
ExtU_vtol_controller_T vtol_controller_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_vtol_controller_T vtol_controller_Y;

/* Real-time model */
static RT_MODEL_vtol_controller_T vtol_controller_M_;
RT_MODEL_vtol_controller_T *const vtol_controller_M = &vtol_controller_M_;

/* Model step function */
void vtol_controller_step(void)
{
  real_T den;

  /* MATLAB Function: '<S1>/feedforward_control' incorporates:
   *  Inport: '<Root>/z1_d_x'
   *  Inport: '<Root>/z2_d_y'
   */
  den = (vtol_controller_U.z2_d_y[2] + 1.0) * (vtol_controller_U.z2_d_y[2] + 1.0)
    + vtol_controller_U.z1_d_x[2] * vtol_controller_U.z1_d_x[2];

  /* Outport: '<Root>/u1_ff' incorporates:
   *  MATLAB Function: '<S1>/feedforward_control'
   */
  vtol_controller_Y.u1_ff = sqrt(den);

  /* Outport: '<Root>/u2_ff' incorporates:
   *  Inport: '<Root>/z1_d_x'
   *  Inport: '<Root>/z2_d_y'
   *  MATLAB Function: '<S1>/feedforward_control'
   */
  vtol_controller_Y.u2_ff = ((vtol_controller_U.z2_d_y[2] + 1.0) *
    vtol_controller_U.z2_d_y[3] + vtol_controller_U.z1_d_x[2] *
    vtol_controller_U.z1_d_x[3]) * 2.0 * ((vtol_controller_U.z2_d_y[2] + 1.0) *
    vtol_controller_U.z1_d_x[3] - vtol_controller_U.z1_d_x[2] *
    vtol_controller_U.z2_d_y[3]) / (den * den) - ((vtol_controller_U.z2_d_y[2] +
    1.0) * vtol_controller_U.z1_d_x[4] - vtol_controller_U.z1_d_x[2] *
    vtol_controller_U.z2_d_y[4]) / den;
}

/* Model initialize function */
void vtol_controller_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(vtol_controller_M, (NULL));

  /* external inputs */
  (void)memset(&vtol_controller_U, 0, sizeof(ExtU_vtol_controller_T));

  /* external outputs */
  (void)memset(&vtol_controller_Y, 0, sizeof(ExtY_vtol_controller_T));
}

/* Model terminate function */
void vtol_controller_terminate(void)
{
  /* (no terminate code required) */
}
