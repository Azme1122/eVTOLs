/*
 * vtol_controller.h
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

#ifndef vtol_controller_h_
#define vtol_controller_h_
#ifndef vtol_controller_COMMON_INCLUDES_
#define vtol_controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* vtol_controller_COMMON_INCLUDES_ */

#include "vtol_controller_types.h"
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T z1_d_x[5];                    /* '<Root>/z1_d_x' */
  real_T z2_d_y[5];                    /* '<Root>/z2_d_y' */
} ExtU_vtol_controller_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T u1_ff;                        /* '<Root>/u1_ff' */
  real_T u2_ff;                        /* '<Root>/u2_ff' */
} ExtY_vtol_controller_T;

/* Real-time Model Data Structure */
struct tag_RTM_vtol_controller_T {
  const char_T *errorStatus;
};

/* External inputs (root inport signals with default storage) */
extern ExtU_vtol_controller_T vtol_controller_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_vtol_controller_T vtol_controller_Y;

/* Model entry point functions */
extern void vtol_controller_initialize(void);
extern void vtol_controller_step(void);
extern void vtol_controller_terminate(void);

/* Real-time Model object */
extern RT_MODEL_vtol_controller_T *const vtol_controller_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'vtol_controller'
 * '<S1>'   : 'vtol_controller/Controller'
 * '<S2>'   : 'vtol_controller/Controller/feedforward_control'
 */
#endif                                 /* vtol_controller_h_ */
