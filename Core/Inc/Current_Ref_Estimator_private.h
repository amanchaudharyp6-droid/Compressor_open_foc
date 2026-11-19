/*
 * File: Current_Ref_Estimator_private.h
 *
 * Code generated for Simulink model 'Current_Ref_Estimator'.
 *
 * Model version                  : 18.444
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Wed Jun  3 14:26:40 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef Current_Ref_Estimator_private_h_
#define Current_Ref_Estimator_private_h_
#include <stdbool.h>
#include <stdint.h>
#include "complex_types.h"
#include "Current_Ref_Estimator.h"
#include "Current_Ref_Estimator_types.h"

extern float rt_atan2f_snf(float u0, float u1);
extern float rt_hypotf_snf(float u0, float u1);
extern void Cu_Motor_Parameter_Degrade_Init(DW_Motor_Parameter_Degrade_Cu_T
  *localDW);
extern void Current_Motor_Parameter_Degrade(float rtu_Motor_Parameter, float
  rtu_Motor_P_Watts, float rtu_Motor_P_lim_Watts, float rtu_Motor_P_Max_Watts,
  float rtu_Percentage_decrease, float *rty_k_factor, float
  *rty_New_motor_parameter, DW_Motor_Parameter_Degrade_Cu_T *localDW);

#endif                                 /* Current_Ref_Estimator_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
