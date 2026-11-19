/*
 * File: Current_Ref_Estimator_types.h
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

#ifndef Current_Ref_Estimator_types_h_
#define Current_Ref_Estimator_types_h_
#ifndef DEFINED_TYPEDEF_FOR_Id_Iq_MTPA_limitBus_
#define DEFINED_TYPEDEF_FOR_Id_Iq_MTPA_limitBus_

typedef struct {
  float Id_up_limit;
  float Id_low_limit;
  float Iq_up_limit;
  float Iq_low_limit;
} Id_Iq_MTPA_limitBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Rate_limiterBus_
#define DEFINED_TYPEDEF_FOR_Rate_limiterBus_

typedef struct {
  float Id_ramp_up;
  float Id_ramp_down;
  float Iq_ramp_up;
  float Iq_ramp_down;
  float Torque_gen_ramp_up;
  float Torque_gen_ramp_down;
  float Ref_Speed_rate_up;
  float Ref_Speed_rate_down;
  float Ref_Speed_rate_down_neutral;
} Rate_limiterBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Motor_ParametersBus_
#define DEFINED_TYPEDEF_FOR_Motor_ParametersBus_

typedef struct {
  float Pole_Pairs;
  float Rs;
  float Ld;
  float Lq;
  float Lambda;
  float Motor_P_lim_Watts;
  float Motor_P_Max_Watts;
  float Ld_Percentage;
  float Lq_Percentage;
  float Flux_Percentage;
} Motor_ParametersBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_FWCBus_
#define DEFINED_TYPEDEF_FOR_FWCBus_

typedef struct {
  float Kfw;
  float Kaw;
  float FW_Int_reset_factor;
  float M2F_factor;
  float F2M_factor;
} FWCBus;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_Current_Ref_Estimator_T RT_MODEL_Current_Ref_Estimato_T;

#endif                                 /* Current_Ref_Estimator_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
