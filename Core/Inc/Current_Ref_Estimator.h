/*
 * File: Current_Ref_Estimator.h
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

#ifndef Current_Ref_Estimator_h_
#define Current_Ref_Estimator_h_
#ifndef Current_Ref_Estimator_COMMON_INCLUDES_
#define Current_Ref_Estimator_COMMON_INCLUDES_
#include <stdbool.h>
#include <stdint.h>
#include "complex_types.h"
#include "rt_nonfinite.h"
#include "math.h"
#include "main.h"
#endif                              /* Current_Ref_Estimator_COMMON_INCLUDES_ */

#include "Current_Ref_Estimator_types.h"
#include "rtGetNaN.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system 'Motor_Parameter_Degrade' (':5261') */
typedef struct {
  float DelayOneStep_DSTATE;           /* 'Delay One Step' (':5261:36') */
  bool icLoad;                         /* 'Delay One Step' (':5261:36') */
} DW_Motor_Parameter_Degrade_Cu_T;

/* Block signals (default storage) */
typedef struct {
  float Merge[2];                      /* 'Merge' (':5530') */
} B_Current_Ref_Estimator_T;

/* Block states (default storage) for system 'Current_Ref_Estimator' */
typedef struct {
  float Delay1_DSTATE;                 /* 'Delay1' (':5311') */
  float Delay_DSTATE;                  /* 'Delay' (':5531:19') */
  float Delay_DSTATE_e;                /* 'Delay' (':5532:19') */
  float UnitDelay_DSTATE;              /* 'Unit Delay' (':5463') */
  float Vd_UnitDelay_DSTATE;            /* Previous d-axis voltage component */
  float Vq_UnitDelay_DSTATE;            /* Previous q-axis voltage component */
  float DiscreteTimeIntegrator_DSTATE;/* 'Discrete-Time Integrator' (':5449') */
  int8_t DiscreteTimeIntegrator_PrevRese;
                                      /* 'Discrete-Time Integrator' (':5449') */
  uint8_t is_active_c1_Current_Ref_Estima;/* 'Chart' (':5546') */
  uint8_t is_c1_Current_Ref_Estimator; /* 'Chart' (':5546') */
  bool icLoad;                         /* 'Delay' (':5531:19') */
  bool icLoad_b;                       /* 'Delay' (':5532:19') */
  DW_Motor_Parameter_Degrade_Cu_T Motor_Parameter_Degrade2;/* 'Motor_Parameter_Degrade' (':5261') */
  DW_Motor_Parameter_Degrade_Cu_T Motor_Parameter_Degrade1;/* 'Motor_Parameter_Degrade' (':5261') */
  DW_Motor_Parameter_Degrade_Cu_T Motor_Parameter_Degrade;/* 'Motor_Parameter_Degrade' (':5261') */
} DW_Current_Ref_Estimator_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  float Actual_Speed_mech;             /* 'Actual_Speed_mech' (':5680') */
  Id_Iq_MTPA_limitBus Id_Iq_MTPA_limit;/* 'Id_Iq_MTPA_limit' (':5681') */
  Rate_limiterBus Rate_limiter;        /* 'Rate_limiter' (':5682') */
  Motor_ParametersBus Motor_Parameters;/* 'Motor_Parameters' (':5683') */
  float T_ref_final;                   /* 'T_ref_FNR' (':5684') */
  float Is_max;                        /* 'Is_max' (':5685') */
  FWCBus FWC_values;                   /* 'FWC_values' (':5686') */
  float BusVoltage_V;                  /* 'Bus_Voltage' (':5687') */
  float Actual_torque_from_meas;       /* 'Actual_torque_from_meas' (':5688') */
} ExtU_Current_Ref_Estimator_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  float Id_ref_final;                  /* 'Id_ref_final' (':5689') */
  float Iq_ref_final;                  /* 'Iq_ref_final' (':5690') */
  float Id_ref_MTPA;                   /* 'Id_ref_MTPA' (':5691') */
  float Iq_ref_MTPA;                   /* 'Iq_ref_MTPA' (':5692') */
  float V_s;                           /* 'V_s' (':5693') */
  float Ld;                            /* 'Ld' (':5695') */
  float Lq;                            /* 'Lq' (':5696') */
  float Pole_Pairs;                    /* 'Pole_Pairs' (':5697') */
  float Rs;                            /* 'Rs' (':5698') */
  float Lambda;                        /* 'Lambda' (':5699') */
  float V_max;                         /* 'V_max' (':5700') */
  float Ldb;                           /* 'Ldb' (':5701') */
  float Lqb;                           /* 'Lqb' (':5702') */
  bool MTPA_FWC_condition;             /* 'MTPA_FWC_condition' (':5703') */
  float Lambdab;                       /* 'Lambdab' (':5704') */
  float Actual_torque_from_ref;        /* 'Actual_torque_from_ref' (':5705') */
  float Motor_P_Watts;                 /* 'Motor_P_Watts' (':5706') */
  float I_mag;                         /* 'I_mag' (':5707') */
  float I_qo;                          /* 'I_qo' (':5708') */
} ExtY_Current_Ref_Estimator_T;

/* Real-time Model Data Structure */
struct tag_RTM_Current_Ref_Estimator_T {
  const char * volatile errorStatus;
};

/* Block signals (default storage) */
extern B_Current_Ref_Estimator_T Current_Ref_Estimator_B;

/* Block states (default storage) */
extern DW_Current_Ref_Estimator_T Current_Ref_Estimator_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_Current_Ref_Estimator_T Current_Ref_Estimator_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Current_Ref_Estimator_T Current_Ref_Estimator_Y;

/* Model entry point functions */
extern void Current_Ref_Estimator_initialize(void);
extern void Current_Ref_Estimator_step(void);
extern void Current_Ref_Estimator_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Current_Ref_Estimato_T *const Current_Ref_Estimator_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Eliminated since input and output rates are identical
 * Block synthesized block : Eliminated since input and output rates are identical
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is 'block_name' ('SID'), where block_name is the name of the block
 * and SID is the Simulink identifier of the block without the model name.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('IPMSM_motor_Equation_FOC/FOC_MTPA_FWC_FF/Current_Ref_Estimator')    - opens subsystem IPMSM_motor_Equation_FOC/FOC_MTPA_FWC_FF/Current_Ref_Estimator
 * hilite_system('IPMSM_motor_Equation_FOC/FOC_MTPA_FWC_FF/Current_Ref_Estimator/Kp') - opens and selects block Kp
 */
#endif                                 /* Current_Ref_Estimator_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
