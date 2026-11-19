/*
 * File: FOC_Loop.h
 *
 * Code generated for Simulink model 'FOC_Loop'.
 *
 * Model version                  : 18.444
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Wed Jun  3 14:26:01 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef FOC_Loop_h_
#define FOC_Loop_h_
#ifndef FOC_Loop_COMMON_INCLUDES_
#define FOC_Loop_COMMON_INCLUDES_
#include <stdbool.h>
#include <stdint.h>
#include "complex_types.h"
#include "rt_nonfinite.h"
#include "math.h"
#include "main.h"
#endif                                 /* FOC_Loop_COMMON_INCLUDES_ */

#include "FOC_Loop_types.h"
#include "rtGetNaN.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  float Merge[6];                      /* 'Merge' (':5051') */
  float Merge2[6];                     /* 'Merge2' (':5053') */
  float Merge1[6];                     /* 'Merge1' (':5052') */
  float T_ref;                         /* 'FNR_switching' (':4893') */
} B_FOC_Loop_T;

/* Block states (default storage) for system 'FOC_Loop' */
typedef struct {
  Rate_limiterBus TmpRTBAtSpeed_LoopInport3_Buffe;/* synthesized block */
  float drive_sign;                   /* 'FNR_switching' (':4893') */
  float UnitDelay_DSTATE;              /* 'Unit Delay' (':3455:366') */
  float UnitDelay_DSTATE_f;            /* 'Unit Delay' (':3456:366') */
  float Integrator_DSTATE;             /* 'Integrator' (':42:843') */
  float Filter_DSTATE;                 /* 'Filter' (':42:737') */
  float Integrator_DSTATE_h;           /* 'Integrator' (':5:843') */
  float Filter_DSTATE_d;               /* 'Filter' (':5:737') */
  float Delay_DSTATE;                  /* 'Delay' (':5248') */
  float Delay_DSTATE_c;                /* 'Delay' (':3597') */
  float Integrator_DSTATE_g;           /* 'Integrator' (':4086:843') */
  float Filter_DSTATE_c;               /* 'Filter' (':4086:737') */
  float TmpRTBAtMerge1Outport1_Buffer[6];/* synthesized block */
  float RateTransition_Buffer0;        /* 'Rate Transition' (':4080') */
  float TmpRTBAtDataTypeConversion1Outp;/* synthesized block */
  float TmpRTBAtSpeed_LoopInport1_Buffe;/* synthesized block */
  float TmpRTBAtSpeed_LoopInport6_Buffe;/* synthesized block */
  float wm_input_Buffer;               /* synthesized block */
  float counter;                       /* 'FNR_switching' (':4893') */
  float init_speed;                    /* 'FNR_switching' (':4893') */
  float mid_speed;                     /* 'FNR_switching' (':4893') */
  float rpm_prev;                      /* 'FNR_switching' (':4893') */
  float max_negTorque;                 /* 'FNR_switching' (':4893') */
  uint32_t durationCounter_1;          /* 'FNR_switching' (':4893') */
  uint32_t durationCounter_2;          /* 'FNR_switching' (':4893') */
  uint32_t durationCounter_1_m;        /* 'FNR_switching' (':4893') */
  uint32_t durationCounter_1_l;        /* 'FNR_switching' (':4893') */
  uint32_t durationCounter_2_e;        /* 'FNR_switching' (':4893') */
  uint8_t is_active_c2_FOC_Loop;       /* 'FNR_switching' (':4893') */
  uint8_t is_c2_FOC_Loop;              /* 'FNR_switching' (':4893') */
  uint8_t is_Neutral;                  /* 'FNR_switching' (':4893') */
  bool icLoad;                         /* 'Delay' (':5248') */
  bool icLoad_k;                       /* 'Delay' (':3597') */
} DW_FOC_Loop_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  Rate_limiterBus Rate_limiter;        /* 'Rate_limiter' (':5680') */
  float angle;                         /* 'angle' (':5681') */
  PID_MTPABus MTPA_PID;                /* 'MTPA_PID' (':5682') */
  float ActualSpeed_mech_radsec;    /* 'Actual Speed_mech_rad//sec' (':5683') */
  float Ref_Speed_mech_rpm;            /* 'Ref_Speed_mech_rpm' (':5684') */
  IIR_FilterBus IIR_Filter_Coefficient;/* 'IIR_Filter_Coefficient' (':5685') */
  float RPM_min_limit;                 /* 'RPM_min_limit' (':5686') */
  float Drive_State;                   /* 'Drive_State' (':5687') */
  float PhaseCurrent[3];               /* 'Phase Current' (':5688') */
  float Iq_Torque_ratio;               /* 'Iq_Torque_ratio' (':5689') */
  bool Speed_1_Torque_0;               /* 'Speed_1_Torque_0' (':5690') */
  float BusVoltage_V;                  /* 'Bus_Voltage' (':5691') */
  float RefTrq;                        /* 'T_ref' (':5692') */
  Dead_TimeBus Dead_Time_Compensator;  /* 'Dead_Time_Compensator' (':5693') */
  ThrottleBus Throttle_input;          /* 'Throttle_input' (':5694') */
  RegenBus Regen;                      /* 'Regen' (':5695') */
  float Id_ref_final;                  /* 'Id_ref_final' (':5696') */
  float Iq_ref_final;                  /* 'Iq_ref_final' (':5697') */
  float Ld;                            /* 'Ld' (':5698') */
  float Lq;                            /* 'Lq' (':5699') */
  float Pole_Pairs;                    /* 'Pole_Pairs' (':5700') */
  float Rs;                            /* 'Rs' (':5701') */
  float Lambda;                        /* 'Lambda' (':5702') */
} ExtU_FOC_Loop_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  float Va;                            /* 'Va' (':5704') */
  float Vb;                            /* 'Vb' (':5705') */
  float Vc;                            /* 'Vc' (':5706') */
  bool PID_Switch;                     /* 'PID_Switch' (':5707') */
  float Id;                            /* 'Id' (':5708') */
  float Iq;                            /* 'Iq' (':5709') */
  float Ref_speed_out_RPM;             /* 'Ref_speed_out_RPM' (':5710') */
  bool Throttle_en;                    /* 'Throttle_en' (':5711') */
  float Vd_PID;                        /* 'Vd_PID' (':5712') */
  ThrottleState Throttle_State;        /* 'Throttle_State' (':5713') */
  float Vq_PID;                        /* 'Vq_PID' (':5714') */
  float Speed_error;                   /* 'Speed_error' (':5715') */
  float Vd;                            /* 'Vd' (':5716') */
  bool Throttle_error;                 /* 'Throttle_error' (':5717') */
  float Vq;                            /* 'Vq' (':5718') */
  float T_gen;                         /* 'T_gen' (':5719') */
  float T_ref_FNR;                     /* 'T_ref_FNR' (':5720') */
  float Id_error;                      /* 'Id_error' (':5721') */
  bool Regen_Mode;                     /* 'Regen_Mode' (':5722') */
  float Iq_gen;                        /* 'Iq_gen' (':5723') */
  float Iq_error;                      /* 'Iq_error' (':5724') */
  float Actual_torque_from_meas; 
  float Vd_next;      /* 'Actual_torque_from_meas' (':5725') */
} ExtY_FOC_Loop_T;

/* Real-time Model Data Structure */
struct tag_RTM_FOC_Loop_T {
  const char * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint8_t TID0_1;
    } RateInteraction;
  } Timing;
};

/* Block signals (default storage) */
extern B_FOC_Loop_T FOC_Loop_B;

/* Block states (default storage) */
extern DW_FOC_Loop_T FOC_Loop_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_FOC_Loop_T FOC_Loop_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_FOC_Loop_T FOC_Loop_Y;

/* Model entry point functions */
extern void FOC_Loop_initialize(void);
extern void FOC_Loop_step0(void);      /* Sample time: [0.0001s, 0.0s] */
extern void FOC_Loop_step1(void);      /* Sample time: [0.001s, 0.0s] */
extern void FOC_Loop_terminate(void);

/* Real-time Model object */
extern RT_MODEL_FOC_Loop_T *const FOC_Loop_M;

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
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Unused code path elimination
 * Block synthesized block : Eliminated since input and output rates are identical
 * Block 'Kalpha' (':710:652') : Eliminated nontunable gain of 1
 * Block 'Kbeta' (':710:653') : Eliminated nontunable gain of 1
 * Block 'Kb' (':42:616') : Eliminated nontunable gain of 1
 * Block 'Kb' (':5:616') : Eliminated nontunable gain of 1
 * Block synthesized block : Eliminate redundant data type conversion
 * Block 'Ka' (':527:565') : Eliminated nontunable gain of 1
 * Block 'Kb' (':527:566') : Eliminated nontunable gain of 1
 * Block 'Kc' (':527:567') : Eliminated nontunable gain of 1
 * Block synthesized block : Eliminated since input and output rates are identical
 * Block 'Kb' (':4086:616') : Eliminated nontunable gain of 1
 * Block 'FilterConstant' (':3455:478') : Unused code path elimination
 * Block 'OneMinusFilterConstant' (':3455:480') : Unused code path elimination
 * Block 'UseInputPort' (':3455:484') : Unused code path elimination
 * Block 'FilterConstant' (':3456:478') : Unused code path elimination
 * Block 'OneMinusFilterConstant' (':3456:480') : Unused code path elimination
 * Block 'UseInputPort' (':3456:484') : Unused code path elimination
 * Block 'Offset' (':712:512') : Unused code path elimination
 * Block synthesized block : Unused code path elimination
 * Block 'Offset' (':482:753') : Unused code path elimination
 * Block synthesized block : Unused code path elimination
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
 * hilite_system('IPMSM_motor_Equation_FOC/FOC_MTPA_FWC_FF/FOC_Loop')    - opens subsystem IPMSM_motor_Equation_FOC/FOC_MTPA_FWC_FF/FOC_Loop
 * hilite_system('IPMSM_motor_Equation_FOC/FOC_MTPA_FWC_FF/FOC_Loop/Kp') - opens and selects block Kp
 */
#endif                                 /* FOC_Loop_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
