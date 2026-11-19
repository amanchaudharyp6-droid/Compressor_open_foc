/*
 * File: FOC_Loop.c
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

#include "FOC_Loop.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "FOC_Loop_private.h"
#include "FOC_Loop_types.h"
#include "rt_nonfinite.h"
#include "Vdq_Circle_Limit.h"	/* HAND-EDIT: see FOC_VDQ_CIRCLE_LIMIT */
#include "Isense_Delay_Comp.h"	/* HAND-EDIT: see FOC_ISENSE_DELAY_COMP */

/* Named constants for Chart: ''FNR_switching' (':4893')' */
#define FOC_Loop_IN_Forward            ((uint8_t)1U)
#define FOC_Loop_IN_Initial            ((uint8_t)2U)
#define FOC_Loop_IN_Invalid_Transition ((uint8_t)1U)
#define FOC_Loop_IN_NO_ACTIVE_CHILD    ((uint8_t)0U)
#define FOC_Loop_IN_Neutral            ((uint8_t)3U)
#define FOC_Loop_IN_Normal             ((uint8_t)2U)
#define FOC_Loop_IN_Regen_mode1        ((uint8_t)4U)
#define FOC_Loop_IN_Reverse            ((uint8_t)5U)
#define FOC_Loop_IN_Throttle_error     ((uint8_t)6U)

/* Block signals (default storage) */
B_FOC_Loop_T FOC_Loop_B;

/* Block states (default storage) */
DW_FOC_Loop_T FOC_Loop_DW;

/* External inputs (root inport signals with default storage) */
ExtU_FOC_Loop_T FOC_Loop_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_FOC_Loop_T FOC_Loop_Y;

/* Real-time model */
static RT_MODEL_FOC_Loop_T FOC_Loop_M_;
RT_MODEL_FOC_Loop_T *const FOC_Loop_M = &FOC_Loop_M_;

/* Forward declaration for local functions */
static void FOC_Loop_Regen_mode1(const float *Abs, const bool *FNR_Active);
// float rt_hypotf_snf(float u0, float u1)
// {
//   float a;
//   float b;
//   float y;
//   a = fabsf(u0);
//   b = fabsf(u1);
//   if (a < b) {
//     a /= b;
//     y = sqrtf(a * a + 1.0F) * b;
//   } else if (a > b) {
//     b /= a;
//     y = sqrtf(b * b + 1.0F) * a;
//   } else if (rtIsNaNF(b)) {
//     y = (rtNaNF);
//   } else {
//     y = a * 1.41421354F;
//   }

//   return y;
// }

/* Function for Chart: ''FNR_switching' (':4893')' */
static void FOC_Loop_Regen_mode1(const float *Abs, const bool *FNR_Active)
{
  float ref_speed;
  float shape;
  float x2;
  bool tmp;
  tmp = !*FNR_Active;

  /* Inport: 'Regen' (':5695') */
  /* HAND-EDIT (2026-07-29, regen-latch throttle-dead-zone fix): each exit is
   * now guarded by its polarity FIRST, so a throttle re-press (tmp) routes to
   * the driver's ACTUAL forward drive state. Previously `tmp` was the leading
   * OR term in both guards, so any re-press always fired this branch1
   * (-> IN_Reverse); on a REGEN_POLARITY=false build that collided with the
   * driver's Forward direction and dropped into the Reverse->Neutral
   * (Invalid_Transition) lockout, so the throttle was ignored until ~600 RPM.
   * MIRROR THIS IN THE SIMULINK FNR_switching CHART (exit-transition guards)
   * or the next model regeneration silently reverts it. */
  if (FOC_Loop_U.Regen.Polarity_switch &&
      (tmp || (*Abs <= FOC_Loop_U.Regen.RPM_Regen_min_Speed))) {
    FOC_Loop_B.T_ref = 0.0F;
    FOC_Loop_DW.counter = 0.0F;

    /* Outport: 'Regen_Mode' (':5722') */
    FOC_Loop_Y.Regen_Mode = false;
    FOC_Loop_DW.durationCounter_2_e = 0U;
    FOC_Loop_DW.durationCounter_1_l = 0U;
    FOC_Loop_DW.is_c2_FOC_Loop = FOC_Loop_IN_Reverse;

    /* Outport: 'Throttle_en' (':5711') */
    FOC_Loop_Y.Throttle_en = true;

    /* Outport: 'Throttle_State' (':5713') */
    FOC_Loop_Y.Throttle_State = Reverse;
  } else if ((!FOC_Loop_U.Regen.Polarity_switch) &&
             (tmp || (*Abs <= FOC_Loop_U.Regen.RPM_Regen_min_Speed))) {
    FOC_Loop_B.T_ref = 0.0F;
    FOC_Loop_DW.counter = 0.0F;

    /* Outport: 'Regen_Mode' (':5722') */
    FOC_Loop_Y.Regen_Mode = false;
    FOC_Loop_DW.durationCounter_2 = 0U;
    FOC_Loop_DW.durationCounter_1 = 0U;
    FOC_Loop_DW.is_c2_FOC_Loop = FOC_Loop_IN_Forward;

    /* Outport: 'Throttle_en' (':5711') */
    FOC_Loop_Y.Throttle_en = true;

    /* Outport: 'Throttle_State' (':5713') */
    FOC_Loop_Y.Throttle_State = Forward;
  } else {
    FOC_Loop_DW.counter += fabsf(FOC_Loop_DW.rpm_prev - *Abs);
    if (FOC_Loop_DW.mid_speed <= FOC_Loop_U.Regen.regen_cap_rpm) {
      ref_speed = FOC_Loop_DW.mid_speed;
    } else {
      ref_speed = FOC_Loop_DW.init_speed;
    }

    ref_speed = fminf(fmaxf((ref_speed - FOC_Loop_U.Regen.RPM_Regen_min_Speed) /
      (fmaxf(ref_speed, FOC_Loop_U.Regen.regen_cap_rpm) -
       FOC_Loop_U.Regen.RPM_Regen_min_Speed), 0.0F), 1.0F) *
      FOC_Loop_DW.max_negTorque;
    if (*Abs >= FOC_Loop_DW.mid_speed) {
      x2 = fminf(fmaxf(FOC_Loop_DW.counter / fabsf(FOC_Loop_DW.init_speed -
        FOC_Loop_DW.mid_speed), 0.0F), 1.0F);
      shape = 1.0F - (1.0F - x2) * (1.0F - x2);
    } else {
      x2 = fminf(fmaxf((*Abs - FOC_Loop_U.Regen.RPM_Regen_min_Speed) /
                       (FOC_Loop_DW.mid_speed -
                        FOC_Loop_U.Regen.RPM_Regen_min_Speed), 0.0F), 1.0F);
      shape = 1.0F - (1.0F - x2) * (1.0F - x2);
    }

    x2 = fabsf(ref_speed);
    FOC_Loop_B.T_ref = (float)-FOC_Loop_DW.drive_sign * x2 * shape;
    if (-FOC_Loop_DW.drive_sign < 0.0) {
      FOC_Loop_B.T_ref = fminf(fmaxf(FOC_Loop_B.T_ref, ref_speed), 0.0F);
    } else if (-FOC_Loop_DW.drive_sign > 0.0) {
      FOC_Loop_B.T_ref = fmaxf(fminf(FOC_Loop_B.T_ref, x2), 0.0F);
    } else {
      FOC_Loop_B.T_ref = 0.0F;
    }

    FOC_Loop_DW.rpm_prev = *Abs;
  }

  /* End of Inport: 'Regen' (':5695') */
}

/* Model step function for TID0 */
void FOC_Loop_step0(void)              /* Sample time: [0.0001s, 0.0s] */
{
  float rtb_Add2;
  float rtb_Add3;
  float rtb_DataTypeConversion1;
  float rtb_Gain_idx_1;
  float rtb_Product2_h;
  float rtb_Sum;
  float rtb_Sum_f;
  float rtb_Switch_fv_idx_0;
  float rtb_VdFF_unsat;
  int32_t i;
  bool FNR_Active;
  bool tmp;
  bool tmp_0;

  /* Update the flag to indicate when data transfers from
   *  Sample time: [0.0001s, 0.0s] to Sample time: [0.001s, 0.0s]  */
  (FOC_Loop_M->Timing.RateInteraction.TID0_1)++;
  if ((FOC_Loop_M->Timing.RateInteraction.TID0_1) > 9) {
    FOC_Loop_M->Timing.RateInteraction.TID0_1 = 0;
  }

  /* Outputs for Atomic SubSystem: 'Two phase CRL wrap' (':710:627') */
  /* Gain: 'one_by_sqrt3' (':710:634') incorporates:
   *  Inport: 'Phase Current' (':5688')
   *  Sum: 'a_plus_2b' (':710:633')
   */
  rtb_DataTypeConversion1 = ((FOC_Loop_U.PhaseCurrent[0] +
    FOC_Loop_U.PhaseCurrent[1]) + FOC_Loop_U.PhaseCurrent[1]) * 0.577350259F;

  /* End of Outputs for SubSystem: 'Two phase CRL wrap' (':710:627') */

  /* Trigonometry: 'Sin_theta' (':713') incorporates:
   *  Inport: 'angle' (':5681')
   *  Trigonometry: 'Sin_theta' (':30')
   */
  rtb_Gain_idx_1 = sinf(FOC_Loop_U.angle);

  /* Trigonometry: 'Cos_theta' (':714') incorporates:
   *  Inport: 'angle' (':5681')
   *  Trigonometry: 'Cos_theta' (':29')
   */
  rtb_Product2_h = cosf(FOC_Loop_U.angle);

#if FOC_ISENSE_DELAY_COMP
  /* HAND-EDIT (current-sense delay compensation) — NOT IN THE SIMULINK MODEL.
   * The phase currents are ~tau old when they reach the Park transform below,
   * so rotate by the angle the rotor had then (Isense_Park_Angle). Only the
   * Park transform reads these two before FOC_VOUT_DELAY_COMP recomputes
   * them for the inverse Park (vehicle_config.h enforces that pairing).
   * tau is CAN-set (0x115). See FOC_ISENSE_DELAY_COMP in vehicle_config.h. */
  {
    float theta_i = Isense_Park_Angle(FOC_Loop_U.angle, d.omega_e, can_d.isense_delay_s);
    rtb_Gain_idx_1 = sinf(theta_i);
    rtb_Product2_h = cosf(theta_i);
  }
#endif

  /* Outputs for Atomic SubSystem: 'Two inputs CRL' (':712:522') */
  /* Outputs for Atomic SubSystem: 'Two phase CRL wrap' (':710:627') */
  /* AlgorithmDescriptorDelegate generated from: 'a16' (':712:520') incorporates:
   *  AlgorithmDescriptorDelegate generated from: 'a16' (':710:632')
   *  Inport: 'Phase Current' (':5688')
   *  Product: 'acos' (':712:239')
   *  Product: 'bsin' (':712:238')
   *  Sum: 'sum_Ds' (':712:242')
   *  Trigonometry: 'Cos_theta' (':714')
   *  Trigonometry: 'Sin_theta' (':713')
   */
  FOC_Loop_Y.Id = FOC_Loop_U.PhaseCurrent[0] * rtb_Product2_h +
    rtb_DataTypeConversion1 * rtb_Gain_idx_1;

  /* AlgorithmDescriptorDelegate generated from: 'a16' (':712:520') incorporates:
   *  AlgorithmDescriptorDelegate generated from: 'a16' (':710:632')
   *  Inport: 'Phase Current' (':5688')
   *  Product: 'asin' (':712:240')
   *  Product: 'bcos' (':712:241')
   *  Sum: 'sum_Qs' (':712:243')
   *  Trigonometry: 'Cos_theta' (':714')
   *  Trigonometry: 'Sin_theta' (':713')
   */
  FOC_Loop_Y.Iq = rtb_DataTypeConversion1 * rtb_Product2_h -
    FOC_Loop_U.PhaseCurrent[0] * rtb_Gain_idx_1;

  /* End of Outputs for SubSystem: 'Two phase CRL wrap' (':710:627') */
  /* End of Outputs for SubSystem: 'Two inputs CRL' (':712:522') */

  /* Sum: 'Add1' (':3455:360') incorporates:
   *  Constant: 'One' (':3455:479')
   *  Inport: 'IIR_Filter_Coefficient' (':5685')
   *  Product: 'Product' (':3455:364')
   *  Product: 'Product1' (':3455:365')
   *  Sum: 'Sum' (':3455:481')
   *  Switch: 'Switch1' (':3455:483')
   *  UnitDelay: 'Unit Delay' (':3455:366')
   */
  FOC_Loop_DW.UnitDelay_DSTATE = (1.0F -
    FOC_Loop_U.IIR_Filter_Coefficient.Id_Filter_coefficient) *
    FOC_Loop_DW.UnitDelay_DSTATE + FOC_Loop_Y.Id *
    FOC_Loop_U.IIR_Filter_Coefficient.Id_Filter_coefficient;

  /* Switch: 'Switch' (':3463') incorporates:
   *  Inport: 'IIR_Filter_Coefficient' (':5685')
   */
  if (FOC_Loop_U.IIR_Filter_Coefficient.Id_Filter_switch) {
    /* AlgorithmDescriptorDelegate generated from: 'a16' (':712:520') incorporates:
     *  UnitDelay: 'Unit Delay' (':3455:366')
     */
    FOC_Loop_Y.Id = FOC_Loop_DW.UnitDelay_DSTATE;
  }

  /* End of Switch: 'Switch' (':3463') */

  /* Sum: 'Add1' (':3456:360') incorporates:
   *  Constant: 'One' (':3456:479')
   *  Inport: 'IIR_Filter_Coefficient' (':5685')
   *  Product: 'Product' (':3456:364')
   *  Product: 'Product1' (':3456:365')
   *  Sum: 'Sum' (':3456:481')
   *  Switch: 'Switch1' (':3456:483')
   *  UnitDelay: 'Unit Delay' (':3456:366')
   */
  FOC_Loop_DW.UnitDelay_DSTATE_f = (1.0F -
    FOC_Loop_U.IIR_Filter_Coefficient.Iq_Filter_coefficient) *
    FOC_Loop_DW.UnitDelay_DSTATE_f + FOC_Loop_Y.Iq *
    FOC_Loop_U.IIR_Filter_Coefficient.Iq_Filter_coefficient;

  /* Switch: 'Switch1' (':3464') incorporates:
   *  Inport: 'IIR_Filter_Coefficient' (':5685')
   */
  if (FOC_Loop_U.IIR_Filter_Coefficient.Iq_Filter_switch) {
    /* AlgorithmDescriptorDelegate generated from: 'a16' (':712:520') incorporates:
     *  UnitDelay: 'Unit Delay' (':3456:366')
     */
    FOC_Loop_Y.Iq = FOC_Loop_DW.UnitDelay_DSTATE_f;
  }

  /* End of Switch: 'Switch1' (':3464') */

  /* Switch: 'Switch' (':5057') incorporates:
   *  Inport: 'MTPA_PID' (':5682')
   *  Math: 'Hypot' (':5070')
   *  RelationalOperator: 'Relational Operator1' (':5055')
   *  RelationalOperator: 'Relational Operator2' (':5056')
   */
  /* ---- HAND-EDIT (2026-06-30, commit 6776a32) — NOT IN THE SIMULINK MODEL --
   * Retro-marked 2026-07-31: this edit was live and unmarked for a month, so
   * it was indistinguishable from generated code and would have been reverted
   * invisibly by the next regeneration.
   *
   * The torque-criterion branch was generated as
   *     tmp = (T_limit <= 0.0F);
   * With LOW_SPEED_I_T_SWITCH = 1 and LOW_SPEED_T_LIMIT = 10.0 that is
   * 10.0 <= 0.0, i.e. CONSTANT FALSE — so tmp never set, PID_Switch below
   * could never assert, and the high-speed Id/Iq/speed gain set was
   * unreachable. The machine always ran the low-speed gains.
   *
   * Comparing the limit against the measured torque restores the intent and
   * mirrors the polarity of the current-criterion branch below
   * (hypot(Id,Iq) >= I_limit): both are "the measurement has exceeded the
   * threshold".
   *
   * MIRROR THIS IN THE SIMULINK MODEL — the 0.0 constant on 'Relational
   * Operator2' (':5056') is the underlying defect and wants fixing there, or
   * the next regeneration silently disables the high-speed gain set again. */
  if (FOC_Loop_U.MTPA_PID.Selection.I_0_T_1_switch) {
    tmp = (FOC_Loop_U.MTPA_PID.Selection.T_limit <= FOC_Loop_Y.Actual_torque_from_meas);
  } else {
    tmp = (rt_hypotf_snf(FOC_Loop_Y.Id, FOC_Loop_Y.Iq) >=
           FOC_Loop_U.MTPA_PID.Selection.I_limit);
  }

  /* RelationalOperator: 'Relational Operator1' (':4899') incorporates:
   *  Gain: 'Gain' (':5008')
   *  Inport: 'Actual Speed_mech_rad//sec' (':5683')
   *  Inport: 'MTPA_PID' (':5682')
   *  Logic: 'Logical Operator' (':5050')
   *  RelationalOperator: 'Relational Operator' (':5054')
   *  Switch: 'Switch' (':5057')
   */
  FOC_Loop_Y.PID_Switch = ((9.54929638F * FOC_Loop_U.ActualSpeed_mech_radsec >=
    FOC_Loop_U.MTPA_PID.Selection.Speed_RPM_limit) && tmp);

  /* If: 'If' (':5011') */
  if (FOC_Loop_Y.PID_Switch) {
    /* Outputs for IfAction SubSystem: 'If Action Subsystem' (':5012') incorporates:
     *  ActionPort: 'Action Port' (':5014')
     */
    /* SignalConversion generated from: 'Bus Selector' (':5015') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge' (':5051')
     */
    FOC_Loop_B.Merge[0] = FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Kp_flux_PID_MTPA;

    /* SignalConversion generated from: 'Bus Selector' (':5015') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge' (':5051')
     */
    FOC_Loop_B.Merge[1] = FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Ki_flux_PID_MTPA;

    /* SignalConversion generated from: 'Bus Selector' (':5015') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge' (':5051')
     */
    FOC_Loop_B.Merge[2] = FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Kd_flux_PID_MTPA;

    /* SignalConversion generated from: 'Bus Selector' (':5015') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge' (':5051')
     */
    FOC_Loop_B.Merge[3] = FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Filter_flux_PID_MTPA;

    /* SignalConversion generated from: 'Bus Selector' (':5015') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge' (':5051')
     */
    FOC_Loop_B.Merge[4] = FOC_Loop_U.MTPA_PID.Up_Limit_flux_PID;

    /* SignalConversion generated from: 'Bus Selector' (':5015') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge' (':5051')
     */
    FOC_Loop_B.Merge[5] = FOC_Loop_U.MTPA_PID.Low_Limit_flux_PID;

    /* End of Outputs for SubSystem: 'If Action Subsystem' (':5012') */
  } else {
    /* Outputs for IfAction SubSystem: 'If Action Subsystem1' (':5018') incorporates:
     *  ActionPort: 'Action Port' (':5020')
     */
    /* SignalConversion generated from: 'Bus Selector' (':5021') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge' (':5051')
     */
    FOC_Loop_B.Merge[0] = FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Kp_flux_PID_MTPA_low;

    /* SignalConversion generated from: 'Bus Selector' (':5021') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge' (':5051')
     */
    FOC_Loop_B.Merge[1] = FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Ki_flux_PID_MTPA_low;

    /* SignalConversion generated from: 'Bus Selector' (':5021') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge' (':5051')
     */
    FOC_Loop_B.Merge[2] = FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Kd_flux_PID_MTPA_low;

    /* SignalConversion generated from: 'Bus Selector' (':5021') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge' (':5051')
     */
    FOC_Loop_B.Merge[3] =
      FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Filter_flux_PID_MTPA_low;

    /* SignalConversion generated from: 'Bus Selector' (':5021') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge' (':5051')
     */
    FOC_Loop_B.Merge[4] = FOC_Loop_U.MTPA_PID.Up_Limit_flux_PID;

    /* SignalConversion generated from: 'Bus Selector' (':5021') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge' (':5051')
     */
    FOC_Loop_B.Merge[5] = FOC_Loop_U.MTPA_PID.Low_Limit_flux_PID;

    /* End of Outputs for SubSystem: 'If Action Subsystem1' (':5018') */
  }

  /* End of If: 'If' (':5011') */

  /* Sum: 'Sum3' (':41') incorporates:
   *  Inport: 'Id_ref_final' (':5696')
   */
  FOC_Loop_Y.Id_error = FOC_Loop_U.Id_ref_final - FOC_Loop_Y.Id;

  /* Product: 'NProd Out' (':42:878') incorporates:
   *  DiscreteIntegrator: 'Filter' (':42:737')
   *  Product: 'DProd Out' (':42:684')
   *  Sum: 'SumD' (':42:738')
   */
  rtb_DataTypeConversion1 = (FOC_Loop_Y.Id_error * FOC_Loop_B.Merge[2] -
    FOC_Loop_DW.Filter_DSTATE) * FOC_Loop_B.Merge[3];

  /* Sum: 'Sum' (':42:3526') incorporates:
   *  DiscreteIntegrator: 'Integrator' (':42:843')
   *  Product: 'PProd Out' (':42:909')
   */
  rtb_Sum = (FOC_Loop_Y.Id_error * FOC_Loop_B.Merge[0] +
             FOC_Loop_DW.Integrator_DSTATE) + rtb_DataTypeConversion1;

  /* Switch: 'Switch2' (':42:3846:8') incorporates:
   *  RelationalOperator: 'LowerRelop1' (':42:3846:6')
   *  RelationalOperator: 'UpperRelop' (':42:3846:9')
   *  Switch: 'Switch' (':42:3846:7')
   */
  if (rtb_Sum > FOC_Loop_B.Merge[4]) {
    /* Switch: 'Switch2' (':42:3846:8') */
    FOC_Loop_Y.Vd_PID = FOC_Loop_B.Merge[4];
  } else if (rtb_Sum < FOC_Loop_B.Merge[5]) {
    /* Switch: 'Switch2' (':42:3846:8') incorporates:
     *  Switch: 'Switch' (':42:3846:7')
     */
    FOC_Loop_Y.Vd_PID = FOC_Loop_B.Merge[5];
  } else {
    /* Switch: 'Switch2' (':42:3846:8') incorporates:
     *  Switch: 'Switch' (':42:3846:7')
     */
    FOC_Loop_Y.Vd_PID = rtb_Sum;
  }

  /* End of Switch: 'Switch2' (':42:3846:8') */

  /* Gain: 'Bus_V_RMS' (':3504') incorporates:
   *  Inport: 'Bus_Voltage' (':5691')
   */
  rtb_Add3 = 0.577350259F * FOC_Loop_U.BusVoltage_V;

  /* Product: 'we' (':4986') incorporates:
   *  Inport: 'Actual Speed_mech_rad//sec' (':5683')
   *  Inport: 'Pole_Pairs' (':5700')
   */
  rtb_Add2 = FOC_Loop_U.ActualSpeed_mech_radsec * FOC_Loop_U.Pole_Pairs;

  /* Gain: 'VdFF_unsat' (':4968') incorporates:
   *  Inport: 'Iq_ref_final' (':5697')
   *  Inport: 'Lq' (':5699')
   *  Product: 'w_Lq_iq' (':4978')
   */
  rtb_VdFF_unsat = -(FOC_Loop_U.Lq * FOC_Loop_U.Iq_ref_final * rtb_Add2);

  /* Switch: 'Switch2' (':4972:8') incorporates:
   *  RelationalOperator: 'LowerRelop1' (':4972:6')
   *  RelationalOperator: 'UpperRelop' (':4972:9')
   *  Switch: 'Switch' (':4972:7')
   *  UnaryMinus: 'Unary Minus' (':4970')
   */
  if (rtb_VdFF_unsat > rtb_Add3) {
    rtb_VdFF_unsat = rtb_Add3;
  } else if (rtb_VdFF_unsat < -rtb_Add3) {
    /* Switch: 'Switch' (':4972:7') incorporates:
     *  UnaryMinus: 'Unary Minus' (':4970')
     */
    rtb_VdFF_unsat = -rtb_Add3;
  }

#if FOC_BENCH_FAST_TLM
  /* HAND-EDIT (bench FF telemetry, ext 0x422) — NOT IN THE SIMULINK MODEL.
   * Vd FF as applied (clamped) and the clamp limit. rtb_Add3 is still the
   * positive +/-0.577*Vbus limit here: the Vq clamp below negates/overwrites
   * it, so the limit must be taken now. See FOC_BENCH_FAST_TLM in
   * vehicle_config.h. */
  fast_tlm_ff.vd_ff = rtb_VdFF_unsat;
  fast_tlm_ff.ff_lim = rtb_Add3;
#endif

  /* Sum: 'Add' (':3500') incorporates:
   *  Switch: 'Switch2' (':4972:8')
   */
  FOC_Loop_Y.Vd = FOC_Loop_Y.Vd_PID + rtb_VdFF_unsat;

#if FOC_VDQ_CIRCLE_LIMIT
  /* HAND-EDIT: keep the Vd feed-forward for the circle-limit anti-windup;
   * rtb_VdFF_unsat is reused for the Vq feed-forward just below. */
  const float vd_ff_aw = rtb_VdFF_unsat;
#endif

  /* Switch: 'Switch2' (':2480:8') incorporates:
   *  Inport: 'MTPA_PID' (':5682')
   *  RelationalOperator: 'LowerRelop1' (':2480:6')
   *  RelationalOperator: 'UpperRelop' (':2480:9')
   *  Switch: 'Switch' (':2480:7')
   */
  if (FOC_Loop_Y.Vd > FOC_Loop_U.MTPA_PID.Up_Limit_flux_PID) {
    /* Sum: 'Add' (':3500') */
    FOC_Loop_Y.Vd = FOC_Loop_U.MTPA_PID.Up_Limit_flux_PID;
  } else if (FOC_Loop_Y.Vd < FOC_Loop_U.MTPA_PID.Low_Limit_flux_PID) {
    /* Sum: 'Add' (':3500') incorporates:
     *  Switch: 'Switch' (':2480:7')
     */
    FOC_Loop_Y.Vd = FOC_Loop_U.MTPA_PID.Low_Limit_flux_PID;
  }

  /* End of Switch: 'Switch2' (':2480:8') */

  /* Product: 'VqFF_unsat' (':4979') incorporates:
   *  Inport: 'Id_ref_final' (':5696')
   *  Inport: 'Lambda' (':5702')
   *  Inport: 'Ld' (':5698')
   *  Product: 'Ld_id' (':4980')
   *  Sum: 'fluxD' (':4975')
   */
  rtb_VdFF_unsat = (FOC_Loop_U.Ld * FOC_Loop_U.Id_ref_final + FOC_Loop_U.Lambda)
    * rtb_Add2;

#if FOC_BENCH_FAST_TLM
  /* HAND-EDIT (bench FF telemetry, ext 0x422) — NOT IN THE SIMULINK MODEL.
   * rtb_VdFF_unsat now holds the UNCLAMPED Vq feed-forward (it was reused
   * from the Vd FF above). See FOC_BENCH_FAST_TLM in vehicle_config.h. */
  fast_tlm_ff.vq_ff_unsat = rtb_VdFF_unsat;
#endif

  /* Switch: 'Switch2' (':4974:8') incorporates:
   *  RelationalOperator: 'LowerRelop1' (':4974:6')
   */
  if (!(rtb_VdFF_unsat > rtb_Add3)) {
    /* Switch: 'Switch' (':4974:7') incorporates:
     *  RelationalOperator: 'UpperRelop' (':4974:9')
     *  UnaryMinus: 'Unary Minus' (':4970')
     */
    if (rtb_VdFF_unsat < -rtb_Add3) {
      rtb_Add3 = -rtb_Add3;
    } else {
      rtb_Add3 = rtb_VdFF_unsat;
    }

    /* End of Switch: 'Switch' (':4974:7') */
  }

  /* End of Switch: 'Switch2' (':4974:8') */

#if FOC_BENCH_FAST_TLM
  /* HAND-EDIT (bench FF telemetry, ext 0x422) — NOT IN THE SIMULINK MODEL.
   * rtb_Add3 now holds the CLAMPED Vq feed-forward as applied (its limit was
   * overwritten above, captured earlier as ff_lim). See FOC_BENCH_FAST_TLM in
   * vehicle_config.h. */
  fast_tlm_ff.vq_ff = rtb_Add3;
#endif

  /* If: 'If2' (':5049') */
  if (FOC_Loop_Y.PID_Switch) {
    /* Outputs for IfAction SubSystem: 'If Action Subsystem4' (':5036') incorporates:
     *  ActionPort: 'Action Port' (':5038')
     */
    /* SignalConversion generated from: 'Bus Selector' (':5039') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge2' (':5053')
     */
    FOC_Loop_B.Merge2[0] =
      FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Kp_torque_PID_MTPA;

    /* SignalConversion generated from: 'Bus Selector' (':5039') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge2' (':5053')
     */
    FOC_Loop_B.Merge2[1] =
      FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Ki_torque_PID_MTPA;

    /* SignalConversion generated from: 'Bus Selector' (':5039') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge2' (':5053')
     */
    FOC_Loop_B.Merge2[2] =
      FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Kd_torque_PID_MTPA;

    /* SignalConversion generated from: 'Bus Selector' (':5039') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge2' (':5053')
     */
    FOC_Loop_B.Merge2[3] =
      FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Filter_torque_PID_MTPA;

    /* SignalConversion generated from: 'Bus Selector' (':5039') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge2' (':5053')
     */
    FOC_Loop_B.Merge2[4] = FOC_Loop_U.MTPA_PID.Up_Limit_torque_PID;

    /* SignalConversion generated from: 'Bus Selector' (':5039') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge2' (':5053')
     */
    FOC_Loop_B.Merge2[5] = FOC_Loop_U.MTPA_PID.Low_Limit_torque_PID;

    /* End of Outputs for SubSystem: 'If Action Subsystem4' (':5036') */
  } else {
    /* Outputs for IfAction SubSystem: 'If Action Subsystem5' (':5042') incorporates:
     *  ActionPort: 'Action Port' (':5044')
     */
    /* SignalConversion generated from: 'Bus Selector' (':5045') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge2' (':5053')
     */
    FOC_Loop_B.Merge2[0] =
      FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Kp_torque_PID_MTPA_low;

    /* SignalConversion generated from: 'Bus Selector' (':5045') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge2' (':5053')
     */
    FOC_Loop_B.Merge2[1] =
      FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Ki_torque_PID_MTPA_low;

    /* SignalConversion generated from: 'Bus Selector' (':5045') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge2' (':5053')
     */
    FOC_Loop_B.Merge2[2] =
      FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Kd_torque_PID_MTPA_low;

    /* SignalConversion generated from: 'Bus Selector' (':5045') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge2' (':5053')
     */
    FOC_Loop_B.Merge2[3] =
      FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Filter_torque_PID_MTPA_low;

    /* SignalConversion generated from: 'Bus Selector' (':5045') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge2' (':5053')
     */
    FOC_Loop_B.Merge2[4] = FOC_Loop_U.MTPA_PID.Up_Limit_torque_PID;

    /* SignalConversion generated from: 'Bus Selector' (':5045') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge2' (':5053')
     */
    FOC_Loop_B.Merge2[5] = FOC_Loop_U.MTPA_PID.Low_Limit_torque_PID;

    /* End of Outputs for SubSystem: 'If Action Subsystem5' (':5042') */
  }

  /* End of If: 'If2' (':5049') */

  /* Sum: 'Sum2' (':4') incorporates:
   *  Inport: 'Iq_ref_final' (':5697')
   */
  FOC_Loop_Y.Iq_error = FOC_Loop_U.Iq_ref_final - FOC_Loop_Y.Iq;

  /* Product: 'NProd Out' (':5:878') incorporates:
   *  DiscreteIntegrator: 'Filter' (':5:737')
   *  Product: 'DProd Out' (':5:684')
   *  Sum: 'SumD' (':5:738')
   */
  rtb_VdFF_unsat = (FOC_Loop_Y.Iq_error * FOC_Loop_B.Merge2[2] -
                    FOC_Loop_DW.Filter_DSTATE_d) * FOC_Loop_B.Merge2[3];

  /* Sum: 'Sum' (':5:3526') incorporates:
   *  DiscreteIntegrator: 'Integrator' (':5:843')
   *  Product: 'PProd Out' (':5:909')
   */
  rtb_Sum_f = (FOC_Loop_Y.Iq_error * FOC_Loop_B.Merge2[0] +
               FOC_Loop_DW.Integrator_DSTATE_h) + rtb_VdFF_unsat;

  /* Switch: 'Switch2' (':5:3846:8') incorporates:
   *  RelationalOperator: 'LowerRelop1' (':5:3846:6')
   *  RelationalOperator: 'UpperRelop' (':5:3846:9')
   *  Switch: 'Switch' (':5:3846:7')
   */
  if (rtb_Sum_f > FOC_Loop_B.Merge2[4]) {
    /* Switch: 'Switch2' (':5:3846:8') */
    FOC_Loop_Y.Vq_PID = FOC_Loop_B.Merge2[4];
  } else if (rtb_Sum_f < FOC_Loop_B.Merge2[5]) {
    /* Switch: 'Switch2' (':5:3846:8') incorporates:
     *  Switch: 'Switch' (':5:3846:7')
     */
    FOC_Loop_Y.Vq_PID = FOC_Loop_B.Merge2[5];
  } else {
    /* Switch: 'Switch2' (':5:3846:8') incorporates:
     *  Switch: 'Switch' (':5:3846:7')
     */
    FOC_Loop_Y.Vq_PID = rtb_Sum_f;
  }

  /* End of Switch: 'Switch2' (':5:3846:8') */

  /* Sum: 'Add1' (':3501') */
  FOC_Loop_Y.Vq = rtb_Add3 + FOC_Loop_Y.Vq_PID;

  /* Switch: 'Switch2' (':2481:8') incorporates:
   *  Inport: 'MTPA_PID' (':5682')
   *  RelationalOperator: 'LowerRelop1' (':2481:6')
   *  RelationalOperator: 'UpperRelop' (':2481:9')
   *  Switch: 'Switch' (':2481:7')
   */
  if (FOC_Loop_Y.Vq > FOC_Loop_U.MTPA_PID.Up_Limit_torque_PID) {
    /* Sum: 'Add1' (':3501') */
    FOC_Loop_Y.Vq = FOC_Loop_U.MTPA_PID.Up_Limit_torque_PID;
  } else if (FOC_Loop_Y.Vq < FOC_Loop_U.MTPA_PID.Low_Limit_torque_PID) {
    /* Sum: 'Add1' (':3501') incorporates:
     *  Switch: 'Switch' (':2481:7')
     */
    FOC_Loop_Y.Vq = FOC_Loop_U.MTPA_PID.Low_Limit_torque_PID;
  }

  /* End of Switch: 'Switch2' (':2481:8') */

#if FOC_VDQ_CIRCLE_LIMIT
  /* HAND-EDIT (circular |Vdq| limit) — NOT IN THE SIMULINK MODEL.
   * The per-axis clamps above let the pair reach sqrt2 x the SVM circle.
   * Shorten it to the circle, d-axis priority with |Vd| held to
   * FOC_VDQ_LIMIT_VD_MAX_FRAC of the radius so Vq is never zeroed, and give the integrator
   * updates below the PI output that was actually applied (final voltage
   * minus feed-forward), so their back-calculation sees this limit and the
   * per-axis clamps, not just the PI's own clamp. rtb_Add3 still holds the
   * clamped Vq feed-forward here (reassigned at the inverse Park).
   * d.vdq_demand is the pre-limit magnitude for 0x715 byte1.
   * See FOC_VDQ_CIRCLE_LIMIT in vehicle_config.h. */
  d.vdq_demand = sqrtf(FOC_Loop_Y.Vd * FOC_Loop_Y.Vd +
                       FOC_Loop_Y.Vq * FOC_Loop_Y.Vq);
  d.vdq_lim_cycles++;
  if (Vdq_Circle_Limit(&FOC_Loop_Y.Vd, &FOC_Loop_Y.Vq, d.Vmax_SVM,
                       d.Vmax_SVM * FOC_VDQ_LIMIT_VD_MAX_FRAC)) {
    d.vdq_lim_clips++;
  }
  const float vd_pi_aw = FOC_Loop_Y.Vd - vd_ff_aw;
  const float vq_pi_aw = FOC_Loop_Y.Vq - rtb_Add3;
#endif

#if FOC_VOUT_DELAY_COMP
  /* HAND-EDIT (output delay compensation) — NOT IN THE SIMULINK MODEL.
   * The inverse Park below reused the sin/cos of the sampling angle, so the
   * voltage was rotated for where the rotor WAS, not where it will be when
   * the PWM lands ~FOC_VOUT_DELAY_S later. Re-evaluate both at the advanced
   * angle; the current Park transform above keeps the sampling angle. Safe
   * to overwrite: nothing between the Park transform and here reads these
   * two, and everything after the inverse Park reassigns them before use.
   * See FOC_VOUT_DELAY_COMP in vehicle_config.h. */
  {
    float theta_out = FOC_Loop_U.angle + d.omega_e * FOC_VOUT_DELAY_S;
    rtb_Gain_idx_1 = sinf(theta_out);
    rtb_Product2_h = cosf(theta_out);
  }
#endif

  /* Outputs for Atomic SubSystem: 'Two inputs CRL' (':482:778') */
  /* Switch: 'Switch' (':482:755') incorporates:
   *  Product: 'dcos' (':482:431')
   *  Product: 'qsin' (':482:433')
   *  Sum: 'sum_alpha' (':482:432')
   */
  rtb_Switch_fv_idx_0 = FOC_Loop_Y.Vd * rtb_Product2_h - FOC_Loop_Y.Vq *
    rtb_Gain_idx_1;

  /* Gain: 'one_by_two' (':527:493') incorporates:
   *  AlgorithmDescriptorDelegate generated from: 'a16' (':482:761')
   */
  rtb_Add2 = 0.5F * rtb_Switch_fv_idx_0;

  /* Gain: 'sqrt3_by_two' (':527:492') incorporates:
   *  Product: 'dsin' (':482:435')
   *  Product: 'qcos' (':482:434')
   *  Sum: 'sum_beta' (':482:436')
   */
  rtb_Add3 = (FOC_Loop_Y.Vq * rtb_Product2_h + FOC_Loop_Y.Vd * rtb_Gain_idx_1) *
    0.866025388F;

  /* End of Outputs for SubSystem: 'Two inputs CRL' (':482:778') */

  /* Switch: 'Switch' (':5314:107') incorporates:
   *  AlgorithmDescriptorDelegate generated from: 'a16' (':482:761')
   *  Inport: 'Dead_Time_Compensator' (':5693')
   *  MultiPortSwitch: 'Multiport Switch' (':5314:145')
   *  Product: 'Product' (':5314:67')
   *  Sum: 'Sum1' (':5314:71')
   *  Sum: 'add_b' (':527:490')
   *  Sum: 'add_c' (':527:499')
   */
  if (FOC_Loop_U.Dead_Time_Compensator.Enable) {
    /* Product: 'Product2' (':5314:69') incorporates:
     *  Constant: 'Constant' (':5314:65')
     *  Inport: 'Bus_Voltage' (':5691')
     *  Product: 'Product1' (':5314:68')
     */
    rtb_Product2_h = 1.0E-6F * FOC_Loop_U.Dead_Time_Compensator.Switching_Freq *
      FOC_Loop_U.BusVoltage_V;

    /* MultiPortSwitch: 'Multiport Switch' (':5314:145') incorporates:
     *  Constant: 'Zero' (':5314:142')
     *  Constant: 'ilessthan0' (':5314:147')
     *  Constant: 'imorethan0' (':5314:146')
     *  Inport: 'Phase Current' (':5688')
     *  RelationalOperator: 'Relational Operator' (':5314:141')
     */
    if (!(FOC_Loop_U.PhaseCurrent[0] >= 0.0F)) {
      i = -1;
    } else {
      i = 1;
    }

    /* Outputs for Atomic SubSystem: 'Two inputs CRL' (':482:778') */
    rtb_Switch_fv_idx_0 += rtb_Product2_h * (float)i;

    /* End of Outputs for SubSystem: 'Two inputs CRL' (':482:778') */

    /* MultiPortSwitch: 'Multiport Switch' (':5314:145') incorporates:
     *  AlgorithmDescriptorDelegate generated from: 'a16' (':482:761')
     *  Constant: 'Zero' (':5314:142')
     *  Constant: 'ilessthan0' (':5314:147')
     *  Constant: 'imorethan0' (':5314:146')
     *  Inport: 'Phase Current' (':5688')
     *  Product: 'Product' (':5314:67')
     *  RelationalOperator: 'Relational Operator' (':5314:141')
     *  Sum: 'Sum1' (':5314:71')
     */
    if (!(FOC_Loop_U.PhaseCurrent[1] >= 0.0F)) {
      i = -1;
    } else {
      i = 1;
    }

    rtb_Gain_idx_1 = (rtb_Add3 - rtb_Add2) + rtb_Product2_h * (float)i;

    /* MultiPortSwitch: 'Multiport Switch' (':5314:145') incorporates:
     *  Constant: 'Zero' (':5314:142')
     *  Constant: 'ilessthan0' (':5314:147')
     *  Constant: 'imorethan0' (':5314:146')
     *  Inport: 'Phase Current' (':5688')
     *  Product: 'Product' (':5314:67')
     *  RelationalOperator: 'Relational Operator' (':5314:141')
     *  Sum: 'Sum1' (':5314:71')
     *  Sum: 'add_b' (':527:490')
     */
    if (!(FOC_Loop_U.PhaseCurrent[2] >= 0.0F)) {
      i = -1;
    } else {
      i = 1;
    }

    rtb_Add3 = ((0.0F - rtb_Add2) - rtb_Add3) + rtb_Product2_h * (float)i;
  } else {
    rtb_Gain_idx_1 = rtb_Add3 - rtb_Add2;
    rtb_Add3 = (0.0F - rtb_Add2) - rtb_Add3;
  }

  /* End of Switch: 'Switch' (':5314:107') */

  /* Gain: 'one_by_two' (':594:1405') incorporates:
   *  MinMax: 'Max' (':594:1403')
   *  MinMax: 'Min' (':594:1404')
   *  Sum: 'Add' (':594:1400')
   */
  rtb_Add2 = (fmaxf(fmaxf(rtb_Switch_fv_idx_0, rtb_Gain_idx_1), rtb_Add3) +
              fminf(fminf(rtb_Switch_fv_idx_0, rtb_Gain_idx_1), rtb_Add3)) *
    -0.5F;

  /* Outport: 'Va' (':5704') incorporates:
   *  Gain: 'Gain' (':594:1551')
   *  Sum: 'Add3' (':594:1409')
   */
  FOC_Loop_Y.Va = (rtb_Switch_fv_idx_0 + rtb_Add2) * 1.15470052F;

  /* Outport: 'Vb' (':5705') incorporates:
   *  Gain: 'Gain' (':594:1551')
   *  Sum: 'Add1' (':594:1407')
   */
  FOC_Loop_Y.Vb = (rtb_Gain_idx_1 + rtb_Add2) * 1.15470052F;

  /* Outport: 'Vc' (':5706') incorporates:
   *  Gain: 'Gain' (':594:1551')
   *  Sum: 'Add2' (':594:1408')
   */
  FOC_Loop_Y.Vc = (rtb_Add2 + rtb_Add3) * 1.15470052F;

  /* If: 'If1' (':5048') */
  if (FOC_Loop_Y.PID_Switch) {
    /* Outputs for IfAction SubSystem: 'If Action Subsystem2' (':5024') incorporates:
     *  ActionPort: 'Action Port' (':5026')
     */
    /* SignalConversion generated from: 'Bus Selector' (':5027') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge1' (':5052')
     */
    FOC_Loop_B.Merge1[0] = FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Kp_speed_PID_MTPA;

    /* SignalConversion generated from: 'Bus Selector' (':5027') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge1' (':5052')
     */
    FOC_Loop_B.Merge1[1] = FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Ki_speed_PID_MTPA;

    /* SignalConversion generated from: 'Bus Selector' (':5027') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge1' (':5052')
     */
    FOC_Loop_B.Merge1[2] = FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Kd_speed_PID_MTPA;

    /* SignalConversion generated from: 'Bus Selector' (':5027') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge1' (':5052')
     */
    FOC_Loop_B.Merge1[3] =
      FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Filter_speed_PID_MTPA;

    /* SignalConversion generated from: 'Bus Selector' (':5027') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge1' (':5052')
     */
    FOC_Loop_B.Merge1[4] = FOC_Loop_U.MTPA_PID.Up_Limit_speed_PID_MTPA;

    /* SignalConversion generated from: 'Bus Selector' (':5027') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge1' (':5052')
     */
    FOC_Loop_B.Merge1[5] = FOC_Loop_U.MTPA_PID.Low_Limit_speed_PID_MTPA;

    /* End of Outputs for SubSystem: 'If Action Subsystem2' (':5024') */
  } else {
    /* Outputs for IfAction SubSystem: 'If Action Subsystem3' (':5030') incorporates:
     *  ActionPort: 'Action Port' (':5032')
     */
    /* SignalConversion generated from: 'Bus Selector' (':5033') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge1' (':5052')
     */
    FOC_Loop_B.Merge1[0] =
      FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Kp_speed_PID_MTPA_low;

    /* SignalConversion generated from: 'Bus Selector' (':5033') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge1' (':5052')
     */
    FOC_Loop_B.Merge1[1] =
      FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Ki_speed_PID_MTPA_low;

    /* SignalConversion generated from: 'Bus Selector' (':5033') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge1' (':5052')
     */
    FOC_Loop_B.Merge1[2] =
      FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Kd_speed_PID_MTPA_low;

    /* SignalConversion generated from: 'Bus Selector' (':5033') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge1' (':5052')
     */
    FOC_Loop_B.Merge1[3] =
      FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Filter_speed_PID_MTPA_low;

    /* SignalConversion generated from: 'Bus Selector' (':5033') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge1' (':5052')
     */
    FOC_Loop_B.Merge1[4] = FOC_Loop_U.MTPA_PID.Up_Limit_speed_PID_MTPA;

    /* SignalConversion generated from: 'Bus Selector' (':5033') incorporates:
     *  Inport: 'MTPA_PID' (':5682')
     *  Merge: 'Merge1' (':5052')
     */
    FOC_Loop_B.Merge1[5] = FOC_Loop_U.MTPA_PID.Low_Limit_speed_PID_MTPA;

    /* End of Outputs for SubSystem: 'If Action Subsystem3' (':5030') */
  }

  /* End of If: 'If1' (':5048') */

  /* Outputs for Atomic SubSystem: 'Torque_calculation' (':5304') */
  /* Outport: 'Actual_torque_from_meas' (':5725') incorporates:
   *  Constant: 'Constant' (':5304:46')
   *  Inport: 'Lambda' (':5702')
   *  Inport: 'Ld' (':5698')
   *  Inport: 'Lq' (':5699')
   *  Inport: 'Pole_Pairs' (':5700')
   *  Product: 'Product' (':5304:47')
   *  Product: 'Product1' (':5304:48')
   *  Product: 'Product2' (':5304:49')
   *  Sum: 'Add' (':5304:45')
   *  Sum: 'Subtract' (':5304:50')
   */
  FOC_Loop_Y.Actual_torque_from_meas = (float)((FOC_Loop_Y.Id * FOC_Loop_Y.Iq *
    (FOC_Loop_U.Ld - FOC_Loop_U.Lq) + FOC_Loop_Y.Iq * FOC_Loop_U.Lambda) *
    FOC_Loop_U.Pole_Pairs * 1.5);

  /* End of Outputs for SubSystem: 'Torque_calculation' (':5304') */

  /* RateTransition generated from: 'Merge1' (':5052') incorporates:
   *  RateTransition: 'Rate Transition' (':4080')
   *  RateTransition generated from: 'Data Type Conversion1' (':4040')
   */
  tmp = (FOC_Loop_M->Timing.RateInteraction.TID0_1 == 1);
  if (tmp) {
    for (i = 0; i < 6; i++) {
      FOC_Loop_DW.TmpRTBAtMerge1Outport1_Buffer[i] = FOC_Loop_B.Merge1[i];
    }

    /* Outport: 'T_gen' (':5719') */
    FOC_Loop_Y.T_gen = FOC_Loop_DW.RateTransition_Buffer0;
  }

  /* End of RateTransition generated from: 'Merge1' (':5052') */

  /* Delay: 'Delay' (':5248') incorporates:
   *  Constant: 'Constant' (':5247')
   */
  if (FOC_Loop_DW.icLoad) {
    FOC_Loop_DW.Delay_DSTATE = 0.0F;
  }

  /* Abs: 'Abs' (':4887') incorporates:
   *  Gain: 'Rad//sec to rpm' (':4897')
   *  Inport: 'Actual Speed_mech_rad//sec' (':5683')
   */
  rtb_Add2 = fabsf(9.54929638F * FOC_Loop_U.ActualSpeed_mech_radsec);

  /* RelationalOperator: 'FNR_Active' (':4892') incorporates:
   *  Inport: 'Throttle_input' (':5694')
   *  Sum: 'Add' (':4888')
   */
  FNR_Active = (FOC_Loop_U.Throttle_input.Throttle_Inst_Voltage <=
                FOC_Loop_U.Throttle_input.Min_Throttle_Voltage +
                FOC_Loop_U.Throttle_input.Throttle_V_Hyst_band_FNR_shift);

  /* Outport: 'Throttle_error' (':5717') incorporates:
   *  Inport: 'Throttle_input' (':5694')
   *  Logic: 'Logical Operator' (':4896')
   *  RelationalOperator: 'Relational Operator' (':4898')
   *  RelationalOperator: 'Relational Operator1' (':4899')
   */
  if (cS == ANGLE_CALIB)
    FOC_Loop_Y.Throttle_error = false;

  if (cS == FOC_START) {
    FOC_Loop_Y.Throttle_error = false;
  }

  /* Chart: 'FNR_switching' (':4893') incorporates:
   *  Inport: 'Drive_State' (':5687')
   *  Inport: 'RPM_min_limit' (':5686')
   *  Inport: 'Regen' (':5695')
   *  Outport: 'T_gen' (':5719')
   *  Outport: 'Throttle_error' (':5717')
   */
  if (FOC_Loop_DW.is_active_c2_FOC_Loop == 0) {
    FOC_Loop_DW.is_active_c2_FOC_Loop = 1U;
    FOC_Loop_DW.is_c2_FOC_Loop = FOC_Loop_IN_Initial;

    /* Outport: 'Throttle_en' (':5711') */
    FOC_Loop_Y.Throttle_en = false;

    /* Outport: 'Throttle_State' (':5713') */
    FOC_Loop_Y.Throttle_State = Neutral;
    FOC_Loop_B.T_ref = 0.0F;
  } else {
    switch (FOC_Loop_DW.is_c2_FOC_Loop) {
     case FOC_Loop_IN_Forward:
      /* Outport: 'Throttle_State' (':5713') */
      FOC_Loop_Y.Throttle_State = Forward;
      if (!FOC_Loop_Y.Throttle_error) {
        FOC_Loop_DW.durationCounter_2 = 0U;
      }

      if (FOC_Loop_DW.durationCounter_2 > 1000U) {
        FOC_Loop_DW.is_c2_FOC_Loop = FOC_Loop_IN_Throttle_error;

        /* Outport: 'Throttle_en' (':5711') */
        FOC_Loop_Y.Throttle_en = false;

        /* Outport: 'Throttle_State' (':5713') */
        FOC_Loop_Y.Throttle_State = Throttle_Error;
      } else if (FOC_Loop_U.Drive_State == 2.0F) {
        FOC_Loop_DW.durationCounter_1_m = 0U;
        FOC_Loop_DW.is_c2_FOC_Loop = FOC_Loop_IN_Neutral;

        /* Outport: 'Throttle_en' (':5711') */
        FOC_Loop_Y.Throttle_en = false;

        /* Outport: 'Throttle_State' (':5713') */
        FOC_Loop_Y.Throttle_State = Neutral;
        FOC_Loop_DW.is_Neutral = FOC_Loop_IN_Invalid_Transition;
      } else if (FOC_Loop_U.Drive_State == 3.0F) {
        FOC_Loop_DW.durationCounter_1_m = 0U;
        FOC_Loop_DW.is_c2_FOC_Loop = FOC_Loop_IN_Neutral;

        /* Outport: 'Throttle_en' (':5711') */
        FOC_Loop_Y.Throttle_en = false;

        /* Outport: 'Throttle_State' (':5713') */
        FOC_Loop_Y.Throttle_State = Neutral;
        FOC_Loop_DW.is_Neutral = FOC_Loop_IN_Normal;
      } else {
        if ((!FNR_Active) || (!(rtb_Add2 > FOC_Loop_U.Regen.RPM_Regen_min_Speed))
            || FOC_Loop_U.Regen.Polarity_switch ||
            (!FOC_Loop_U.Regen.Regen_Active)) {
          FOC_Loop_DW.durationCounter_1 = 0U;
        }

        if (FOC_Loop_DW.durationCounter_1 > 1000U) {
          FOC_Loop_DW.drive_sign = 1.0;
          FOC_Loop_DW.max_negTorque = -FOC_Loop_U.Regen.max_Regen_Torque;
          FOC_Loop_DW.is_c2_FOC_Loop = FOC_Loop_IN_Regen_mode1;

          /* Outport: 'Regen_Mode' (':5722') */
          FOC_Loop_Y.Regen_Mode = true;
          FOC_Loop_DW.init_speed = rtb_Add2;
          FOC_Loop_DW.mid_speed = (FOC_Loop_DW.init_speed +
            FOC_Loop_U.Regen.RPM_Regen_min_Speed) / 2.0F;
          FOC_Loop_DW.rpm_prev = rtb_Add2;
          FOC_Loop_DW.counter = 0.0F;
          FOC_Loop_B.T_ref = 0.0F;
        } else {
          FOC_Loop_B.T_ref = FOC_Loop_Y.T_gen;
        }
      }
      break;

     case FOC_Loop_IN_Initial:
      /* Outport: 'Throttle_State' (':5713') */
      FOC_Loop_Y.Throttle_State = Neutral;
      if ((FOC_Loop_Y.T_gen < 1.0F) && FNR_Active) {
        FOC_Loop_DW.durationCounter_1_m = 0U;
        FOC_Loop_DW.is_c2_FOC_Loop = FOC_Loop_IN_Neutral;

        /* Outport: 'Throttle_en' (':5711') */
        FOC_Loop_Y.Throttle_en = false;

        /* Outport: 'Throttle_State' (':5713') */
        FOC_Loop_Y.Throttle_State = Neutral;
        FOC_Loop_DW.is_Neutral = FOC_Loop_IN_Normal;
      }
      break;

     case FOC_Loop_IN_Neutral:
      /* Outport: 'Throttle_State' (':5713') */
      FOC_Loop_Y.Throttle_State = Neutral;
      tmp_0 = !FOC_Loop_Y.Throttle_error;
      if (((!(FOC_Loop_U.Drive_State == 1.0F)) || tmp_0) &&
          ((!(FOC_Loop_U.Drive_State == 2.0F)) || tmp_0)) {
        FOC_Loop_DW.durationCounter_1_m = 0U;
      }

      if (FOC_Loop_DW.durationCounter_1_m > 1000U) {
        FOC_Loop_DW.is_Neutral = FOC_Loop_IN_NO_ACTIVE_CHILD;
        FOC_Loop_DW.is_c2_FOC_Loop = FOC_Loop_IN_Throttle_error;

        /* Outport: 'Throttle_en' (':5711') */
        FOC_Loop_Y.Throttle_en = false;

        /* Outport: 'Throttle_State' (':5713') */
        FOC_Loop_Y.Throttle_State = Throttle_Error;
      } else {
        FOC_Loop_B.T_ref = FOC_Loop_Y.T_gen;
        if (FOC_Loop_DW.is_Neutral == FOC_Loop_IN_Invalid_Transition) {
          if (FOC_Loop_U.Drive_State == 3.0F) {
            FOC_Loop_DW.is_Neutral = FOC_Loop_IN_Normal;
          }

          /* case IN_Normal: */
        } else if ((FOC_Loop_U.Drive_State == 2.0F) && (rtb_Add2 <=
                    FOC_Loop_U.RPM_min_limit) && tmp_0 && (FOC_Loop_Y.T_gen <
                    1.0F) && FNR_Active) {
          FOC_Loop_DW.is_Neutral = FOC_Loop_IN_NO_ACTIVE_CHILD;
          FOC_Loop_DW.durationCounter_2_e = 0U;
          FOC_Loop_DW.durationCounter_1_l = 0U;
          FOC_Loop_DW.is_c2_FOC_Loop = FOC_Loop_IN_Reverse;

          /* Outport: 'Throttle_en' (':5711') */
          FOC_Loop_Y.Throttle_en = true;

          /* Outport: 'Throttle_State' (':5713') */
          FOC_Loop_Y.Throttle_State = Reverse;
        } else if ((FOC_Loop_U.Drive_State == 1.0F) && (rtb_Add2 <=
                    FOC_Loop_U.RPM_min_limit) && tmp_0 && (FOC_Loop_Y.T_gen <
                    1.0F) && FNR_Active) {
          FOC_Loop_DW.is_Neutral = FOC_Loop_IN_NO_ACTIVE_CHILD;
          FOC_Loop_DW.durationCounter_2 = 0U;
          FOC_Loop_DW.durationCounter_1 = 0U;
          FOC_Loop_DW.is_c2_FOC_Loop = FOC_Loop_IN_Forward;

          /* Outport: 'Throttle_en' (':5711') */
          FOC_Loop_Y.Throttle_en = true;

          /* Outport: 'Throttle_State' (':5713') */
          FOC_Loop_Y.Throttle_State = Forward;
        }
      }
      break;

     case FOC_Loop_IN_Regen_mode1:
      FOC_Loop_Regen_mode1(&rtb_Add2, &FNR_Active);
      break;

     case FOC_Loop_IN_Reverse:
      /* Outport: 'Throttle_State' (':5713') */
      FOC_Loop_Y.Throttle_State = Reverse;
      if (!FOC_Loop_Y.Throttle_error) {
        FOC_Loop_DW.durationCounter_1_l = 0U;
      }

      if (FOC_Loop_DW.durationCounter_1_l > 1000U) {
        FOC_Loop_DW.is_c2_FOC_Loop = FOC_Loop_IN_Throttle_error;

        /* Outport: 'Throttle_en' (':5711') */
        FOC_Loop_Y.Throttle_en = false;

        /* Outport: 'Throttle_State' (':5713') */
        FOC_Loop_Y.Throttle_State = Throttle_Error;
      } else if (FOC_Loop_U.Drive_State == 3.0F) {
        FOC_Loop_DW.durationCounter_1_m = 0U;
        FOC_Loop_DW.is_c2_FOC_Loop = FOC_Loop_IN_Neutral;

        /* Outport: 'Throttle_en' (':5711') */
        FOC_Loop_Y.Throttle_en = false;

        /* Outport: 'Throttle_State' (':5713') */
        FOC_Loop_Y.Throttle_State = Neutral;
        FOC_Loop_DW.is_Neutral = FOC_Loop_IN_Normal;
      } else if (FOC_Loop_U.Drive_State == 1.0F) {
        FOC_Loop_DW.durationCounter_1_m = 0U;
        FOC_Loop_DW.is_c2_FOC_Loop = FOC_Loop_IN_Neutral;

        /* Outport: 'Throttle_en' (':5711') */
        FOC_Loop_Y.Throttle_en = false;

        /* Outport: 'Throttle_State' (':5713') */
        FOC_Loop_Y.Throttle_State = Neutral;
        FOC_Loop_DW.is_Neutral = FOC_Loop_IN_Invalid_Transition;
      } else {
        if ((!FNR_Active) || (!(rtb_Add2 > FOC_Loop_U.Regen.RPM_Regen_min_Speed))
            || (!FOC_Loop_U.Regen.Polarity_switch) ||
            (!FOC_Loop_U.Regen.Regen_Active)) {
          FOC_Loop_DW.durationCounter_2_e = 0U;
        }

        if (FOC_Loop_DW.durationCounter_2_e > 1000U) {
          FOC_Loop_DW.drive_sign = -1.0;
          FOC_Loop_DW.max_negTorque = FOC_Loop_U.Regen.max_Regen_Torque;
          FOC_Loop_DW.is_c2_FOC_Loop = FOC_Loop_IN_Regen_mode1;

          /* Outport: 'Regen_Mode' (':5722') */
          FOC_Loop_Y.Regen_Mode = true;
          FOC_Loop_DW.init_speed = rtb_Add2;
          FOC_Loop_DW.mid_speed = (FOC_Loop_DW.init_speed +
            FOC_Loop_U.Regen.RPM_Regen_min_Speed) / 2.0F;
          FOC_Loop_DW.rpm_prev = rtb_Add2;
          FOC_Loop_DW.counter = 0.0F;
          FOC_Loop_B.T_ref = 0.0F;
        } else {
          FOC_Loop_B.T_ref = -FOC_Loop_Y.T_gen;
        }
      }
      break;

     default:
      /* Outport: 'Throttle_State' (':5713') */
      /* case IN_Throttle_error: */
      FOC_Loop_Y.Throttle_State = Throttle_Error;
      if ((FOC_Loop_U.Drive_State == 3.0F) && (rtb_Add2 <=
           FOC_Loop_U.RPM_min_limit) && (!FOC_Loop_Y.Throttle_error) &&
          FNR_Active) {
        FOC_Loop_DW.durationCounter_1_m = 0U;
        FOC_Loop_DW.is_c2_FOC_Loop = FOC_Loop_IN_Neutral;

        /* Outport: 'Throttle_en' (':5711') */
        FOC_Loop_Y.Throttle_en = false;

        /* Outport: 'Throttle_State' (':5713') */
        FOC_Loop_Y.Throttle_State = Neutral;
        FOC_Loop_DW.is_Neutral = FOC_Loop_IN_Normal;
      } else {
        FOC_Loop_B.T_ref = FOC_Loop_Y.T_gen;
      }
      break;
    }
  }

  tmp_0 = (FNR_Active && (rtb_Add2 > FOC_Loop_U.Regen.RPM_Regen_min_Speed));
  if (tmp_0 && (!FOC_Loop_U.Regen.Polarity_switch) &&
      FOC_Loop_U.Regen.Regen_Active) {
    FOC_Loop_DW.durationCounter_1++;
  } else {
    FOC_Loop_DW.durationCounter_1 = 0U;
  }

  if (FOC_Loop_Y.Throttle_error) {
    FOC_Loop_DW.durationCounter_2++;
  } else {
    FOC_Loop_DW.durationCounter_2 = 0U;
  }

  if (((FOC_Loop_U.Drive_State == 1.0F) && FOC_Loop_Y.Throttle_error) ||
      ((FOC_Loop_U.Drive_State == 2.0F) && FOC_Loop_Y.Throttle_error)) {
    FOC_Loop_DW.durationCounter_1_m++;
  } else {
    FOC_Loop_DW.durationCounter_1_m = 0U;
  }

  if (FOC_Loop_Y.Throttle_error) {
    FOC_Loop_DW.durationCounter_1_l++;
  } else {
    FOC_Loop_DW.durationCounter_1_l = 0U;
  }

  if (tmp_0 && FOC_Loop_U.Regen.Polarity_switch && FOC_Loop_U.Regen.Regen_Active)
  {
    FOC_Loop_DW.durationCounter_2_e++;
  } else {
    FOC_Loop_DW.durationCounter_2_e = 0U;
  }

  /* End of Chart: 'FNR_switching' (':4893') */

  /* Sum: 'Difference Inputs1' (':5249') incorporates:
   *  Delay: 'Delay' (':5248')
   *
   * Block description for 'Difference Inputs1' (':5249'):
   *
   *  Add in CPU
   */
  rtb_Add2 = FOC_Loop_B.T_ref - FOC_Loop_DW.Delay_DSTATE;

  /* Product: 'delta rise limit' (':5254') incorporates:
   *  Inport: 'Rate_limiter' (':5680')
   *  SampleTimeMath: 'sample time' (':5255')
   *
   * About 'sample time' (':5255'):
   *  y = K where K = ( w * Ts )
   *   */
  rtb_Add3 = FOC_Loop_U.Rate_limiter.Torque_gen_ramp_up * d.Op_ts;

  /* Switch: 'Switch2' (':5251:8') incorporates:
   *  RelationalOperator: 'LowerRelop1' (':5251:6')
   */
  if (!(rtb_Add2 > rtb_Add3)) {
    /* Product: 'delta fall limit' (':5253') incorporates:
     *  Inport: 'Rate_limiter' (':5680')
     *  SampleTimeMath: 'sample time' (':5255')
     *
     * About 'sample time' (':5255'):
     *  y = K where K = ( w * Ts )
     *   */
    rtb_Add3 = d.Op_ts * FOC_Loop_U.Rate_limiter.Torque_gen_ramp_down;

    /* Switch: 'Switch' (':5251:7') incorporates:
     *  RelationalOperator: 'UpperRelop' (':5251:9')
     */
    if (!(rtb_Add2 < rtb_Add3)) {
      rtb_Add3 = rtb_Add2;
    }

    /* End of Switch: 'Switch' (':5251:7') */
  }

  /* End of Switch: 'Switch2' (':5251:8') */

  /* Sum: 'Difference Inputs2' (':5250') incorporates:
   *  Delay: 'Delay' (':5248')
   *
   * Block description for 'Difference Inputs2' (':5250'):
   *
   *  Add in CPU
   */
  FOC_Loop_Y.T_ref_FNR = rtb_Add3 + FOC_Loop_DW.Delay_DSTATE;

  /* RateTransition generated from: 'Data Type Conversion1' (':4040') incorporates:
   *  DataTypeConversion: 'Data Type Conversion1' (':4040')
   *  Inport: 'Ref_Speed_mech_rpm' (':5684')
   *  Outport: 'Throttle_State' (':5713')
   *  RateTransition generated from: 'Speed_Loop' (':4099')
   */
  if (tmp) {
    FOC_Loop_DW.TmpRTBAtDataTypeConversion1Outp = (float)
      FOC_Loop_Y.Throttle_State;
    FOC_Loop_DW.TmpRTBAtSpeed_LoopInport1_Buffe = FOC_Loop_U.Ref_Speed_mech_rpm;

    /* RateTransition generated from: 'Speed_Loop' (':4099') incorporates:
     *  DataTypeConversion: 'Data Type Conversion1' (':4040')
     *  Inport: 'Rate_limiter' (':5680')
     *  Inport: 'Ref_Speed_mech_rpm' (':5684')
     *  Outport: 'Throttle_State' (':5713')
     */
    FOC_Loop_DW.TmpRTBAtSpeed_LoopInport3_Buffe = FOC_Loop_U.Rate_limiter;

    /* RateTransition generated from: 'Speed_Loop' (':4099') incorporates:
     *  Inport: 'T_ref' (':5692')
     */
    FOC_Loop_DW.TmpRTBAtSpeed_LoopInport6_Buffe = FOC_Loop_U.RefTrq;

    /* RateTransition generated from: 'Speed_Loop' (':4099') incorporates:
     *  Inport: 'Actual Speed_mech_rad//sec' (':5683')
     */
    FOC_Loop_DW.wm_input_Buffer = FOC_Loop_U.ActualSpeed_mech_radsec;
  }

  /* Update for DiscreteIntegrator: 'Integrator' (':42:843') incorporates:
   *  Product: 'IProd Out' (':42:787')
   *  Sum: 'SumI2' (':42:617')
   *  Sum: 'SumI4' (':42:618')
   */
#if FOC_VDQ_CIRCLE_LIMIT
  /* HAND-EDIT: back-calculate against the applied PI output (circle limit) */
  FOC_Loop_DW.Integrator_DSTATE += ((vd_pi_aw - rtb_Sum) * can_d.canId_PID.Back_Kaw +
    FOC_Loop_Y.Id_error * FOC_Loop_B.Merge[1]) * d.Op_ts;
#else
  FOC_Loop_DW.Integrator_DSTATE += ((FOC_Loop_Y.Vd_PID - rtb_Sum) * can_d.canId_PID.Back_Kaw +
    FOC_Loop_Y.Id_error * FOC_Loop_B.Merge[1]) * d.Op_ts;
#endif
  if (FOC_Loop_DW.Integrator_DSTATE > d.Vmax_SVM) {
    FOC_Loop_DW.Integrator_DSTATE = d.Vmax_SVM;
  } else if (FOC_Loop_DW.Integrator_DSTATE < -d.Vmax_SVM) {
    FOC_Loop_DW.Integrator_DSTATE = -d.Vmax_SVM;
  }

  /* End of Update for DiscreteIntegrator: 'Integrator' (':42:843') */

  /* Update for DiscreteIntegrator: 'Filter' (':42:737') */
  FOC_Loop_DW.Filter_DSTATE += d.Op_ts * rtb_DataTypeConversion1;

  /* Update for DiscreteIntegrator: 'Integrator' (':5:843') incorporates:
   *  Product: 'IProd Out' (':5:787')
   *  Sum: 'SumI2' (':5:617')
   *  Sum: 'SumI4' (':5:618')
   */
#if FOC_VDQ_CIRCLE_LIMIT
  /* HAND-EDIT: back-calculate against the applied PI output (circle limit) */
  FOC_Loop_DW.Integrator_DSTATE_h += ((vq_pi_aw - rtb_Sum_f) * can_d.canIq_PID.Back_Kaw +
    FOC_Loop_Y.Iq_error * FOC_Loop_B.Merge2[1]) * d.Op_ts;
#else
  FOC_Loop_DW.Integrator_DSTATE_h += ((FOC_Loop_Y.Vq_PID - rtb_Sum_f) * can_d.canIq_PID.Back_Kaw +
    FOC_Loop_Y.Iq_error * FOC_Loop_B.Merge2[1]) * d.Op_ts;
#endif
  if (FOC_Loop_DW.Integrator_DSTATE_h > d.Vmax_SVM) {
    FOC_Loop_DW.Integrator_DSTATE_h = d.Vmax_SVM;
  } else if (FOC_Loop_DW.Integrator_DSTATE_h < -d.Vmax_SVM) {
    FOC_Loop_DW.Integrator_DSTATE_h = -d.Vmax_SVM;
  }

  /* End of Update for DiscreteIntegrator: 'Integrator' (':5:843') */

  /* Update for DiscreteIntegrator: 'Filter' (':5:737') */
  FOC_Loop_DW.Filter_DSTATE_d += d.Op_ts * rtb_VdFF_unsat;

  /* Update for Delay: 'Delay' (':5248') */
  FOC_Loop_DW.icLoad = false;
  FOC_Loop_DW.Delay_DSTATE = FOC_Loop_Y.T_ref_FNR;
}

/* Model step function for TID1 */
void FOC_Loop_step1(void)              /* Sample time: [0.001s, 0.0s] */
{
  float tmp;
  float rtb_Sum;
  float rtb_UkYk1_e;
  float rtb_deltafalllimit_j;

  /* Outputs for Atomic SubSystem: 'Speed_Loop' (':4099') */
  /* Delay: 'Delay' (':3597') */
  if (FOC_Loop_DW.icLoad_k) {
    /* Sum: 'Difference Inputs2' (':3599') incorporates:
     *  Constant: 'Constant' (':3596')
     *
     * Block description for 'Difference Inputs2' (':3599'):
     *
     *  Add in CPU
     */
    FOC_Loop_DW.Delay_DSTATE_c = 0.0F;
  }

  /* Product: 'delta rise limit' (':3603') incorporates:
   *  RateTransition generated from: 'Speed_Loop' (':4099')
   *  SampleTimeMath: 'sample time' (':3604')
   *
   * About 'sample time' (':3604'):
   *  y = K where K = ( w * Ts )
   *   */
  rtb_deltafalllimit_j =
    FOC_Loop_DW.TmpRTBAtSpeed_LoopInport3_Buffe.Ref_Speed_rate_up * T_S;

  /* Switch: 'FNR_speed_Switch' (':3955') incorporates:
   *  Constant: 'Constant' (':3954')
   *  RateTransition generated from: 'Data Type Conversion1' (':4040')
   *  RateTransition generated from: 'Speed_Loop' (':4099')
   */
  if (FOC_Loop_DW.TmpRTBAtDataTypeConversion1Outp > 2.0F) {
    tmp = 0.0;
  } else {
    tmp = FOC_Loop_DW.TmpRTBAtSpeed_LoopInport1_Buffe;
  }

  /* Sum: 'Difference Inputs1' (':3598') incorporates:
   *  Delay: 'Delay' (':3597')
   *  Switch: 'FNR_speed_Switch' (':3955')
   *
   * Block description for 'Difference Inputs1' (':3598'):
   *
   *  Add in CPU
   */
  rtb_UkYk1_e = (float)(tmp - FOC_Loop_DW.Delay_DSTATE_c);

  /* Switch: 'Switch2' (':3600:8') incorporates:
   *  RelationalOperator: 'LowerRelop1' (':3600:6')
   */
  if (!(rtb_UkYk1_e > rtb_deltafalllimit_j)) {
    /* Switch: 'Speed_rate_limiter_switch' (':3969') incorporates:
     *  RateTransition generated from: 'Data Type Conversion1' (':4040')
     *  RateTransition generated from: 'Speed_Loop' (':4099')
     */
    if (FOC_Loop_DW.TmpRTBAtDataTypeConversion1Outp > 2.0F) {
      rtb_deltafalllimit_j =
        FOC_Loop_DW.TmpRTBAtSpeed_LoopInport3_Buffe.Ref_Speed_rate_down_neutral;
    } else {
      rtb_deltafalllimit_j =
        FOC_Loop_DW.TmpRTBAtSpeed_LoopInport3_Buffe.Ref_Speed_rate_down;
    }

    /* Product: 'delta fall limit' (':3602') incorporates:
     *  SampleTimeMath: 'sample time' (':3604')
     *  Switch: 'Speed_rate_limiter_switch' (':3969')
     *
     * About 'sample time' (':3604'):
     *  y = K where K = ( w * Ts )
     *   */
    rtb_deltafalllimit_j *= T_S;

    /* Switch: 'Switch' (':3600:7') incorporates:
     *  RelationalOperator: 'UpperRelop' (':3600:9')
     */
    if (!(rtb_UkYk1_e < rtb_deltafalllimit_j)) {
      rtb_deltafalllimit_j = rtb_UkYk1_e;
    }

    /* End of Switch: 'Switch' (':3600:7') */
  }

  /* End of Switch: 'Switch2' (':3600:8') */

  /* Sum: 'Difference Inputs2' (':3599') incorporates:
   *  Delay: 'Delay' (':3597')
   *
   * Block description for 'Difference Inputs2' (':3599'):
   *
   *  Add in CPU
   */
  FOC_Loop_DW.Delay_DSTATE_c += rtb_deltafalllimit_j;

  /* Sum: 'Speed_Error' (':4087') incorporates:
   *  Abs: 'Abs' (':4124')
   *  Gain: 'rpm to Rad//sec' (':2944')
   *  RateTransition generated from: 'Speed_Loop' (':4099')
   */
  FOC_Loop_Y.Speed_error = 0.104719758F * FOC_Loop_DW.Delay_DSTATE_c - fabsf
    (FOC_Loop_DW.wm_input_Buffer);

  /* Product: 'NProd Out' (':4086:878') incorporates:
   *  DiscreteIntegrator: 'Filter' (':4086:737')
   *  Product: 'DProd Out' (':4086:684')
   *  RateTransition generated from: 'Merge1' (':5052')
   *  Sum: 'SumD' (':4086:738')
   */
  rtb_deltafalllimit_j = (FOC_Loop_Y.Speed_error *
    FOC_Loop_DW.TmpRTBAtMerge1Outport1_Buffer[2] - FOC_Loop_DW.Filter_DSTATE_c) *
    FOC_Loop_DW.TmpRTBAtMerge1Outport1_Buffer[3];

  /* Sum: 'Sum' (':4086:3526') incorporates:
   *  DiscreteIntegrator: 'Integrator' (':4086:843')
   *  Product: 'PProd Out' (':4086:909')
   *  RateTransition generated from: 'Merge1' (':5052')
   */
  rtb_Sum = (FOC_Loop_Y.Speed_error * FOC_Loop_DW.TmpRTBAtMerge1Outport1_Buffer
             [0] + FOC_Loop_DW.Integrator_DSTATE_g) + rtb_deltafalllimit_j;

  /* Switch: 'Switch2' (':4086:3846:8') incorporates:
   *  RateTransition generated from: 'Merge1' (':5052')
   *  RelationalOperator: 'LowerRelop1' (':4086:3846:6')
   *  RelationalOperator: 'UpperRelop' (':4086:3846:9')
   *  Switch: 'Switch' (':4086:3846:7')
   */
  if (rtb_Sum > FOC_Loop_DW.TmpRTBAtMerge1Outport1_Buffer[4]) {
    /* Switch: 'Switch2' (':4086:3846:8') */
    FOC_Loop_Y.Iq_gen = FOC_Loop_DW.TmpRTBAtMerge1Outport1_Buffer[4];
  } else if (rtb_Sum < FOC_Loop_DW.TmpRTBAtMerge1Outport1_Buffer[5]) {
    /* Switch: 'Switch2' (':4086:3846:8') incorporates:
     *  Switch: 'Switch' (':4086:3846:7')
     */
    FOC_Loop_Y.Iq_gen = FOC_Loop_DW.TmpRTBAtMerge1Outport1_Buffer[5];
  } else {
    /* Switch: 'Switch2' (':4086:3846:8') incorporates:
     *  Switch: 'Switch' (':4086:3846:7')
     */
    FOC_Loop_Y.Iq_gen = rtb_Sum;
  }

  /* End of Switch: 'Switch2' (':4086:3846:8') */

  /* Switch: 'Speed_Torque_switch' (':3571') incorporates:
   *  Inport: 'Iq_Torque_ratio' (':5689')
   *  Inport: 'Speed_1_Torque_0' (':5690')
   *  Product: 'Product1' (':3552')
   *  RateTransition generated from: 'Data Type Conversion1' (':4040')
   *  RateTransition generated from: 'Speed_Loop' (':4099')
   *  Switch: 'Switch2' (':4070')
   */
  if (FOC_Loop_U.Speed_1_Torque_0) {
    rtb_UkYk1_e = FOC_Loop_Y.Iq_gen * FOC_Loop_U.Iq_Torque_ratio;
  } else if (FOC_Loop_DW.TmpRTBAtDataTypeConversion1Outp > 2.0F) {
    /* Switch: 'Switch2' (':4070') incorporates:
     *  Constant: 'Constant1' (':4069')
     */
    rtb_UkYk1_e = 0.0F;
  } else {
    rtb_UkYk1_e = FOC_Loop_DW.TmpRTBAtSpeed_LoopInport6_Buffe;
  }

  /* End of Switch: 'Speed_Torque_switch' (':3571') */

  /* Update for Delay: 'Delay' (':3597') */
  FOC_Loop_DW.icLoad_k = false;

  /* Update for DiscreteIntegrator: 'Integrator' (':4086:843') incorporates:
   *  Product: 'IProd Out' (':4086:787')
   *  RateTransition generated from: 'Merge1' (':5052')
   *  Sum: 'SumI2' (':4086:617')
   *  Sum: 'SumI4' (':4086:618')
   */
  FOC_Loop_DW.Integrator_DSTATE_g += ((FOC_Loop_Y.Iq_gen - rtb_Sum) * can_d.canSpeed_PID.Back_Kaw +
    FOC_Loop_Y.Speed_error * FOC_Loop_DW.TmpRTBAtMerge1Outport1_Buffer[1]) *
    T_S;
  if (FOC_Loop_DW.Integrator_DSTATE_g > MOTOR_PEAK_AC) {
    FOC_Loop_DW.Integrator_DSTATE_g = MOTOR_PEAK_AC;
  } else if (FOC_Loop_DW.Integrator_DSTATE_g < -MOTOR_PEAK_AC) {
    FOC_Loop_DW.Integrator_DSTATE_g = -MOTOR_PEAK_AC;
  }

  /* End of Update for DiscreteIntegrator: 'Integrator' (':4086:843') */

  /* Update for DiscreteIntegrator: 'Filter' (':4086:737') */
  FOC_Loop_DW.Filter_DSTATE_c += T_S * rtb_deltafalllimit_j;

  /* End of Outputs for SubSystem: 'Speed_Loop' (':4099') */

  /* Outport: 'Ref_speed_out_RPM' (':5710') */
  FOC_Loop_Y.Ref_speed_out_RPM = FOC_Loop_DW.Delay_DSTATE_c;

  /* RateTransition: 'Rate Transition' (':4080') */
  FOC_Loop_DW.RateTransition_Buffer0 = rtb_UkYk1_e;
}

/* Model initialize function */
void FOC_Loop_initialize(void)
{
  /* Registration code */

  /* external outputs */
  FOC_Loop_Y.Throttle_State = Forward;

  /* InitializeConditions for Delay: 'Delay' (':5248') */
  FOC_Loop_DW.icLoad = true;

  /* SystemInitialize for Outport: 'Throttle_State' (':5713') incorporates:
   *  Chart: 'FNR_switching' (':4893')
   */
  FOC_Loop_Y.Throttle_State = Forward;

  /* SystemInitialize for Atomic SubSystem: 'Speed_Loop' (':4099') */
  /* InitializeConditions for Delay: 'Delay' (':3597') */
  FOC_Loop_DW.icLoad_k = true;

  FOC_Loop_U.BusVoltage_V = OP_VOLTAGE;
  FOC_Loop_U.Pole_Pairs = POLEPAIRS;
  FOC_Loop_U.Rs = can_d.can_Rs;
  FOC_Loop_U.Ld = can_d.can_Ld;
  FOC_Loop_U.Lq = can_d.can_Lq;
  FOC_Loop_U.Lambda = can_d.can_Lambda;
  FOC_Loop_U.Iq_Torque_ratio = CURR_TORQUE_RATIO;
  FOC_Loop_U.RPM_min_limit = MIN_RPM_CHANGE_LIM;

  /* Regen Settings */
  FOC_Loop_U.Regen.max_Regen_Torque = REGEN_MAX_TORQUE;
  FOC_Loop_U.Regen.regen_cap_rpm = REGEN_CAP_RPM;
  FOC_Loop_U.Regen.RPM_Regen_min_Speed = REGEN_MIN_RPM_EN;
  FOC_Loop_U.Regen.Polarity_switch = REGEN_POLARITY;
  /* HAND-EDIT (init parameterisation) — NOT IN THE SIMULINK MODEL. Seeded
   * from the app's runtime flag instead of the generated REGEN_ACTIVE_STATE
   * constant; main.c refreshes it every torque-mode loop. */
  FOC_Loop_U.Regen.Regen_Active = d.regen_active;

  /* Low Speed PID Switch settings */
  FOC_Loop_U.MTPA_PID.Selection.Speed_RPM_limit = LOW_SPEED_TO_HIGH_SPEED_SWITCH_RPM;
  FOC_Loop_U.MTPA_PID.Selection.I_limit = LOW_SPEED_I_LIMIT;
  FOC_Loop_U.MTPA_PID.Selection.T_limit = LOW_SPEED_T_LIMIT;
  FOC_Loop_U.MTPA_PID.Selection.I_0_T_1_switch = LOW_SPEED_I_T_SWITCH;

  /* Throttle Settings */
  FOC_Loop_U.Throttle_input.Max_Output_Value = SPORTS_MAX_SPEED;
  FOC_Loop_U.Throttle_input.Max_Throttle_Voltage = THR_MAX_VAL;
  FOC_Loop_U.Throttle_input.Min_Throttle_Voltage = THR_MIN_VAL;
  FOC_Loop_U.Throttle_input.Throttle_V_Hyst_band_FNR_shift = THR_HYS_BAND;

  /* IIR Filter Coefficients */
  FOC_Loop_U.IIR_Filter_Coefficient.Id_Filter_switch = 1;
  FOC_Loop_U.IIR_Filter_Coefficient.Iq_Filter_switch = 1;
  FOC_Loop_U.IIR_Filter_Coefficient.Id_Filter_coefficient = 0.9f;
  FOC_Loop_U.IIR_Filter_Coefficient.Iq_Filter_coefficient = 0.9f; 

  FOC_Loop_U.RefTrq = 0.0f;
  FOC_Loop_U.Ref_Speed_mech_rpm = 0.0f;

  /* HAND-EDIT (init parameterisation) — NOT IN THE SIMULINK MODEL.
   * Dead-time compensation. The model hard-codes t_dead = 1 us (Constant
   * ':5314:65' -> the 1.0E-6 factor in the step function), which matches
   * this board exactly: TIM1 DeadTime = TIM1_DEAD_TIME (100), DTG bit7 = 0
   * and CKD = DIV1 on a 100 MHz timer clock -> 100 * 10 ns = 1 us. If the
   * timer clock, CKD, or DeadTime register ever change, that 1e-6 constant
   * no longer matches the hardware and this must be re-checked.
   * Switching_Freq must track the real PWM rate (10 kHz = 1/Op_ts).
   * Caveat: compensation is switched on raw phase-current SIGN, so it can
   * chatter around the current zero crossing at very low load. */
  FOC_Loop_U.Dead_Time_Compensator.Enable = DEAD_TIME_COMP_ENABLE;
  FOC_Loop_U.Dead_Time_Compensator.Switching_Freq = 10000.0f;

  FOC_Loop_U.Rate_limiter.Ref_Speed_rate_up = 100.0f * 10000.0f;
  FOC_Loop_U.Rate_limiter.Ref_Speed_rate_down = -500.0f * 10000.0f;
  FOC_Loop_U.Rate_limiter.Ref_Speed_rate_down_neutral = -600.0f * 10000.0f;

  /* HAND-EDIT (init parameterisation) — NOT IN THE SIMULINK MODEL.
   * Nm/s (model multiplies by Op_ts). Innermost torque-chain guard —
   * ordering constraints documented in the motor profile. */
  FOC_Loop_U.Rate_limiter.Torque_gen_ramp_up = FOC_TRQ_RAMP_UP_NM_S;
  FOC_Loop_U.Rate_limiter.Torque_gen_ramp_down = -FOC_TRQ_RAMP_DOWN_NM_S;
  FOC_Loop_U.Rate_limiter.Id_ramp_up = 50.0f * 10000.0f;
  FOC_Loop_U.Rate_limiter.Id_ramp_down = -50.0f * 10000.0f;
  FOC_Loop_U.Rate_limiter.Iq_ramp_up = 50.0f * 10000.0f;
  FOC_Loop_U.Rate_limiter.Iq_ramp_down = -50.0f * 10000.0f;
  
  /* Speed PID Values */
  FOC_Loop_U.MTPA_PID.Up_Limit_speed_PID_MTPA = can_d.canSpeed_PID.Output_Up_Limit;
  FOC_Loop_U.MTPA_PID.Low_Limit_speed_PID_MTPA = can_d.canSpeed_PID.Output_Low_Limit;
  /* High Speed PID Values */
  FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Kp_speed_PID_MTPA = can_d.canSpeed_PID.Kp;
  FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Ki_speed_PID_MTPA = can_d.canSpeed_PID.Ki;
  FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Kd_speed_PID_MTPA = can_d.canSpeed_PID.Kd;
  FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Filter_speed_PID_MTPA = can_d.canSpeed_PID.Kd_Filter;
  /* Low Speed PID Values */
  FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Kp_speed_PID_MTPA_low = can_d.canSpeed_PID.Kp * LOW_SPEED_SCALING;
  FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Ki_speed_PID_MTPA_low = can_d.canSpeed_PID.Ki * LOW_SPEED_SCALING;
  FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Kd_speed_PID_MTPA_low = can_d.canSpeed_PID.Kd;
  FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Filter_speed_PID_MTPA_low = can_d.canSpeed_PID.Kd_Filter;

  /* Iq PID Values */
  FOC_Loop_U.MTPA_PID.Up_Limit_torque_PID = can_d.canIq_PID.Output_Up_Limit;
  FOC_Loop_U.MTPA_PID.Low_Limit_torque_PID = can_d.canIq_PID.Output_Low_Limit;
  /* High Speed PID Values */
  FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Kp_torque_PID_MTPA = can_d.canIq_PID.Kp;
  FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Ki_torque_PID_MTPA = can_d.canIq_PID.Ki;
  FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Kd_torque_PID_MTPA = can_d.canId_PID.Kd;
  FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Filter_torque_PID_MTPA = can_d.canIq_PID.Kd_Filter;
  /* Low Speed PID Values */
  FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Kp_torque_PID_MTPA_low = can_d.canIq_PID.Kp * LOW_SPEED_SCALING;
  FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Ki_torque_PID_MTPA_low = can_d.canIq_PID.Ki * LOW_SPEED_SCALING;
  FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Kd_torque_PID_MTPA_low = can_d.canId_PID.Kd;
  FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Filter_torque_PID_MTPA_low = can_d.canIq_PID.Kd_Filter;

  /* Id PID Values */
  FOC_Loop_U.MTPA_PID.Up_Limit_flux_PID = can_d.canId_PID.Output_Up_Limit;
  FOC_Loop_U.MTPA_PID.Low_Limit_flux_PID = can_d.canId_PID.Output_Low_Limit;
  /* High Speed PID Values */
  FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Kp_flux_PID_MTPA = can_d.canId_PID.Kp;
  FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Ki_flux_PID_MTPA = can_d.canId_PID.Ki;
  FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Kd_flux_PID_MTPA = can_d.canId_PID.Kd;
  FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Filter_flux_PID_MTPA = can_d.canId_PID.Kd_Filter;
  /* Low Speed PID Values */
  FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Kp_flux_PID_MTPA_low = can_d.canId_PID.Kp * LOW_SPEED_SCALING;
  FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Ki_flux_PID_MTPA_low = can_d.canId_PID.Ki * LOW_SPEED_SCALING;
  FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Kd_flux_PID_MTPA_low = can_d.canId_PID.Kd;
  FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Filter_flux_PID_MTPA_low = can_d.canId_PID.Kd_Filter;

  FOC_Loop_Y.Throttle_State = Forward;

  /* End of SystemInitialize for SubSystem: 'Speed_Loop' (':4099') */
}

/* Model terminate function */
void FOC_Loop_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
