/*
 * File: Current_Ref_Estimator.c
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

#include "Current_Ref_Estimator.h"
#include <stdint.h>
#include "Current_Ref_Estimator_private.h"
#include <math.h>
#include "rt_nonfinite.h"
#include <stdbool.h>
#include "rt_defines.h"

/* Named constants for Chart: ''Chart' (':5546')' */
#define Current_Ref_Estimator_IN_FW    ((uint8_t)1U)
#define Current_Ref_Estimator_IN_MTPA  ((uint8_t)2U)

/* Block signals (default storage) */
B_Current_Ref_Estimator_T Current_Ref_Estimator_B;

/* Block states (default storage) */
DW_Current_Ref_Estimator_T Current_Ref_Estimator_DW;

#ifndef FW_CLOSED_LOOP
#error "FW_CLOSED_LOOP must be defined (vehicle_config.h)"
#endif
#if FW_CLOSED_LOOP
#if !defined(FW_CL_VREF_PCT) || !defined(FW_CL_KI) || !defined(FW_CL_ID_FLOOR_K) || !defined(FW_CL_ACTIVE_A)
#error "FW_CLOSED_LOOP needs FW_CL_VREF_PCT / FW_CL_KI / FW_CL_ID_FLOOR_K / FW_CL_ACTIVE_A in the motor profile"
#endif
#if !defined(FW_CL_ID_ERR_A) || !defined(FW_CL_IQ_KI) || !defined(FW_CL_IQ_RELEASE_A_S)
#error "FW_CLOSED_LOOP needs FW_CL_ID_ERR_A / FW_CL_IQ_KI / FW_CL_IQ_RELEASE_A_S in the motor profile"
#endif
#if FW_CL_VREF_PCT <= 0 || FW_CL_VREF_PCT > 100
#error "FW_CL_VREF_PCT must be in (0, 100]"
#endif
/* HAND-EDIT (closed-loop FW) — NOT IN THE SIMULINK MODEL. Extra d-axis
 * current (A, always <= 0) added on top of the MTPA Id reference. */
static float fw_cl_dId = 0.0F;
/* HAND-EDIT (closed-loop FW) — Iq ceiling (A, magnitude) applied once the
 * d-axis loop loses authority at the voltage edge; FW_CL_IQ_CAP_OFF = none. */
#define FW_CL_IQ_CAP_OFF  1.0e4F
static float fw_cl_iq_cap = FW_CL_IQ_CAP_OFF;
#endif

/* External inputs (root inport signals with default storage) */
ExtU_Current_Ref_Estimator_T Current_Ref_Estimator_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Current_Ref_Estimator_T Current_Ref_Estimator_Y;

/* Real-time model */
static RT_MODEL_Current_Ref_Estimato_T Current_Ref_Estimator_M_;
RT_MODEL_Current_Ref_Estimato_T *const Current_Ref_Estimator_M =
  &Current_Ref_Estimator_M_;

/*
 * System initialize for atomic system:
 *    'Motor_Parameter_Degrade' (':5261')
 *    'Motor_Parameter_Degrade1' (':5262')
 *    'Motor_Parameter_Degrade2' (':5376')
 */
void Cu_Motor_Parameter_Degrade_Init(DW_Motor_Parameter_Degrade_Cu_T *localDW)
{
  /* InitializeConditions for Delay: 'Delay One Step' (':5261:36') */
  localDW->icLoad = true;
}

/*
 * Output and update for atomic system:
 *    'Motor_Parameter_Degrade' (':5261')
 *    'Motor_Parameter_Degrade1' (':5262')
 *    'Motor_Parameter_Degrade2' (':5376')
 */
void Current_Motor_Parameter_Degrade(float rtu_Motor_Parameter, float
  rtu_Motor_P_Watts, float rtu_Motor_P_lim_Watts, float rtu_Motor_P_Max_Watts,
  float rtu_Percentage_decrease, float *rty_k_factor, float
  *rty_New_motor_parameter, DW_Motor_Parameter_Degrade_Cu_T *localDW)
{
  float u0;

  /* Delay: 'Delay One Step' (':5261:36') */
  if (localDW->icLoad) {
    localDW->DelayOneStep_DSTATE = rtu_Motor_Parameter;
  }

  *rty_New_motor_parameter = localDW->DelayOneStep_DSTATE;

  /* End of Delay: 'Delay One Step' (':5261:36') */

  /* Product: 'alpha' (':5261:28') incorporates:
   *  Sum: 'Subtract1' (':5261:26')
   *  Sum: 'Subtract2' (':5261:27')
   */
  u0 = (rtu_Motor_P_Watts - rtu_Motor_P_lim_Watts) / (rtu_Motor_P_Max_Watts -
    rtu_Motor_P_lim_Watts);

  /* Saturate: 'Saturation' (':5261:25') */
  if (u0 > 1.0F) {
    u0 = 1.0F;
  } else if (u0 < 0.0F) {
    u0 = 0.0F;
  }

  /* Sum: 'k' (':5261:29') incorporates:
   *  Constant: 'Constant' (':5261:22')
   *  Gain: 'Gain' (':5261:23')
   *  Product: 'Product' (':5261:24')
   *  Saturate: 'Saturation' (':5261:25')
   */
  *rty_k_factor = 1.0F - 0.01F * rtu_Percentage_decrease * u0;

  /* Update for Delay: 'Delay One Step' (':5261:36') incorporates:
   *  Product: 'Product1' (':5261:35')
   */
  localDW->icLoad = false;
  localDW->DelayOneStep_DSTATE = rtu_Motor_Parameter * *rty_k_factor;
}

float rt_atan2f_snf(float u0, float u1)
{
  float y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    int32_t tmp;
    int32_t tmp_0;
    if (u0 > 0.0F) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0F) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2f((float)tmp, (float)tmp_0);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = atan2f(u0, u1);
  }

  return y;
}

float rt_hypotf_snf(float u0, float u1)
{
  float a;
  float b;
  float y;
  a = fabsf(u0);
  b = fabsf(u1);
  if (a < b) {
    a /= b;
    y = sqrtf(a * a + 1.0F) * b;
  } else if (a > b) {
    b /= a;
    y = sqrtf(b * b + 1.0F) * a;
  } else if (rtIsNaNF(b)) {
    y = (rtNaNF);
  } else {
    y = a * 1.41421354F;
  }

  return y;
}

/* Model step function */
void Current_Ref_Estimator_step(void)
{
  float rtb_Add_k;
  float rtb_Sqrt1;
  float rtb_V_do;
#if !FW_CLOSED_LOOP
  bool guard1;
  bool guard2;
  bool rtb_RelationalOperator;
#endif

  /* Abs: 'Abs' (':5375') incorporates:
   *  Delay: 'Delay1' (':5311')
   *  Inport: 'Actual_Speed_mech' (':5680')
   *  Product: 'Product3' (':5184')
   */
  Current_Ref_Estimator_Y.Motor_P_Watts = fabsf
    (Current_Ref_Estimator_U.Actual_Speed_mech *
     Current_Ref_Estimator_DW.Delay1_DSTATE);

  /* Outputs for Atomic SubSystem: 'Motor_Parameter_Degrade1' (':5262') */
  /* Inport: 'Motor_Parameters' (':5683') incorporates:
   *  Outport: 'Lambda' (':5699')
   *  Outport: 'Lq' (':5696')
   */
  Current_Motor_Parameter_Degrade(Current_Ref_Estimator_U.Motor_Parameters.Lq,
    Current_Ref_Estimator_Y.Motor_P_Watts,
    Current_Ref_Estimator_U.Motor_Parameters.Motor_P_lim_Watts,
    Current_Ref_Estimator_U.Motor_Parameters.Motor_P_Max_Watts,
    Current_Ref_Estimator_U.Motor_Parameters.Lq_Percentage, &rtb_Add_k,
    &Current_Ref_Estimator_Y.Lq,
    &Current_Ref_Estimator_DW.Motor_Parameter_Degrade1);

  /* End of Outputs for SubSystem: 'Motor_Parameter_Degrade1' (':5262') */

  /* Outputs for Atomic SubSystem: 'Motor_Parameter_Degrade2' (':5376') */
  Current_Motor_Parameter_Degrade
    (Current_Ref_Estimator_U.Motor_Parameters.Lambda,
     Current_Ref_Estimator_Y.Motor_P_Watts,
     Current_Ref_Estimator_U.Motor_Parameters.Motor_P_lim_Watts,
     Current_Ref_Estimator_U.Motor_Parameters.Motor_P_Max_Watts,
     Current_Ref_Estimator_U.Motor_Parameters.Flux_Percentage, &rtb_Add_k,
     &Current_Ref_Estimator_Y.Lambda,
     &Current_Ref_Estimator_DW.Motor_Parameter_Degrade2);

  /* End of Outputs for SubSystem: 'Motor_Parameter_Degrade2' (':5376') */

  /* Product: 'Divide' (':5504') incorporates:
   *  Abs: 'Abs1' (':5497')
   *  Constant: 'Constant1' (':5502')
   *  Gain: 'Gain' (':5507')
   *  Inport: 'Motor_Parameters' (':5683')
   *  Inport: 'T_ref_FNR' (':5684')
   *  Outport: 'Lambda' (':5699')
   *  Product: 'Product' (':5514')
   *
   * HAND-EDIT (pointer, no code change): when FOC_SALIENT_TORQUE_INVERSE is 1,
   * this non-salient inverse and the clamp below no longer determine I_mag —
   * both are recomputed after 'Motor_Parameter_Degrade' (':5261'); see the
   * salient-inverse block there. When 0 (fix disabled) this path stands and
   * I_qo determines I_mag as generated. I_qo is also a live diagnostic signal.
   */
  Current_Ref_Estimator_Y.I_qo = fabsf(2.0F *
    Current_Ref_Estimator_U.T_ref_final) / (float)
    (Current_Ref_Estimator_Y.Lambda *
     Current_Ref_Estimator_U.Motor_Parameters.Pole_Pairs * 3.0);

  /* Switch: 'Switch2' (':5517:8') incorporates:
   *  Inport: 'Id_Iq_MTPA_limit' (':5681')
   *  RelationalOperator: 'LowerRelop1' (':5517:6')
   *  RelationalOperator: 'UpperRelop' (':5517:9')
   *  Switch: 'Switch' (':5517:7')
   */
  if (Current_Ref_Estimator_Y.I_qo >
      Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Iq_up_limit) {
    /* Switch: 'Switch2' (':5517:8') */
    Current_Ref_Estimator_Y.I_mag =
      Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Iq_up_limit;
  } else if (Current_Ref_Estimator_Y.I_qo <
             Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Iq_low_limit) {
    /* Switch: 'Switch2' (':5517:8') incorporates:
     *  Switch: 'Switch' (':5517:7')
     */
    Current_Ref_Estimator_Y.I_mag =
      Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Iq_low_limit;
  } else {
    /* Switch: 'Switch2' (':5517:8') incorporates:
     *  Switch: 'Switch' (':5517:7')
     */
    Current_Ref_Estimator_Y.I_mag = Current_Ref_Estimator_Y.I_qo;
  }

  /* End of Switch: 'Switch2' (':5517:8') */

  /* Math: 'Math Function2' (':5512') */
  rtb_Sqrt1 = Current_Ref_Estimator_Y.I_mag * Current_Ref_Estimator_Y.I_mag;

  /* Outputs for Atomic SubSystem: 'Motor_Parameter_Degrade' (':5261') */
  /* Inport: 'Motor_Parameters' (':5683') incorporates:
   *  Outport: 'Ld' (':5695')
   */
  Current_Motor_Parameter_Degrade(Current_Ref_Estimator_U.Motor_Parameters.Ld,
    Current_Ref_Estimator_Y.Motor_P_Watts,
    Current_Ref_Estimator_U.Motor_Parameters.Motor_P_lim_Watts,
    Current_Ref_Estimator_U.Motor_Parameters.Motor_P_Max_Watts,
    Current_Ref_Estimator_U.Motor_Parameters.Ld_Percentage, &rtb_Add_k,
    &Current_Ref_Estimator_Y.Ld,
    &Current_Ref_Estimator_DW.Motor_Parameter_Degrade);

  /* End of Outputs for SubSystem: 'Motor_Parameter_Degrade' (':5261') */

  /* HAND-EDIT (salient torque inverse) — NOT IN THE SIMULINK MODEL.
   *
   * THE DEFECT. 'Divide' (':5504') above inverts the NON-SALIENT torque
   * equation, I_qo = |2*T_ref| / (3*P*lambda), and 'Switch2' (':5517:8')
   * feeds that q-axis current into the MTPA split (':5521'/':5522') as the
   * TOTAL current MAGNITUDE. 'Torque_calculation' (':5575') then evaluates
   * the SALIENT equation, 1.5*P*(lambda*Iq + (Ld-Lq)*Id*Iq), which includes
   * the reluctance term the inverse ignored. The two are not inverses, so
   * commanded torque was not delivered torque:
   *
   *     T_out ~= T_cmd/sqrt(2) + (Lq-Ld)*T_cmd^2 / (3*P*lambda^2)
   *
   * Two compounding errors: a q-axis current reused as a magnitude (Iq falls
   * to I_mag/sqrt(2), -29%), and the reluctance torque the split just created
   * being absent from the inverse (much larger, and quadratic). The error
   * scales as (Lq-Ld)/lambda^2, so the low-flux/high-saliency default profile
   * is the worst case: a 40 Nm SPORTS command produced 338 A and delivered
   * 90.9 Nm (0x724 measured 88 Nm on the vehicle); 16 Nm ECO delivered 22.4.
   * Below the crossover (7.7 Nm here, 27.3 Nm on LivGuard) it UNDER-delivers.
   *
   * THE FIX. Solve the salient equation ON the MTPA locus for the current
   * magnitude. With q = -Id and c = lambda/(Lq-Ld), the locus gives
   * Iq = sqrt(q*(q+c)) and I_mag = sqrt(q*(2q+c)), and the torque equation
   * collapses to T = 1.5*P*(Lq-Ld) * (q+c)^(3/2) * q^(1/2). Normalising
   * x = q/c leaves a single scalar root-find,
   *
   *     x*(x+1)^3 = k,   k = ( T_ref*(Lq-Ld) / (1.5*P*lambda^2) )^2
   *
   * which is dimensionless and well conditioned (k stays O(1..40) across the
   * whole torque range, unlike the un-normalised form which reaches 1e9).
   *
   * Placed after 'Motor_Parameter_Degrade' (':5261') because that block is
   * what produces the degraded Ld the MTPA split will use; recomputes I_mag
   * and its square in place, so ':5504'/':5517' above are left untouched and
   * I_qo survives as a diagnostic. Once the model is corrected, ':5504' should
   * be replaced by this solve and this block deleted.
   *
   * NOTE FOR TUNING: every torque cap is a COMMAND, so all of them were
   * calibrated against the inflated axis and now mean what they say. See
   * HANDOFF_HILLHOLD_AND_FW.md for the rescaled values that reproduce
   * today's delivered torque exactly.
   *
   * A/B TOGGLE (FOC_SALIENT_TORQUE_INVERSE, vehicle_config.h): when 0 this whole
   * block is compiled out and the generated non-salient I_qo path above stands
   * (inflated torque, matching pre-fix tuning); when 1 the solve below overwrites
   * I_mag / rtb_Sqrt1 with the correct salient values (honest torque, needs the
   * cap rescale in HANDOFF_HILLHOLD_AND_FW.md Issue 3).
   */
#if FOC_SALIENT_TORQUE_INVERSE
  {
    float dL = Current_Ref_Estimator_Y.Lq - Current_Ref_Estimator_Y.Ld;

    /* Thresholds, not >0: a (near-)non-salient machine has no reluctance
     * torque to account for, and c = lambda/dL would overflow. The generated
     * non-salient inverse above is already correct in that limit. */
    if ((dL > 1.0e-6F) && (Current_Ref_Estimator_Y.Lambda > 1.0e-3F)) {
      float c;
      float k;
      float x;
      int32_t i;

      c = Current_Ref_Estimator_Y.Lambda / dL;
      x = fabsf(Current_Ref_Estimator_U.T_ref_final) * dL /
        (1.5F * Current_Ref_Estimator_U.Motor_Parameters.Pole_Pairs *
         Current_Ref_Estimator_Y.Lambda * Current_Ref_Estimator_Y.Lambda);
      k = x * x;

      /* Seed with the tighter of the two upper bounds on the root:
       * x <= k        because (x+1)^3 >= 1, and
       * x <= k^(1/4)  because (x+1)^3 >= x^3.
       * k <= 1 makes the first tighter, k > 1 the second. */
      x = (k > 1.0F) ? sqrtf(sqrtf(k)) : k;

      /* f(x) = x*(x+1)^3 - k is increasing and convex for x >= 0, so Newton
       * started at or above the root descends to it monotonically: no
       * bracketing, no overshoot, and a fixed iteration count keeps the ISR
       * constant-time. 5 iterations reaches the float32 floor (max error
       * 4e-5 Nm) over 0..120 Nm on both motor profiles and across the whole
       * Ld/Lq degrade range; 4 gives 0.01 Nm, 3 gives 1 Nm. */
      for (i = 0; i < 5; i++) {
        float s = x + 1.0F;
        float s2 = s * s;
        x -= (x * s2 * s - k) / (s2 * (4.0F * x + 1.0F));
      }

      if (x < 0.0F) {                  /* rounding backstop */
        x = 0.0F;
      }

      Current_Ref_Estimator_Y.I_mag = c * sqrtf(x * (2.0F * x + 1.0F));

      /* Re-apply the model's current clamp ('Switch2' ':5517:8'). Only the
       * upper limit can act: I_mag is a magnitude, so the Iq_low_limit branch
       * is unreachable here exactly as it was in the generated code. The
       * clamp is now a true current-circle radius rather than a q-axis
       * bound, which is what IQ_UP_LIMIT was always sized to be. */
      if (Current_Ref_Estimator_Y.I_mag >
          Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Iq_up_limit) {
        Current_Ref_Estimator_Y.I_mag =
          Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Iq_up_limit;
      }

      /* 'Math Function2' (':5512') — recomputed from the corrected I_mag. */
      rtb_Sqrt1 = Current_Ref_Estimator_Y.I_mag *
        Current_Ref_Estimator_Y.I_mag;
    }
  }
#endif /* FOC_SALIENT_TORQUE_INVERSE */

  /* Product: 'Divide1' (':5505') incorporates:
   *  Gain: 'Gain2' (':5508')
   *  Outport: 'Lambda' (':5699')
   *  Outport: 'Ld' (':5695')
   *  Outport: 'Lq' (':5696')
   *  Sum: 'Subtract2' (':5523')
   */
  rtb_V_do = Current_Ref_Estimator_Y.Lambda / ((Current_Ref_Estimator_Y.Lq -
    Current_Ref_Estimator_Y.Ld) * 4.0F);

  /* Sum: 'Subtract' (':5521') incorporates:
   *  Abs: 'Abs2' (':5498')
   *  Gain: 'Gain4' (':5509')
   *  Math: 'Math Function' (':5511')
   *  Sqrt: 'Sqrt' (':5519')
   *  Sum: 'Add' (':5499')
   */
  Current_Ref_Estimator_Y.Id_ref_MTPA = rtb_V_do - sqrtf(fabsf(rtb_V_do *
    rtb_V_do + 0.5F * rtb_Sqrt1));

  /* Switch: 'Switch2' (':5516:8') incorporates:
   *  Inport: 'Id_Iq_MTPA_limit' (':5681')
   *  RelationalOperator: 'LowerRelop1' (':5516:6')
   *  RelationalOperator: 'UpperRelop' (':5516:9')
   *  Switch: 'Switch' (':5516:7')
   */
  if (Current_Ref_Estimator_Y.Id_ref_MTPA >
      Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Id_up_limit) {
    /* Sum: 'Subtract' (':5521') */
    Current_Ref_Estimator_Y.Id_ref_MTPA =
      Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Id_up_limit;
  } else if (Current_Ref_Estimator_Y.Id_ref_MTPA <
             Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Id_low_limit) {
    /* Sum: 'Subtract' (':5521') incorporates:
     *  Switch: 'Switch' (':5516:7')
     */
    Current_Ref_Estimator_Y.Id_ref_MTPA =
      Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Id_low_limit;
  }

  /* End of Switch: 'Switch2' (':5516:8') */

  /* Signum: 'Sign' (':5518') incorporates:
   *  Inport: 'T_ref_FNR' (':5684')
   */
  if (rtIsNaNF(Current_Ref_Estimator_U.T_ref_final)) {
    rtb_V_do = (rtNaNF);
  } else if (Current_Ref_Estimator_U.T_ref_final < 0.0F) {
    rtb_V_do = -1.0F;
  } else {
    rtb_V_do = (float)(Current_Ref_Estimator_U.T_ref_final > 0.0F);
  }

  /* Product: 'Product1' (':5515') incorporates:
   *  Abs: 'Abs' (':5496')
   *  Math: 'Math Function3' (':5513')
   *  Signum: 'Sign' (':5518')
   *  Sqrt: 'Sqrt1' (':5520')
   *  Sum: 'Subtract1' (':5522')
   */
  Current_Ref_Estimator_Y.Iq_ref_MTPA = sqrtf(fabsf(rtb_Sqrt1 -
    Current_Ref_Estimator_Y.Id_ref_MTPA * Current_Ref_Estimator_Y.Id_ref_MTPA)) *
    rtb_V_do;

  /* Product: 'Product' (':5559') incorporates:
   *  Inport: 'Actual_Speed_mech' (':5680')
   *  Inport: 'Motor_Parameters' (':5683')
   */
  rtb_Add_k = Current_Ref_Estimator_U.Motor_Parameters.Pole_Pairs *
    Current_Ref_Estimator_U.Actual_Speed_mech;

  /* Gain: 'Gain' (':5552') incorporates:
   *  Outport: 'Lq' (':5696')
   *  Product: 'Product1' (':5560')
   */
  rtb_V_do = -(rtb_Add_k * Current_Ref_Estimator_Y.Lq *
               Current_Ref_Estimator_Y.Iq_ref_MTPA);

  /* Product: 'Product3' (':5562') incorporates:
   *  Outport: 'Lambda' (':5699')
   *  Outport: 'Ld' (':5695')
   *  Product: 'Product2' (':5561')
   *  Sum: 'Add' (':5543')
   */
  rtb_Add_k *= Current_Ref_Estimator_Y.Ld * Current_Ref_Estimator_Y.Id_ref_MTPA
    + Current_Ref_Estimator_Y.Lambda;

  /* Sum: 'Subtract' (':5567') incorporates:
   *  Gain: 'Gain1' (':5553')
   *  Inport: 'Bus_Voltage' (':5687')
   *  Inport: 'Is_max' (':5685')
   *  Inport: 'Motor_Parameters' (':5683')
   *  Product: 'Product4' (':5563')
   */
  Current_Ref_Estimator_Y.V_max = 0.577350259F *
    Current_Ref_Estimator_U.BusVoltage_V - Current_Ref_Estimator_U.Is_max *
    Current_Ref_Estimator_U.Motor_Parameters.Rs;

  /* Outputs for IfAction SubSystem: 'If Action Subsystem1' (':5425') incorporates:
   *  ActionPort: 'Action Port' (':5433')
   */
  /* If: 'If' (':5418') incorporates:
   *  Math: 'Square' (':5565')
   *  Math: 'Square1' (':5566')
   *  Sqrt: 'Sqrt' (':5564')
   *  Sqrt: 'Sqrt' (':5450:459')
   *  Sum: 'Add1' (':5544')
   */
  /* Use the previous step's d/q voltage components for the voltage magnitude. */
  Current_Ref_Estimator_Y.V_s = sqrtf(
    Current_Ref_Estimator_DW.Vd_UnitDelay_DSTATE *
    Current_Ref_Estimator_DW.Vd_UnitDelay_DSTATE +
    Current_Ref_Estimator_DW.Vq_UnitDelay_DSTATE *
    Current_Ref_Estimator_DW.Vq_UnitDelay_DSTATE);

  /* Unit Delay updates occur after reading the old state. */
  Current_Ref_Estimator_DW.Vd_UnitDelay_DSTATE = rtb_V_do;
  Current_Ref_Estimator_DW.Vq_UnitDelay_DSTATE = rtb_Add_k;

  /* End of Outputs for SubSystem: 'If Action Subsystem1' (':5425') */

#if FW_CLOSED_LOOP
  /* HAND-EDIT (closed-loop FW) — NOT IN THE SIMULINK MODEL. Replaces 'Chart'
   * (':5546') and the FW integrator in 'If Action Subsystem1' (':5425') below.
   *
   * THE DEFECT. The generated regulator integrates (1 - V_s/V_max), but V_s
   * above is built from Id/Iq_ref_MTPA — computed before FW is applied — so
   * nothing FW does can move it. It also only rotates the MTPA vector at
   * constant magnitude, so -Id can never exceed |I_MTPA|. Dyno peak_power_2/3
   * (HiRange TR, 66 V supply, 2500 rpm): Id_ref stayed within 1-2 A of MTPA
   * while "FW active", Vd pinned at -Vbus/sqrt3 (all of it spent cancelling
   * we*Lq*Iq), then Id ran positive once Iq_ref crossed ~Vd_max/(we*Lq) and
   * speed collapsed to ~1700 rpm / 6 kW.
   *
   * THE FIX. Standard voltage-feedback FW. Feedback is the actual commanded
   * voltage |Vd,Vq| from the current PIs (FOC_Loop_Y, one 10 kHz step old),
   * which does fall as -Id rises, so the loop settles where |V| = V_ref.
   * Output is extra -Id on top of MTPA; Iq is then trimmed to the Is_max
   * circle. Below base speed |V| < V_ref, dId winds back to 0 and this is
   * exactly MTPA — no MTPA<->FW state machine, so no hysteresis to chatter.
   * Integral-only: a P term would feed PI-output noise straight into Id_ref. */
  {
    float v_lim = 0.577350259F * Current_Ref_Estimator_U.BusVoltage_V;
    float v_ref = (float)FW_CL_VREF_PCT * 0.01F * v_lim;
#if FOC_VDQ_CIRCLE_LIMIT
    /* FOC_Loop_Y.Vd/Vq are circle-limited (<= v_lim), which would cap the
     * FW error at v_ref - v_lim; the demanded magnitude keeps the full push
     * when the current loops ask for more than the circle holds. */
    float v_meas = d.vdq_demand;
#else
    float v_meas = sqrtf(FOC_Loop_Y.Vd * FOC_Loop_Y.Vd +
                         FOC_Loop_Y.Vq * FOC_Loop_Y.Vq);
#endif
    float id_mtpa = Current_Ref_Estimator_Y.Id_ref_MTPA;
    float id_floor = Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Id_low_limit;
    float is_max = Current_Ref_Estimator_U.Is_max;
    float id_ref;
    float iq_ref;
    float iq_room;

    /* Id floor. -Lambda/Ld is where the d-axis flux reaches zero; with
     * Lq >> Ld the minimum-|V| point for a given torque sits beyond it, so
     * FW_CL_ID_FLOOR_K (> 1) allows that. Uses the degraded Ld/Lambda the
     * MTPA split above used. Hard-capped at the Is_max circle: the added -Id
     * must never by itself exceed the current rating. */
    if (Current_Ref_Estimator_Y.Ld > 1.0e-7F) {
      float id_char = -(float)FW_CL_ID_FLOOR_K * Current_Ref_Estimator_Y.Lambda /
        Current_Ref_Estimator_Y.Ld;
      if (id_char > id_floor) {
        id_floor = id_char;
      }
    }

    if (id_floor < -is_max) {
      id_floor = -is_max;
    }

    /* Never let the floor sit above MTPA's own Id (very high torque): FW may
     * only add -Id, never take it away. */
    if (id_floor > id_mtpa) {
      id_floor = id_mtpa;
    }

    fw_cl_dId += (float)FW_CL_KI * (v_ref - v_meas) * d.Op_ts;
    if (fw_cl_dId > 0.0F) {
      fw_cl_dId = 0.0F;
    }

    if (id_mtpa + fw_cl_dId < id_floor) {
      fw_cl_dId = id_floor - id_mtpa;   /* anti-windup at the floor */
    }

    id_ref = id_mtpa + fw_cl_dId;
    iq_ref = Current_Ref_Estimator_Y.Iq_ref_MTPA;

    /* Trim Iq to the Is_max circle, but only while FW is adding -Id: with
     * dId = 0 this is bit-for-bit the MTPA reference (peak-torque behaviour
     * below base speed is unchanged, including the up-to-120 % reference
     * headroom above Is_max). */
    if (fw_cl_dId < 0.0F) {
      iq_room = is_max * is_max - id_ref * id_ref;
      if (iq_room <= 0.0F) {
        iq_ref = 0.0F;
      } else {
        float iq_max = sqrtf(iq_room);
        if (fabsf(iq_ref) > iq_max) {
          iq_ref = copysignf(iq_max, iq_ref);
        }
      }
    }

    /* Iq ceiling at the voltage edge. Once Id sits on the floor (or MTPA is
     * already past it), more load can only come from Iq, and Vd must cancel
     * we*Lq*Iq. When that exceeds the Vd rail the d-axis loop loses
     * authority: measured Id drifts positive away from Id_ref, the
     * reluctance torque vanishes and speed collapses (so_close_yet_so_far,
     * 75.2 s: Id -327 -> -247 -> +75 A with Id_ref -335). Detect exactly
     * that — Id tracking error while voltage-limited — and stop Iq from
     * growing, so overload becomes a small speed droop instead of a cliff.
     * The sign is stabilising: less Iq frees Vd, Id comes back. Inactive
     * below base speed (v_meas < v_ref) and whenever Id tracks. */
    {
      float id_err = FOC_Loop_Y.Id - Current_Ref_Estimator_Y.Id_ref_final;
      float iq_abs = fabsf(iq_ref);

      if ((v_meas > v_ref) && (id_err > (float)FW_CL_ID_ERR_A)) {
        if (fw_cl_iq_cap > iq_abs) {
          fw_cl_iq_cap = iq_abs;      /* seed at the present reference */
        }

        fw_cl_iq_cap -= (float)FW_CL_IQ_KI * (id_err - (float)FW_CL_ID_ERR_A) *
          d.Op_ts;
        if (fw_cl_iq_cap < 0.0F) {
          fw_cl_iq_cap = 0.0F;
        }
      } else if (fw_cl_iq_cap < FW_CL_IQ_CAP_OFF) {
        fw_cl_iq_cap += (float)FW_CL_IQ_RELEASE_A_S * d.Op_ts;
        if (fw_cl_iq_cap > is_max) {
          fw_cl_iq_cap = FW_CL_IQ_CAP_OFF;
        }
      }

      if (iq_abs > fw_cl_iq_cap) {
        iq_ref = copysignf(fw_cl_iq_cap, iq_ref);
      }
    }

    Current_Ref_Estimator_B.Merge[0] = id_ref;
    Current_Ref_Estimator_B.Merge[1] = iq_ref;

    /* Outport: 'MTPA_FWC_condition' (':5703') — 0x715 byte 2 telemetry. */
    Current_Ref_Estimator_Y.MTPA_FWC_condition =
      (fw_cl_dId < -(float)FW_CL_ACTIVE_A);
  }
#else

  /* Chart: 'Chart' (':5546') */
  guard1 = false;
  guard2 = false;
  if (Current_Ref_Estimator_DW.is_active_c1_Current_Ref_Estima == 0) {
    Current_Ref_Estimator_DW.is_active_c1_Current_Ref_Estima = 1U;
    Current_Ref_Estimator_DW.is_c1_Current_Ref_Estimator =
      Current_Ref_Estimator_IN_MTPA;

    /* Outport: 'MTPA_FWC_condition' (':5703') */
    Current_Ref_Estimator_Y.MTPA_FWC_condition = false;
    guard2 = true;
  } else if (Current_Ref_Estimator_DW.is_c1_Current_Ref_Estimator ==
             Current_Ref_Estimator_IN_FW) {
    if (Current_Ref_Estimator_Y.V_s <
        Current_Ref_Estimator_U.FWC_values.F2M_factor *
        Current_Ref_Estimator_Y.V_max) {
      Current_Ref_Estimator_DW.is_c1_Current_Ref_Estimator =
        Current_Ref_Estimator_IN_MTPA;

      /* Outport: 'MTPA_FWC_condition' (':5703') */
      Current_Ref_Estimator_Y.MTPA_FWC_condition = false;
      guard2 = true;
    } else {
      /* Outport: 'MTPA_FWC_condition' (':5703') */
      Current_Ref_Estimator_Y.MTPA_FWC_condition = true;
      guard1 = true;
    }

    /* case IN_MTPA: */
  } else if (Current_Ref_Estimator_Y.V_s >
             Current_Ref_Estimator_U.FWC_values.M2F_factor *
             Current_Ref_Estimator_Y.V_max) {
    Current_Ref_Estimator_DW.is_c1_Current_Ref_Estimator =
      Current_Ref_Estimator_IN_FW;

    /* Outport: 'MTPA_FWC_condition' (':5703') */
    Current_Ref_Estimator_Y.MTPA_FWC_condition = true;
    guard1 = true;
  } else {
    /* Outport: 'MTPA_FWC_condition' (':5703') */
    Current_Ref_Estimator_Y.MTPA_FWC_condition = false;
    guard2 = true;
  }

  if (guard2) {
    /* Outputs for IfAction SubSystem: 'If Action Subsystem' (':5419') incorporates:
     *  ActionPort: 'Action Port' (':5422')
     */
    /* If: 'If' (':5418') incorporates:
     *  Merge: 'Merge' (':5530')
     *  SignalConversion generated from: 'Id_ref' (':5420')
     *  SignalConversion generated from: 'Iq_ref' (':5421')
     */
    Current_Ref_Estimator_B.Merge[0] = Current_Ref_Estimator_Y.Id_ref_MTPA;
    Current_Ref_Estimator_B.Merge[1] = Current_Ref_Estimator_Y.Iq_ref_MTPA;

    /* End of Outputs for SubSystem: 'If Action Subsystem' (':5419') */
  }

  if (guard1) {
    /* Outputs for IfAction SubSystem: 'If Action Subsystem1' (':5425') incorporates:
     *  ActionPort: 'Action Port' (':5433')
     */
    /* If: 'If' (':5418') incorporates:
     *  Bias: 'Bias' (':5474')
     *  Bias: 'Bias' (':5479')
     *  Constant: 'Constant' (':5446')
     *  Constant: 'Constant1' (':5447')
     *  Constant: 'Modulation index threshold' (':5451')
     *  Constant: 'Constant1' (':5450:455')
     *  DiscreteIntegrator: 'Discrete-Time Integrator' (':5449')
     *  Fcn: 'x->r' (':5445:155')
     *  Fcn: 'x->theta' (':5445:156')
     *  Fcn: 'r->x' (':5453:192')
     *  Fcn: 'theta->y' (':5453:193')
     *  If: 'If' (':5470')
     *  Inport: 'FWC_values' (':5686')
     *  Merge: 'Merge' (':5530')
     *  Product: 'Product' (':5454')
     *  Product: 'Product1' (':5455')
     *  Product: 'Product2' (':5456')
     *  Product: 'Product' (':5450:458')
     *  Product: 'Product' (':5489')
     *  RelationalOperator: 'Relational Operator' (':5490')
     *  Saturate: 'Saturation' (':5457')
     *  SignalConversion generated from: 'Out1' (':5491')
     *  Sqrt: 'Sqrt' (':5564')
     *  Sum: 'Sum' (':5458')
     *  Sum: 'Sum1' (':5459')
     *  Sum: 'Sum2' (':5460')
     *  Sum: 'Sum3' (':5461')
     *  Sum: 'Sum4' (':5462')
     *  Switch: 'Switch' (':5450:456')
     *  UnitDelay: 'Unit Delay' (':5463')
     * */
    rtb_RelationalOperator = (Current_Ref_Estimator_Y.V_s <=
      Current_Ref_Estimator_Y.V_max *
      Current_Ref_Estimator_U.FWC_values.FW_Int_reset_factor);
    if (rtb_RelationalOperator &&
        (Current_Ref_Estimator_DW.DiscreteTimeIntegrator_PrevRese <= 0)) {
      Current_Ref_Estimator_DW.DiscreteTimeIntegrator_DSTATE = 1.0F;
    }

    if (Current_Ref_Estimator_Y.V_max > 0.1F) {
      rtb_V_do = Current_Ref_Estimator_Y.V_max;
    } else {
      rtb_V_do = 0.1F;
    }

    Current_Ref_Estimator_DW.DiscreteTimeIntegrator_DSTATE += ((1.0F -
      Current_Ref_Estimator_Y.V_s / rtb_V_do) *
      Current_Ref_Estimator_U.FWC_values.Kfw +
      Current_Ref_Estimator_DW.UnitDelay_DSTATE *
      Current_Ref_Estimator_U.FWC_values.Kaw) * d.Op_ts;
    if (Current_Ref_Estimator_DW.DiscreteTimeIntegrator_DSTATE > 1.0F) {
      rtb_V_do = 1.0F;
    } else if (Current_Ref_Estimator_DW.DiscreteTimeIntegrator_DSTATE < 0.0F) {
      rtb_V_do = 0.0F;
    } else {
      rtb_V_do = Current_Ref_Estimator_DW.DiscreteTimeIntegrator_DSTATE;
    }

    Current_Ref_Estimator_DW.UnitDelay_DSTATE = rtb_V_do -
      Current_Ref_Estimator_DW.DiscreteTimeIntegrator_DSTATE;

    {
      float angle_MTPA = rt_atan2f_snf(Current_Ref_Estimator_Y.Iq_ref_MTPA, Current_Ref_Estimator_Y.Id_ref_MTPA);

      float fw_target = (angle_MTPA >= 0.0F) ? (float)3.14159265f : -(float)3.14159265f;

      rtb_V_do = fw_target - (fw_target - angle_MTPA) * rtb_V_do;
    }
    
    // rtb_Add_k = rt_atan2f_snf(Current_Ref_Estimator_Y.Iq_ref_MTPA,
    //   Current_Ref_Estimator_Y.Id_ref_MTPA);
    // if (rtb_Add_k >= 3.14159274F) {
    //   /* Outputs for IfAction SubSystem: 'If Action Subsystem' (':5471') incorporates:
    //    *  ActionPort: 'Action Port' (':5473')
    //    */
    //   rtb_Add_k -= 3.14159274F;

    //   /* End of Outputs for SubSystem: 'If Action Subsystem' (':5471') */
    // } else if (rtb_Add_k <= -3.14159274F) {
    //   /* Outputs for IfAction SubSystem: 'If Action Subsystem1' (':5476') incorporates:
    //    *  ActionPort: 'Action Port' (':5478')
    //    */
    //   rtb_Add_k += 3.14159274F;

    //   /* End of Outputs for SubSystem: 'If Action Subsystem1' (':5476') */
    // }

    // rtb_V_do = (float)(3.1415926535897931 - (float)(3.1415926535897931 -
    //   rtb_Add_k) * rtb_V_do);
    rtb_Add_k = rt_hypotf_snf(Current_Ref_Estimator_Y.Id_ref_MTPA,
      Current_Ref_Estimator_Y.Iq_ref_MTPA);
    Current_Ref_Estimator_B.Merge[0] = rtb_Add_k * cosf(rtb_V_do);
    Current_Ref_Estimator_B.Merge[1] = rtb_Add_k * sinf(rtb_V_do);
    Current_Ref_Estimator_DW.DiscreteTimeIntegrator_PrevRese = (int8_t)
      rtb_RelationalOperator;

    /* End of Outputs for SubSystem: 'If Action Subsystem1' (':5425') */
  }

  /* End of Chart: 'Chart' (':5546') */
#endif /* FW_CLOSED_LOOP */

  /* Outport: 'Ldb' (':5701') incorporates:
   *  Outport: 'Ld' (':5695')
   */
  Current_Ref_Estimator_Y.Ldb = Current_Ref_Estimator_Y.Ld;

  /* Delay: 'Delay' (':5531:19') */
  if (Current_Ref_Estimator_DW.icLoad) {
    /* Sum: 'Difference Inputs2' (':5531:21')
     *
     * Block description for 'Difference Inputs2' (':5531:21'):
     *
     *  Add in CPU
     */
    Current_Ref_Estimator_DW.Delay_DSTATE = Current_Ref_Estimator_B.Merge[0];
  }

  /* Product: 'delta rise limit' (':5531:26') incorporates:
   *  Inport: 'Rate_limiter' (':5682')
   *  SampleTimeMath: 'sample time' (':5531:27')
   *
   * About 'sample time' (':5531:27'):
   *  y = K where K = ( w * Ts )
   *   */
  rtb_V_do = Current_Ref_Estimator_U.Rate_limiter.Id_ramp_up * d.Op_ts;

  /* Sum: 'Difference Inputs1' (':5531:20') incorporates:
   *  Delay: 'Delay' (':5531:19')
   *
   * Block description for 'Difference Inputs1' (':5531:20'):
   *
   *  Add in CPU
   */
  rtb_Add_k = Current_Ref_Estimator_B.Merge[0] -
    Current_Ref_Estimator_DW.Delay_DSTATE;

  /* Switch: 'Switch2' (':5531:23:8') incorporates:
   *  RelationalOperator: 'LowerRelop1' (':5531:23:6')
   */
  if (!(rtb_Add_k > rtb_V_do)) {
    /* Product: 'delta fall limit' (':5531:25') incorporates:
     *  Inport: 'Rate_limiter' (':5682')
     *  SampleTimeMath: 'sample time' (':5531:27')
     *
     * About 'sample time' (':5531:27'):
     *  y = K where K = ( w * Ts )
     *   */
    rtb_V_do = d.Op_ts * Current_Ref_Estimator_U.Rate_limiter.Id_ramp_down;

    /* Switch: 'Switch' (':5531:23:7') incorporates:
     *  RelationalOperator: 'UpperRelop' (':5531:23:9')
     */
    if (!(rtb_Add_k < rtb_V_do)) {
      rtb_V_do = rtb_Add_k;
    }

    /* End of Switch: 'Switch' (':5531:23:7') */
  }

  /* End of Switch: 'Switch2' (':5531:23:8') */

  /* Sum: 'Difference Inputs2' (':5531:21') incorporates:
   *  Delay: 'Delay' (':5531:19')
   *
   * Block description for 'Difference Inputs2' (':5531:21'):
   *
   *  Add in CPU
   */
  Current_Ref_Estimator_DW.Delay_DSTATE += rtb_V_do;

  /* Switch: 'Switch2' (':5533:8') incorporates:
   *  Inport: 'Id_Iq_MTPA_limit' (':5681')
   *  RelationalOperator: 'LowerRelop1' (':5533:6')
   *  RelationalOperator: 'UpperRelop' (':5533:9')
   *  Switch: 'Switch' (':5533:7')
   */
  if (Current_Ref_Estimator_DW.Delay_DSTATE >
      Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Id_up_limit) {
    /* Switch: 'Switch2' (':5533:8') */
    Current_Ref_Estimator_Y.Id_ref_final =
      Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Id_up_limit;
  } else if (Current_Ref_Estimator_DW.Delay_DSTATE <
             Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Id_low_limit) {
    /* Switch: 'Switch2' (':5533:8') incorporates:
     *  Switch: 'Switch' (':5533:7')
     */
    Current_Ref_Estimator_Y.Id_ref_final =
      Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Id_low_limit;
  } else {
    /* Switch: 'Switch2' (':5533:8') incorporates:
     *  Switch: 'Switch' (':5533:7')
     */
    Current_Ref_Estimator_Y.Id_ref_final = Current_Ref_Estimator_DW.Delay_DSTATE;
  }

  /* End of Switch: 'Switch2' (':5533:8') */

  /* Delay: 'Delay' (':5532:19') */
  if (Current_Ref_Estimator_DW.icLoad_b) {
    /* Sum: 'Difference Inputs2' (':5532:21')
     *
     * Block description for 'Difference Inputs2' (':5532:21'):
     *
     *  Add in CPU
     */
    Current_Ref_Estimator_DW.Delay_DSTATE_e = Current_Ref_Estimator_B.Merge[1];
  }

  /* Product: 'delta rise limit' (':5532:26') incorporates:
   *  Inport: 'Rate_limiter' (':5682')
   *  SampleTimeMath: 'sample time' (':5532:27')
   *
   * About 'sample time' (':5532:27'):
   *  y = K where K = ( w * Ts )
   *   */
  rtb_V_do = Current_Ref_Estimator_U.Rate_limiter.Iq_ramp_up * d.Op_ts;

  /* Sum: 'Difference Inputs1' (':5532:20') incorporates:
   *  Delay: 'Delay' (':5532:19')
   *
   * Block description for 'Difference Inputs1' (':5532:20'):
   *
   *  Add in CPU
   */
  rtb_Add_k = Current_Ref_Estimator_B.Merge[1] -
    Current_Ref_Estimator_DW.Delay_DSTATE_e;

  /* Switch: 'Switch2' (':5532:23:8') incorporates:
   *  RelationalOperator: 'LowerRelop1' (':5532:23:6')
   */
  if (!(rtb_Add_k > rtb_V_do)) {
    /* Product: 'delta fall limit' (':5532:25') incorporates:
     *  Inport: 'Rate_limiter' (':5682')
     *  SampleTimeMath: 'sample time' (':5532:27')
     *
     * About 'sample time' (':5532:27'):
     *  y = K where K = ( w * Ts )
     *   */
    rtb_V_do = d.Op_ts * Current_Ref_Estimator_U.Rate_limiter.Iq_ramp_down;

    /* Switch: 'Switch' (':5532:23:7') incorporates:
     *  RelationalOperator: 'UpperRelop' (':5532:23:9')
     */
    if (!(rtb_Add_k < rtb_V_do)) {
      rtb_V_do = rtb_Add_k;
    }

    /* End of Switch: 'Switch' (':5532:23:7') */
  }

  /* End of Switch: 'Switch2' (':5532:23:8') */

  /* Sum: 'Difference Inputs2' (':5532:21') incorporates:
   *  Delay: 'Delay' (':5532:19')
   *
   * Block description for 'Difference Inputs2' (':5532:21'):
   *
   *  Add in CPU
   */
  Current_Ref_Estimator_DW.Delay_DSTATE_e += rtb_V_do;

  /* Switch: 'Switch2' (':5534:8') incorporates:
   *  Inport: 'Id_Iq_MTPA_limit' (':5681')
   *  RelationalOperator: 'LowerRelop1' (':5534:6')
   *  RelationalOperator: 'UpperRelop' (':5534:9')
   *  Switch: 'Switch' (':5534:7')
   */
  if (Current_Ref_Estimator_DW.Delay_DSTATE_e >
      Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Iq_up_limit) {
    /* Switch: 'Switch2' (':5534:8') */
    Current_Ref_Estimator_Y.Iq_ref_final =
      Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Iq_up_limit;
  } else if (Current_Ref_Estimator_DW.Delay_DSTATE_e <
             Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Iq_low_limit) {
    /* Switch: 'Switch2' (':5534:8') incorporates:
     *  Switch: 'Switch' (':5534:7')
     */
    Current_Ref_Estimator_Y.Iq_ref_final =
      Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Iq_low_limit;
  } else {
    /* Switch: 'Switch2' (':5534:8') incorporates:
     *  Switch: 'Switch' (':5534:7')
     */
    Current_Ref_Estimator_Y.Iq_ref_final =
      Current_Ref_Estimator_DW.Delay_DSTATE_e;
  }

  /* End of Switch: 'Switch2' (':5534:8') */

  /* Outputs for Atomic SubSystem: 'Torque_calculation' (':5575') */
  /* Outport: 'Actual_torque_from_ref' (':5705') incorporates:
   *  Constant: 'Constant' (':5575:46')
   *  Inport: 'Motor_Parameters' (':5683')
   *  Outport: 'Lambda' (':5699')
   *  Outport: 'Ld' (':5695')
   *  Outport: 'Lq' (':5696')
   *  Product: 'Product' (':5575:47')
   *  Product: 'Product1' (':5575:48')
   *  Product: 'Product2' (':5575:49')
   *  Sum: 'Add' (':5575:45')
   *  Sum: 'Subtract' (':5575:50')
   */
  Current_Ref_Estimator_Y.Actual_torque_from_ref = (float)
    ((Current_Ref_Estimator_Y.Id_ref_final *
      Current_Ref_Estimator_Y.Iq_ref_final * (Current_Ref_Estimator_Y.Ld -
       Current_Ref_Estimator_Y.Lq) + Current_Ref_Estimator_Y.Iq_ref_final *
      Current_Ref_Estimator_Y.Lambda) *
     Current_Ref_Estimator_U.Motor_Parameters.Pole_Pairs * 1.5);

  /* End of Outputs for SubSystem: 'Torque_calculation' (':5575') */

  /* Outport: 'Lambdab' (':5704') incorporates:
   *  Outport: 'Lambda' (':5699')
   */
  Current_Ref_Estimator_Y.Lambdab = Current_Ref_Estimator_Y.Lambda;

  /* Outport: 'Lqb' (':5702') incorporates:
   *  Outport: 'Lq' (':5696')
   */
  Current_Ref_Estimator_Y.Lqb = Current_Ref_Estimator_Y.Lq;

  /* Outport: 'Pole_Pairs' (':5697') incorporates:
   *  Inport: 'Motor_Parameters' (':5683')
   */
  Current_Ref_Estimator_Y.Pole_Pairs =
    Current_Ref_Estimator_U.Motor_Parameters.Pole_Pairs;

  /* Outport: 'Rs' (':5698') incorporates:
   *  Inport: 'Motor_Parameters' (':5683')
   */
  Current_Ref_Estimator_Y.Rs = Current_Ref_Estimator_U.Motor_Parameters.Rs;

  /* Update for Delay: 'Delay1' (':5311') incorporates:
   *  Inport: 'Actual_torque_from_meas' (':5688')
   */
  Current_Ref_Estimator_DW.Delay1_DSTATE =
    Current_Ref_Estimator_U.Actual_torque_from_meas;

  /* Update for Delay: 'Delay' (':5531:19') */
  Current_Ref_Estimator_DW.icLoad = false;

  /* Update for Delay: 'Delay' (':5532:19') */
  Current_Ref_Estimator_DW.icLoad_b = false;
}

/* Model initialize function */
void Current_Ref_Estimator_initialize(void)
{
  /* Unit Delay initial conditions for the d/q voltage components. */
  Current_Ref_Estimator_DW.Vd_UnitDelay_DSTATE = 0.0F;
  Current_Ref_Estimator_DW.Vq_UnitDelay_DSTATE = 0.0F;

#if FW_CLOSED_LOOP
  fw_cl_dId = 0.0F;
  fw_cl_iq_cap = FW_CL_IQ_CAP_OFF;
#endif

  /* InitializeConditions for Delay: 'Delay' (':5531:19') */
  Current_Ref_Estimator_DW.icLoad = true;

  /* InitializeConditions for Delay: 'Delay' (':5532:19') */
  Current_Ref_Estimator_DW.icLoad_b = true;

  /* SystemInitialize for Atomic SubSystem: 'Motor_Parameter_Degrade1' (':5262') */
  Cu_Motor_Parameter_Degrade_Init
    (&Current_Ref_Estimator_DW.Motor_Parameter_Degrade1);

  /* End of SystemInitialize for SubSystem: 'Motor_Parameter_Degrade1' (':5262') */

  /* SystemInitialize for Atomic SubSystem: 'Motor_Parameter_Degrade2' (':5376') */
  Cu_Motor_Parameter_Degrade_Init
    (&Current_Ref_Estimator_DW.Motor_Parameter_Degrade2);

  /* End of SystemInitialize for SubSystem: 'Motor_Parameter_Degrade2' (':5376') */

  /* SystemInitialize for Atomic SubSystem: 'Motor_Parameter_Degrade' (':5261') */
  Cu_Motor_Parameter_Degrade_Init
    (&Current_Ref_Estimator_DW.Motor_Parameter_Degrade);

  /* End of SystemInitialize for SubSystem: 'Motor_Parameter_Degrade' (':5261') */

  /* SystemInitialize for IfAction SubSystem: 'If Action Subsystem1' (':5425') */
  /* InitializeConditions for DiscreteIntegrator: 'Discrete-Time Integrator' (':5449') */
  Current_Ref_Estimator_DW.DiscreteTimeIntegrator_DSTATE = 1.0F;
  Current_Ref_Estimator_DW.DiscreteTimeIntegrator_PrevRese = 2;

  Current_Ref_Estimator_U.FWC_values.M2F_factor = can_d.canFW_params.M_to_F;
  Current_Ref_Estimator_U.FWC_values.F2M_factor = can_d.canFW_params.F_to_M;
  Current_Ref_Estimator_U.FWC_values.FW_Int_reset_factor = can_d.canFW_params.Rst_Factor;
  Current_Ref_Estimator_U.FWC_values.Kfw = can_d.canFW_params.Kfw;
  Current_Ref_Estimator_U.FWC_values.Kaw = can_d.canFW_params.Kaw;

  Current_Ref_Estimator_U.BusVoltage_V = OP_VOLTAGE;
  Current_Ref_Estimator_U.Is_max = MOTOR_PEAK_AC;
  Current_Ref_Estimator_U.Motor_Parameters.Pole_Pairs = POLEPAIRS;
  Current_Ref_Estimator_U.Motor_Parameters.Rs = can_d.can_Rs;
  Current_Ref_Estimator_U.Motor_Parameters.Ld = can_d.can_Ld;
  Current_Ref_Estimator_U.Motor_Parameters.Lq = can_d.can_Lq;
  Current_Ref_Estimator_U.Motor_Parameters.Lambda = can_d.can_Lambda;
  Current_Ref_Estimator_U.Motor_Parameters.Motor_P_lim_Watts = MIN_POWER;
  Current_Ref_Estimator_U.Motor_Parameters.Motor_P_Max_Watts = MAX_POWER;
  Current_Ref_Estimator_U.Motor_Parameters.Ld_Percentage = LD_PERCENTAGE_CHANGE;
  Current_Ref_Estimator_U.Motor_Parameters.Lq_Percentage = LQ_PERCENTAGE_CHANGE;
  Current_Ref_Estimator_U.Motor_Parameters.Flux_Percentage = FLUX_PERCENTAGE_CHANGE;

  /* HAND-EDIT (init parameterisation) — NOT IN THE SIMULINK MODEL.
   * Id/Iq reference clamp + measured-OC fault threshold (peak A). Sized off
   * MOTOR_PEAK_AC via IDQ_OC_MARGIN_PCT in the active motor profile so each
   * motor gets a correctly-scaled backstop; was a shared hard-coded 560 A that
   * sat above even the board phase-OC fault. See motor_select.h / the profile. */
  Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Iq_up_limit  = IQ_UP_LIMIT;
  Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Iq_low_limit = IQ_LOW_LIMIT;
  Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Id_up_limit  = ID_UP_LIMIT;
  Current_Ref_Estimator_U.Id_Iq_MTPA_limit.Id_low_limit = ID_LOW_LIMIT;

  Current_Ref_Estimator_U.Rate_limiter.Ref_Speed_rate_up = 100.0f * 10000.0f;
  Current_Ref_Estimator_U.Rate_limiter.Ref_Speed_rate_down = -500.0f * 10000.0f;
  Current_Ref_Estimator_U.Rate_limiter.Ref_Speed_rate_down_neutral = -600.0f * 10000.0f;

  /* HAND-EDIT (init parameterisation) — NOT IN THE SIMULINK MODEL.
   * Nm/s — kept in lockstep with FOC_Loop.c via the motor-profile macros. */
  Current_Ref_Estimator_U.Rate_limiter.Torque_gen_ramp_up = FOC_TRQ_RAMP_UP_NM_S;
  Current_Ref_Estimator_U.Rate_limiter.Torque_gen_ramp_down = -FOC_TRQ_RAMP_DOWN_NM_S;
  Current_Ref_Estimator_U.Rate_limiter.Id_ramp_up = 50.0f * 10000.0f;
  Current_Ref_Estimator_U.Rate_limiter.Id_ramp_down = -50.0f * 10000.0f;
  Current_Ref_Estimator_U.Rate_limiter.Iq_ramp_up = 50.0f * 10000.0f;
  Current_Ref_Estimator_U.Rate_limiter.Iq_ramp_down = -50.0f * 10000.0f;

  /* End of SystemInitialize for SubSystem: 'If Action Subsystem1' (':5425') */
}

/* Model terminate function */
void Current_Ref_Estimator_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
