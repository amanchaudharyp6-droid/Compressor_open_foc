#include "Helper_Functions.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern ADC_HandleTypeDef hadc2;

void rt_OneStep(void)
{
  static bool OverrunFlags[2] = { 0, 0 };

  static bool eventFlags[2] = { 0, 0 };/* Model has 2 rates */

  static int taskCounter[2] = { 0, 0 };

  /* Disable interrupts here */

  /* Check base rate for overrun */
  if (OverrunFlags[0]) {
    rtmSetErrorStatus(FOC_Loop_M, "Overrun");
    return;
  }

  OverrunFlags[0] = true;

  /* Save FPU context here (if necessary) */
  /* Re-enable timer or interrupt here */

  /*
   * For a bare-board target (i.e., no operating system), the
   * following code checks whether any subrate overruns,
   * and also sets the rates that need to run this time step.
   */
  if (taskCounter[1] == 0) {
    if (eventFlags[1]) {
      OverrunFlags[0] = false;
      OverrunFlags[1] = true;

      /* Sampling too fast */
      rtmSetErrorStatus(FOC_Loop_M, "Overrun");
      return;
    }

    eventFlags[1] = true;
  }

  taskCounter[1]++;
  if (taskCounter[1] == 10) {
    taskCounter[1]= 0;
  }

  /* Set model inputs associated with base rate here */

  /* Step the model for base rate */
  // FOC_MTPA_FWC_FF_step0();
  Current_Ref_Estimator_U.Actual_torque_from_meas = FOC_Loop_Y.Actual_torque_from_meas;
  Current_Ref_Estimator_U.T_ref_final = FOC_Loop_Y.T_ref_FNR;
  Current_Ref_Estimator_step();
  FOC_Loop_U.Id_ref_final = Current_Ref_Estimator_Y.Id_ref_final;
  FOC_Loop_U.Iq_ref_final = Current_Ref_Estimator_Y.Iq_ref_final;
  FOC_Loop_U.Ld = Current_Ref_Estimator_Y.Ld;
  FOC_Loop_U.Lq = Current_Ref_Estimator_Y.Lq;
  FOC_Loop_U.Lambda = Current_Ref_Estimator_Y.Lambda;
  FOC_Loop_U.Rs = Current_Ref_Estimator_Y.Rs;
  FOC_Loop_step0();

  /* Get model outputs here */
  d.regen_flag = FOC_Loop_Y.Regen_Mode;   /* consumed by Send_on_CAN_706 */

  /* Indicate task for base rate complete */
  OverrunFlags[0] = false;

  /* If task 1 is running, do not run any lower priority task */
  if (OverrunFlags[1]) {
    return;
  }

  /* Step the model for subrate */
  if (eventFlags[1]) {
    OverrunFlags[1] = true;

    /* Set model inputs associated with subrates here */
    /* Gathering speed feedback data and setting motor start flag */
    Speed_Sense(d.mech_angle);
    d.omega_e = d.rad_s * POLEPAIRS;
    Current_Ref_Estimator_U.Actual_Speed_mech = d.rad_s;
    FOC_Loop_U.ActualSpeed_mech_radsec = d.rad_s;
    if (fabsf(d.RPM) >= MIN_RPM_FOR_MOTOR_START)
    {
      d.motor_start = 1;
    }

    /* Step the model for subrate 1 */
    // FOC_MTPA_FWC_FF_step1();
    FOC_Loop_step1();

    /* Get model outputs here */

    /* Indicate task complete for subrate */
    OverrunFlags[1] = false;
    eventFlags[1] = false;
  }

  /* Disable interrupts here */
  /* Restore FPU context here (if necessary) */
  /* Enable interrupts here */
}

void set_Initial_angle(void)
{
  /* Calculating the startup angle and converting to TIM2 counts */
  #if MOTOR_PROFILE == MOTOR_PROFILE_LIVGUARD
  d.Angle_From_Duty = (d.Duty) * 0.01f * TWO_PI - HIGH_PULSE16_ERROR;
  if (d.Angle_From_Duty < 0.0f)
  {
    d.Angle_From_Duty += TWO_PI;
  }
  #endif
  /* HiRange TR shares the default motor + resolver, so it uses the same
   * (100 - Duty) startup-angle convention. This matches the runtime absolute-
   * angle path in main.c (d.abs_Angle_pwm), which also uses (100 - Duty)
   * unconditionally. Without this, TR matched neither branch and started with
   * Angle_From_Duty = 0 (wrong commutation). Confirm during TR alignment. */
  #if (MOTOR_PROFILE == MOTOR_PROFILE_DEFAULT) || (MOTOR_PROFILE == MOTOR_PROFILE_HIRANGE_TR)
  d.Angle_From_Duty = (100.0f - d.Duty) * 0.01f * TWO_PI - HIGH_PULSE16_ERROR;
  if (d.Angle_From_Duty < 0.0f)
  {
    d.Angle_From_Duty += TWO_PI;
  }
  #endif
  
  d.Angle_From_Duty = fmodf(d.Angle_From_Duty, TWO_PI);
  d.Count_From_Duty = (uint32_t)((d.Angle_From_Duty / TWO_PI) * (TIM2_ARR + 1));

  #if ENABLE_ENC_ERRORS
  /* Checking for PWM disconnection error */
  if (encoder_pwm_error_check() == !HAL_OK)
  {
    er.error_triggered = 1;
    er.pwm_error = 1;
    err = ENCODER_PWM_ERROR;
    cS = CONT_ERROR;

    disable_drive();

    return;
  }
  #endif

  HAL_Delay(10);

  HAL_GPIO_WritePin(DRIVE_INHIBIT_GPIO_Port, DRIVE_INHIBIT_Pin, GPIO_PIN_SET);
  cS = CURR_SENS_CALIB;
}

void Disable_tim5(void)
{
  HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_1);
  HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_2);
  HAL_TIM_IC_DeInit(&htim5);
  HAL_TIM_Base_MspDeInit(&htim5);
}

void power_mode_fnr_switch(void)
{
  #if THROTTLE_BASED_DIR
  d.forward_pin = HAL_GPIO_ReadPin(FNR_FWD_GPIO_Port, FNR_FWD_Pin);
  d.reverse_pin = HAL_GPIO_ReadPin(FNR_REV_GPIO_Port, FNR_REV_Pin);
  d.power_pin = HAL_GPIO_ReadPin(PWR_MODE_GPIO_Port, PWR_MODE_Pin);
  #endif

  #if THROTTLE_BASED_DIR
  /* Decode the FNR switch position into a requested state. Both pins high =
   * centre/NEUTRAL; both low (undefined / contact bridge mid-transition) is
   * also treated as NEUTRAL so the safe outcome is always to coast. */
  fnr_state_t fnr_requested;
  if (d.forward_pin == GPIO_PIN_RESET && d.reverse_pin == GPIO_PIN_SET)
    fnr_requested = FORWARD;
  else if (d.forward_pin == GPIO_PIN_SET && d.reverse_pin == GPIO_PIN_RESET)
    fnr_requested = REVERSE;
  else
    fnr_requested = NEUTRAL;   /* both high (centre) or both low (undefined) */

  /* Direction-change interlock (anti-plugging):
   *  - NEUTRAL is honoured immediately (coast / regen to stop).
   *  - A driving direction that is already active stays engaged.
   *  - Any change to the OPPOSITE driving direction (or first engagement out
   *    of NEUTRAL) only latches once the motor is essentially stopped AND the
   *    throttle is released. Until both hold, we force NEUTRAL so the motor
   *    coasts down instead of holding the old direction or plugging. This
   *    makes F->R and R->F behave identically regardless of how the switch
   *    transits centre or whether the throttle is held. */
  if (fnr_requested == NEUTRAL)
  {
    fnr_state = NEUTRAL;
  }
  else if (fnr_requested != fnr_state)
  {
    if (fabsf(d.RPM) <= MIN_RPM_CHANGE_LIM && d.throttle_v <= (THR_MIN_VAL + 0.1f))
      fnr_state = fnr_requested;   /* stopped + throttle released -> commit */
    else
      fnr_state = NEUTRAL;         /* force coast-down until safe */
  }
  /* else: requested == current driving direction -> keep it engaged */

  /* Map the latched fnr_state to the model's electrical Drive_State.
   * MOTOR_DIRECTION == DIR_FLIP swaps FORWARD/REVERSE at the model boundary. */
  if (fnr_state == NEUTRAL)
  {
    FOC_Loop_U.Drive_State = NEUTRAL;
  }
  else
  {
    #if MOTOR_DIRECTION == DIR_FLIP
      FOC_Loop_U.Drive_State = (fnr_state == FORWARD) ? REVERSE : FORWARD;
    #else
      FOC_Loop_U.Drive_State = (fnr_state == FORWARD) ? FORWARD : REVERSE;
    #endif
  }
  #endif

  #if CAN_BASED_DIR
  if (can_d.direction == FORWARD && fabsf(d.RPM) <= MIN_RPM_CHANGE_LIM && can_d.can_instV <= (THR_MIN_VAL + 0.1f) && FOC_Loop_U.Ref_Speed_mech_rpm <= 0.0f) {
    fnr_state = FORWARD;
    FOC_Loop_U.Drive_State = FORWARD;
  } else if (can_d.direction == REVERSE && fabsf(d.RPM) <= MIN_RPM_CHANGE_LIM && can_d.can_instV <= (THR_MIN_VAL + 0.1f) && FOC_Loop_U.Ref_Speed_mech_rpm <= 0.0f) {
    fnr_state = REVERSE;
    FOC_Loop_U.Drive_State = REVERSE;
  } else if (can_d.direction == NEUTRAL) {
    d.speed_ref = 0.0f;
    fnr_state = NEUTRAL;
    FOC_Loop_U.Drive_State = NEUTRAL;
  }

  if (can_d.power_mode == SPORTS) {
    pw_state = SPORTS;
  } else if (can_d.power_mode == ECO) {
    pw_state = SPORTS;
  }
  #endif

  #if THROTTLE_BASED_REF
  ThrottleMap_SetDirection(&g_throttle_cfg, fnr_state);
  #endif

  #if THROTTLE_BASED_DIR
  if (d.power_pin == GPIO_PIN_RESET) {
    pw_state = SPORTS;
  } else {
    pw_state = ECO;
  }
  #endif

  #if THROTTLE_BASED_REF
  ThrottleMap_SetDriveMode(&g_throttle_cfg, pw_state);
  #endif

  #if CAN_BASED_REF
  if (pw_state == ECO && fnr_state == FORWARD) {
    if (d.speed_ref >= ECO_MAX_SPEED) {
      d.speed_ref = ECO_MAX_SPEED;
    }
  } else if (pw_state == SPORTS && fnr_state == FORWARD) {
    if (d.speed_ref >= SPORTS_MAX_SPEED) {
      d.speed_ref = SPORTS_MAX_SPEED;
    }
  }

  if (fnr_state == REVERSE) {
    if (d.speed_ref >= REV_MAX_SPEED) {
      d.speed_ref = REV_MAX_SPEED;
    }
  }
  #endif
}

void disable_drive(void) 
{
  #if ENABLE_FAULTS
  if (cS == CONT_ERROR)
  {
    /* Checking is error is triggered and shutting off gate drivers and TIM1 */
    if (er.error_triggered)
    {
      er.drive_off = 1;

      HAL_GPIO_WritePin(DRIVE_INHIBIT_GPIO_Port, DRIVE_INHIBIT_Pin, GPIO_PIN_SET);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TIM1_ARR_HALF);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, TIM1_ARR_HALF);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, TIM1_ARR_HALF);
      __HAL_TIM_MOE_DISABLE(&htim1);
      HAL_ADCEx_InjectedStop_IT(&hadc2);  /* halt the prio-0 FOC ISR: CONT_ERROR is terminal (reboot-only recovery re-inits ADC at main.c) */
    }

    /* Resetting FOC data structures after error trigger */
    (void)memset(&FOC_Loop_U, 0, sizeof(ExtU_FOC_Loop_T));
    (void)memset(&FOC_Loop_Y, 0, sizeof(ExtY_FOC_Loop_T));
    // (void)memset(&d, 0, sizeof(data_t));
  }
  #endif
}