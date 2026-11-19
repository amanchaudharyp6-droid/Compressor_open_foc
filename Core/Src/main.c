/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "spi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
image_hdr_t image_hdr __attribute__((section(".image_hdr"))) = {
   .image_magic       = IMAGE_MAGIC,
   .image_hdr_version = IMAGE_HEADER_VERSION,
   .crc               = 0,
   .data_size         = 0,
   .image_type        = 0,
   .version_major     = FIRMWARE_VERSION_MAJOR,
   .version_minor     = FIRMWARE_VERSION_MINOR,
   .version_patch     = FIRMWARE_VERSION_SUBMINOR,
#ifdef CURR_APP1
    .active_app = 1,
#endif
#ifdef CURR_APP2
    .active_app = 2,
#endif
    .git_sha = (uint64_t)GIT_SHA,
};

uint8_t UID[12] = {0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
void OTA_Manager(void);   /* defined below; forward-declared so the call in the main loop has a prototype */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t heart_beat_init[8] = {0x01, 0x04};
uint8_t rxMsg[8] = {0};
uint16_t injectedVal[3] = {0};          /* [U, V, W] */
uint16_t currSensOff[3] = {0, 0, 0};
/* ADC1 DMA target — written by hardware, not by any CPU context. volatile is
 * load-bearing here: the build uses -flto, so without it the definition and
 * its reader (ADC1_Analog_Val_Update) are visible to one optimiser that
 * cannot see the DMA writes. Keep the qualifier on the extern too. */
volatile uint16_t adc1_buffer[5] = {0};
uint16_t counter = 0;
uint32_t ia_offset = 0;
uint32_t ib_offset = 0;

/* Autotune stuct instances */
tag_AngleAutoTuneDebugParam TagAutotuneDebug;
tag_AngleAutoTuneOutputParam TagAutotuneOutput;
tag_AngleAutoTuneInputParam TagAutotuneInput;
tag_AutotuneConfig AutotuneConfig;

uint8_t OTA_State = 0;

volatile fnr_state_t fnr_state = NEUTRAL;
volatile power_mode_t pw_state = ECO;
volatile currSession cS = INIT;
errors_nums err = NO_ERROR;
errors er = {0};
can_data_t can_d = {.can_soc = 1.0f, .can_pack_voltage = OP_VOLTAGE, .isense_delay_s = FOC_ISENSE_DELAY_S, .can_dclo = DCLO_DEFAULT, .can_Rs = RS, .can_Lambda = LAMBDA, .can_Ld = LD, .can_Lq = LQ, .canFW_params.M_to_F = FW_M2F, .canFW_params.F_to_M = FW_F2M, .canFW_params.Kaw = FW_KAW, .canFW_params.Kfw = FW_KFW, .canFW_params.Rst_Factor = FW_RST_FACTOR, .canSpeed_PID.Kp = SPEED_KP, .canSpeed_PID.Ki = SPEED_KI, .canSpeed_PID.Kd = SPEED_KD, .canSpeed_PID.Back_Kaw = SPEED_PID_BACK_KAW, .canSpeed_PID.Kd_Filter = SPEED_KD_FILTER, .canSpeed_PID.Output_Up_Limit = SPEED_PID_OUT_UPL, .canSpeed_PID.Output_Low_Limit = SPEED_PID_OUT_LOWL, .canId_PID.Kp = ID_KP, .canId_PID.Ki = ID_KI, .canId_PID.Kd = ID_KD, .canId_PID.Back_Kaw = ID_PID_BACK_KAW, .canId_PID.Kd_Filter = ID_KD_FILTER, .canId_PID.Output_Up_Limit = ID_PID_OUT_UPL, .canId_PID.Output_Low_Limit = ID_PID_OUT_LOWL, .canIq_PID.Kp = IQ_KP, .canIq_PID.Ki = IQ_KI, .canIq_PID.Kd = IQ_KD, .canIq_PID.Back_Kaw = IQ_PID_BACK_KAW, .canIq_PID.Kd_Filter = IQ_KD_FILTER, .canIq_PID.Output_Up_Limit = IQ_PID_OUT_UPL, .canIq_PID.Output_Low_Limit = IQ_PID_OUT_LOWL};
data_t d = {.canBus_ok = 1, .regen_active = false, .Op_ts = 0.0001f, .speed_ref = 0.0f, .start_alignment = 1, .end_alignment = 0, .Vmax_SVM = SVM_VOLTAGE_LIMIT, .Vdc = OP_VOLTAGE, .vdc_filt = OP_VOLTAGE};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  SCB_EnableICache();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  #if EST_CYC_CNT
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR    = 0xC5ACCE55;  /* unlock DWT regs (required on Cortex-M7) */
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
  #endif
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_TIM5_Init();
  MX_TIM17_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_FDCAN2_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(10);

  /* Stopping all TIM1 for safety precauiton at program start */
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);

  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Get_UID();
  /* First read EEPROM for calib done flag */
  Read_EEPROM_at_init();
  HAL_Delay(10);

  if (cS == INIT || FORCE_ANGLETUNE) {
    set_Initial_angle();
    /* Setting Auto tune algo input params */
    AutotuneConfig.f_idSetRef = AUTO_TUNING_IDREF;
    AutotuneConfig.f_IdRefIncrementValue = AUTOTUNE_IDREF_INCREMENT;
    AutotuneConfig.f_CalibElectricalAngleInc = AUTOTUNE_ANGLE_INCREMENT;
    AutotuneConfig.F_MaxVd = MAX_VD;
    AutotuneConfig.F_MinVd = MIN_VD;
    AutotuneConfig.f_Electrical360Rad = ANGLE_RAD_360;
    AutotuneConfig.u8_MOTOR_POLE_PAIRS = MOTOR_POLE_PAIRS;
    AutotuneConfig.u16_AO_Set_Time = AO_SET_TIME;
    AutotuneConfig.u16FreeRunTmo = AUTOTUNE_FREERUNCOUNT;
    AutotuneConfig.f_Autotune_RPM_Set_Ref = TUNING_CALC_RPM;
    AutotuneConfig.u16StateChangeTMO = AUTOTUNESTATE_CHANGE_TMO;
    AutotuneConfig.u16_AUTOTUN_SPEED_RAMP_TIME = AUTOTUN_SPEED_RAMP_TIME;
    AutotuneConfig.u16_AUTOTUNE_SPEED_RAMP = AUTOTUNE_SPEED_RAMP;
    AutotuneConfig.u16_MAX_ITERATION = MAX_ITERATION;
    AutotuneConfig.u16_ALIGNEMENT_TMO = ALIGNEMENT_TMO;
    AutotuneConfig.u16_AVERAGE_SAMPLE = AVERAGE_SAMPLE;
    AutotuneConfig.u16_VD_VQ_AVERAGE_COUNT = VD_VQ_AVERAGE_COUNT;
    d.regen_active = false;
    AutotuneInit(&AutotuneConfig);
  } else if (cS == ANGLE_CALIB_DONE || !FORCE_ANGLETUNE) {
    set_Initial_angle();
    Disable_tim5();
    d.regen_active = true;
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  // Start encoder capture on both channels with interrupts
  if (HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
    Error_Handler();

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

  HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_Delay(10);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buffer, 5);

  /* Initializing FOC and Protections model */
  // FOC_MTPA_FWC_FF_initialize();
  Current_Ref_Estimator_initialize();
  FOC_Loop_initialize();
  #if CAN_SPEED_ONLY
  /* Hold Forward + minimum throttle voltage so the model's FNR chart enters
   * Forward at standstill (needs FNR_Active = throttle <= min + hyst, and
   * Drive_State = Forward). Regen off: with throttle at minimum, Forward and
   * speed above REGEN_MIN_RPM_EN the chart would otherwise enter regen. */
  can_d.direction = FORWARD;
  can_d.can_instV = THR_MIN_VAL;
  FOC_Loop_U.Throttle_input.Throttle_Inst_Voltage = THR_MIN_VAL;
  FOC_Loop_U.Regen.Regen_Active = false;
  d.regen_active = false;
  #endif
  Open_FOC0_initialize();
  #if PROTECTION_MODEL
  MCU_Protections_initialize();
  #endif
  RateLimiter_Init(&limiter, 100.0f, 600.0f, 0.0f);
  /* Throttle-request limiter — the launch-feel knob (TRQ_REQ_SLEW_* in
   * vehicle_config.h). Binding for driver transients; the command slew in
   * Torque_Management is faster and only smooths protection releases. */
  RateLimiter_Init(&torqueRate, TRQ_REQ_SLEW_UP_NM_S, TRQ_REQ_SLEW_DOWN_NM_S, 0.0f);
  HAL_Delay(10);

  /*Starting FDCAN2, Queue Init and Throttle Init*/
  FDCAN_SETUP();
  CAN_Queue_Init();
  #if THROTTLE_BASED_REF
  App_Init();
  #endif

  /*Sending Heartbeat message once at init*/
  _fdcan_transmit_on_can(0x400, FDCAN_STANDARD_ID, heart_beat_init, 0x08);
  Send_heartbeat();
  /* Enable LED */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_PINS, GPIO_PIN_SET);

  /*Starting ADC2 Injected conversions first*/
  HAL_ADCEx_InjectedStart(&hadc1);      /* V current (PF11), polled by ADC2 callback */
  HAL_ADCEx_InjectedStart_IT(&hadc2);
  HAL_TIM_Base_Start_IT(&htim17);

  /*Then starting TIM1 complementary channels and trigger channel*/
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  while (1)
  {
    uint32_t Time_Now = HAL_GetTick();
    static uint32_t Time_prev_25ms = 0; 
    if(Time_Now - Time_prev_25ms >= 25){
      Time_prev_25ms = Time_Now; 
      if(OTA_State){
        OTA_Manager();
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t time_stamp_now = HAL_GetTick();

    /* P4: deferred EEPROM write (ISR set flag to avoid blocking 1kHz TIM17) */
    if (eeprom_write_pending && cS == ANGLE_CALIB_DONE)
    {
      eeprom_write_pending = 0;
      if (EEPROM_Write_Page0(eeprom_cw_pending, eeprom_ccw_pending, eeprom_cw_high_pending) == HAL_OK)
        HAL_NVIC_SystemReset();
      else
      {
        er.eeprom_write_error = 1;
        err = EEPROM_WRITE_ERROR;
      }
    }

    /* CAN-triggered autotune: invalidate EEPROM page 0 and reset so next
     * boot runs ANGLE_CALIB via the existing start_alignment/end_alignment path. */
    if (can_d.autotune_request && fabsf(d.RPM) <= 10.0f)
    {
      disable_drive();
      if (EEPROM_Invalidate_Page0() == HAL_OK)
        HAL_NVIC_SystemReset();
      else
      {
        can_d.autotune_request = 0;
        er.eeprom_write_error = 1;
        err = EEPROM_WRITE_ERROR;
      }
    }

    if (cS == FOC_START)
      power_mode_fnr_switch();

    /* Gathering ADC1 values */
    ADC1_Analog_Val_Update();
    #if PROTECTION_MODEL
    /* MCU Proctections Model input update */
    Model_Params_Input();
    /* Running MCU protections model */
    MCU_Protections_step();
    /* Checking the MCU Protections Model output flags and setting error flags */
    Model_Output_Flag_Checks();
    #endif

    #if CAN_BASED_REF
    if (cS == FOC_START && FOC_Loop_U.Speed_1_Torque_0 == true)
      FOC_Loop_U.Ref_Speed_mech_rpm = RateLimiter_Update(&limiter, d.speed_ref, ((float)time_stamp_now * T_S));
    else if (cS == FOC_START && FOC_Loop_U.Speed_1_Torque_0 == false) {
      FOC_Loop_U.RefTrq = RateLimiter_Update(&torqueRate, d.torque_ref, ((float)time_stamp_now * T_S));
    }
    #endif

    #if THROTTLE_BASED_REF
    if (FOC_Loop_U.Speed_1_Torque_0 == true && cS == FOC_START) {
      d.speed_ref = (float)ThrottleMap_GetRPM(&g_throttle_cfg, d.thr_v_mv);
      /* Pass target_rpm into your FOC speed reference */
      FOC_Loop_U.Ref_Speed_mech_rpm = RateLimiter_Update(&limiter, d.speed_ref, ((float)time_stamp_now * T_S));
    } else if (FOC_Loop_U.Speed_1_Torque_0 == false && cS == FOC_START) {
      /* Request limiter runs every iteration — including brake-command
       * cuts (target 0) — so its state tracks reality and never goes
       * stale. The cut itself stays immediate: apply_torque_scaling gets
       * a raw 0, not the limited value. */
      float torque_req = can_d.brake_command_active ? 0.0f
                         : ThrottleMap_GetTorque(&g_throttle_cfg, d.thr_v_mv, MAX_TORQUE);
      torque_req = RateLimiter_Update(&torqueRate, torque_req, ((float)time_stamp_now * T_S));
      if (can_d.brake_command_active) {
        FOC_Loop_U.RefTrq = d.torque_final = apply_torque_scaling(torque_req, d.rad_s, pw_state, fnr_state, (float)d.Mtr_temp, (float)d.Mtc_temp, &derate_result);
        can_d.brake_command_reply = true;
      } else {
        d.torque_final = apply_torque_scaling(torque_req, d.rad_s, pw_state, fnr_state, (float)d.Mtr_temp, (float)d.Mtc_temp, &derate_result);
        /* Pass target_reftrq into your FOC Torque reference */
        FOC_Loop_U.RefTrq = d.torque_final;
        can_d.brake_command_reply = false;
      }
      /* Regen enable is refreshed every torque-mode loop so it never lingers at
       * a stale init value (torque mode is the production mode; in speed/dyno
       * mode regen arming falls back to the init default d.regen_active). */
      #if REGEN_LESS_EAGER_ARM
      /* Less-eager regen arming (see vehicle_config.h): permit the FOC chart to
       * arm regen only while the driver is NOT commanding drive torque, so a
       * slight lift coasts instead of grabbing regen. Gates ARMING only — the
       * regen exit is handled polarity-correctly inside FOC_Loop.c, and
       * clearing this mid-regen does not force an exit (the else/stay branch
       * ignores Regen_Active).
       *
       * Gate on torque_req — the RAW driver demand — not d.torque_final. The
       * latter is the post-limiter command, so when the speed/power cap cuts it
       * to ~0 at FULL throttle (switching SPORTS->ECO above ECO's speed cap, or
       * any speed-limited crest) torque_final falls below the arm threshold and
       * regen engages WHILE the driver is still asking for thrust — a
       * thrust-to-brake reversal, felt as a violent jerk. torque_req stays high
       * until a genuine lift, so regen only arms when the driver actually backs
       * off. */
      FOC_Loop_U.Regen.Regen_Active =
          d.regen_active && (torque_req <= REGEN_ARM_TORQUE_NM);
      #else
      FOC_Loop_U.Regen.Regen_Active = d.regen_active;
      #endif
    }
    #endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 9;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 3072;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC2)
  {
    /* Reading injected values for the 2 phases */
    /* U = PB1 (ADC2 rank1), W = PB0 (ADC2 rank2), V = PF11 (ADC1-only pin,
     * ADC1 injected, same TIM1 TRGO2 trigger; finishes before ADC2's 2nd conv) */
    injectedVal[PHASE_U] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    injectedVal[PHASE_W] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    injectedVal[PHASE_V] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);

    if (cS == CURR_SENS_CALIB)
      isr_curr_sens_calib();
    else if (cS == ANGLE_CALIB)
      isr_angle_calib();
    else if (cS == FOC_START)
      isr_open_foc();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
  {
    HAL_TIM_Base_Stop_IT(&htim4);
    #if VH_CAN_ID
    Send_on_CAN_7A1();
    Send_on_CAN_7A2();
    Send_on_CAN_7A3();
    #endif
  }

  if (htim->Instance == TIM17)
    tim17_1ms_tick();
}

/* FDCAN Rx Callback — dispatch logic lives in can_rx_handlers.c */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxMessageBuf, rxMsg);

  #if DEBUG_CAN_ID
  if (RxMessageBuf.DataLength >= FDCAN_DLC_BYTES_1)
    can_rx_dispatch(RxMessageBuf.Identifier, rxMsg);
  #endif
  #if VH_CAN_ID
  #if CAN_AUTOTUNE_TRIGGER
  /* Bench-only: another ECU transmits on 0x729 on the vehicle bus (see
   * vehicle_config.h). The FDCAN filter for 0x729 is gated the same way. */
  if (RxMessageBuf.Identifier == 0x729) {
    can_rx_0x729((uint8_t*)rxMsg);
  }
  #endif

  if (RxMessageBuf.Identifier == 0x110) {
    int32_t temp_curr = (int32_t)((rxMsg[7] << 24) | (rxMsg[6] << 16) | (rxMsg[5] << 8) | rxMsg[4]);
    can_d.can_battCurr = (float)(temp_curr) * 1.0E-5;
    can_d.t_rx_0x110_ms = HAL_GetTick();   /* freshness + new-sample detect for DCLO */
  }

  if (RxMessageBuf.Identifier == 0x12A) {
    can_d.can_dclo = (float)(rxMsg[3] << 8 | rxMsg[2]) * 0.1f;
    can_d.can_dcli = (float)(rxMsg[1] << 8 | rxMsg[0]) * 0.1f;
    can_d.t_rx_0x12A_ms = HAL_GetTick();
  }

  if (RxMessageBuf.Identifier == 0x109) {
    /* BMS CAN matrix 0x109: bytes 0-1 SOC (0.01%), bytes 2-3 SOH (0.01%),
     * byte 4 BMSS, byte 5 FFC, bytes 6-7 PV pack voltage (0.1 V).
     * SOC: raw * 0.0001 -> per-unit [0.0, 1.0]. */
    can_d.can_soc = (float)((rxMsg[1] << 8) | rxMsg[0]) * 0.0001f;
    /* Pack voltage: bytes 7 (high) | 6 (low), scale 0.1 -> volts.
     * (Was mis-decoded from bytes 2-3 = SOH: 100% SOH read as "1000 V",
     * which railed the DCLO modulation-index estimate at its floor and
     * loosened the Is cap ~2.4x -> DCLO breach.) Only accept when the
     * frame carries all 8 bytes AND the value is plausible; else keep the
     * last good / OP_VOLTAGE boot default. rxMsg is a shared buffer (HAL
     * writes only DataLength bytes), so a short frame leaves STALE bytes
     * at [6..7], and a zero decode would zero the DCLO Is cap. */
    if (RxMessageBuf.DataLength >= FDCAN_DLC_BYTES_8) {
      float pack_v = (float)((rxMsg[7] << 8) | rxMsg[6]) * 0.1f;
      if (pack_v > 1.0f) can_d.can_pack_voltage = pack_v;
    }
  }

  if (RxMessageBuf.Identifier == 0x725) {
    if (rxMsg[0] == 0x01) {
      can_d.brake_command_active = true;
    } else if (rxMsg[0] == 0x00) {
      can_d.brake_command_active = false;
    }
  }
  #endif
  if(RxMessageBuf.Identifier == 0x6FA)
  {
    if((rxMsg[1] == PEGASUS_ID) && (rxMsg[2] == FUPFG_REQUEST)){
      OTA_Source_ECU = rxMsg[0];
      OTA_State = 1; //Firing Up OTA Manager
      if(rxMsg[3] == RECEIVE_BIN){
        uptype = FOTA;
        upgrade_state = UPGRADE_INIT;
      }
      else if(rxMsg[3] == TRANSFER_COMPLETE){
        upgrade_state = UPGRADE_WAIT;
        transmit_upgrade_Status(SUCCESS_MESS);
      }
      else if(rxMsg[3] == PERFORM_UPGRADE || rxMsg[3] == COMPLETE_OTA){
        uptype = FOTA;
        upgrade_state = UPGRADE_COMPLETE;
        set_complete_flag(1);
        transmit_upgrade_Status(SUCCESS_MESS);
        // switch_partition_and_reset();
      }
      else if(rxMsg[3] == RECEIVE_CONF){
        uptype = COTA;
        upgrade_state = UPGRADE_INIT;
        //COTA TBD :(
      }
    }
  }
  if(RxMessageBuf.Identifier == 0x7FE) //OTA Data Message
  {
    CAN_TP_Receive_interrupt(PEGASUS_CAN_TP_ID, (uint8_t *)rxMsg, RxMessageBuf.DataLength, &firmware_up_recv_shim, &firmware_up_recv_message, &firmware_up_recv_handle);
  }
}

/* FDCAN TX Complete Callback - automatically sends next message */
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
  if (hfdcan->Instance == FDCAN2)
  {
    // Transmission complete, send next message from queue
    CAN_Queue_Process();
  }
}

// Callback
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  if (hfdcan->Instance == FDCAN2)
  {
    if (ErrorStatusITs & FDCAN_IT_BUS_OFF)
    {
      HAL_FDCAN_Stop(&hfdcan2);
      CAN_Queue_Init();
      HAL_FDCAN_Start(&hfdcan2);
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_7)
  {
    d.z_pulse ^= 1;
    d.z_count++;

    /* Stopping uint32_t overflow */
    if (d.z_count >= UINT32_MAX)
    {
      d.z_count = 0;
    }

    /* Not Resetting mech angle and TIM2 reg on every z index pulse, only getting the TIM2 count */
    if (cS == FOC_START)
    {
      /* Getting TIM2 count at Z trig */
      d.count_at_z = __HAL_TIM_GET_COUNTER(&htim2);
      /* Intentionally not resetting mech_angle or TIM2 on Z pulse — angle continuity is maintained by offset calibration */
    }

    /* Storing the prev Z count */
    d.prev_z_count = d.curr_z_count;
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // If the interrupt is triggered by channel 1
    {
      /* Using TIM5 to get the initial angle on startup from the encoder PWM duty */
      // Read the IC value
      d.ICvalue = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

      if (d.ICvalue != 0)
      {
        // calculate the Duty Cycle
        d.Duty = (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2) * 100.0f) / d.ICvalue;

        /* TIM5 is on APB1, so its kernel clock is NOT SYSCLK. With TIMPRE = 0
         * and the APB1 prescaler at /2, the timer clock equals rcc_hclk1 —
         * HCLK, half of SYSCLK. Using SysClockFreq here made this 2x high. */
        d.Frequency = (float)HAL_RCC_GetHCLKFreq() / (d.ICvalue * (TIM5_PSC + 1));

        d.abs_Angle_pwm = (100.0f - d.Duty) * 0.01f * TWO_PI - HIGH_PULSE16_ERROR;
        d.abs_Angle_pwm = fmodf(d.abs_Angle_pwm, TWO_PI);
      }
    }
  }
}

void OTA_Manager(){

    switch(upgrade_state)
    {
    case UPGRADE_INIT:
      __HAL_TIM_MOE_DISABLE(&htim1);
      HAL_TIM_Base_Stop_IT(&htim1);
      // HAL_TIM_Base_Stop_IT(&htim17);
      HAL_ADCEx_InjectedStop_IT(&hadc2);
      OTA_FDCAN_SETUP();
      handle_upgrade_init();
      break;
    case UPGRADE_RECEIVE_DATA:
      handle_rceive_data();
      break;
    case UPGRADE_COMPLETE:
      handle_upgrade_complete();
      break;
    case UPGRADE_FAILED:
      OTA_FDCAN_RESTORE();
      HAL_TIM_Base_Start_IT(&htim1);
      HAL_TIM_Base_Start_IT(&htim17);
      OTA_State = 0;
      break;
    case UPGRADE_WAIT:
      //Handle_Bin_transfer_Complete();
      break;
    default:
        //Add somethin here
      break;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
