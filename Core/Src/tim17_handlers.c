/* tim17_handlers.c — 1 kHz TIM17 tick logic extracted from main.c
 * Call tim17_1ms_tick() from HAL_TIM_PeriodElapsedCallback when htim->Instance == TIM17. */

#include "tim17_handlers.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc2;
extern FDCAN_HandleTypeDef hfdcan2;

/* EEPROM deferred-write flags — written here (ISR context), read/cleared in main loop */
volatile uint8_t  eeprom_write_pending  = 0;
volatile uint16_t eeprom_cw_pending     = 0;
volatile uint16_t eeprom_ccw_pending    = 0;
volatile uint16_t eeprom_cw_high_pending = 0;  /* page0 3rd slot — write-only (read-back ignores it) */

/* Run temperature/encoder disconnection checks and limit-error checks.
 * Only meaningful while the drive is actively running. */
#if ENABLE_FAULTS
static void run_fault_checks(void)
{
    if (cS != FOC_START && cS != ANGLE_CALIB) return;
    Sensor_Disconnection_Check();
    Error_Check();
}
#endif

/* Snapshot angle offsets into EEPROM pending registers and signal the main
 * loop to perform the blocking I2C write. Transitions cS to ANGLE_CALIB_DONE
 * so this block is entered exactly once per calibration. */
static void handle_offset_store(void)
{
    if (cS != ANGLE_OFFSET_STORE) return;
    eeprom_cw_pending      = (uint16_t)(d.offset_angle_mech_cw  * 1000.0f);
    eeprom_ccw_pending     = (uint16_t)(d.offset_angle_mech_ccw * 1000.0f);
    eeprom_cw_high_pending = (uint16_t)((d.offset_angle_mech_cw + 0.1f) * 1000.0f);
    d.offset_angle_mech_avg = (eeprom_ccw_pending + eeprom_cw_pending) / 2.0f;
    eeprom_write_pending = 1;
    cS = ANGLE_CALIB_DONE;  /* prevent re-entry; main loop performs the write */
}

/* Safe shutdown + MCU reset.  Disables PWM outputs and ADC injection before
 * calling NVIC reset so the motor coasts to a stop rather than being driven
 * through an unknown state. */
static void handle_reset_state(void)
{
    if (cS != RESET_STATE) return;

    /* Turn off drive completely */
    HAL_GPIO_WritePin(DRIVE_INHIBIT_GPIO_Port, DRIVE_INHIBIT_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TIM1_ARR_HALF);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, TIM1_ARR_HALF);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, TIM1_ARR_HALF);
    __HAL_TIM_MOE_DISABLE(&htim1);

    /* Stop ADC injected interrupt */
    HAL_ADCEx_InjectedStop_IT(&hadc2);

    /* Reset FOC model data structs */
    (void)memset(&FOC_Loop_U, 0, sizeof(ExtU_FOC_Loop_T));
    (void)memset(&FOC_Loop_Y, 0, sizeof(ExtY_FOC_Loop_T));

    HAL_NVIC_SystemReset();
}

/* Check for BUS OFF and recover, then queue all periodic CAN messages.
 * Returns early without queuing if BUS OFF recovery was required this cycle. */
static void queue_can_messages(void)
{
    d.canBus_ok = 1;

    FDCAN_ProtocolStatusTypeDef psr;
    HAL_FDCAN_GetProtocolStatus(&hfdcan2, &psr);

    if (psr.BusOff)
    {
        HAL_FDCAN_Stop(&hfdcan2);
        CAN_Queue_Init();           /* flush stale queue */
        HAL_FDCAN_Start(&hfdcan2);  /* re-enters error-active after 128×11 recessive bits */
        d.canBus_ok = 0;
    }

    if (!d.canBus_ok) return;

    #if DEBUG_CAN_ID
    Send_Data_On_CAN_401();
    Send_Data_On_CAN_402();
    Send_Data_On_CAN_403();
    Send_Data_On_CAN_404();
    Send_Data_On_CAN_405();
    Send_Data_On_CAN_406();
    Send_Data_On_CAN_407();
    Send_Data_On_CAN_408();
    Send_Data_On_CAN_409();
    Send_Data_On_CAN_410();
    Send_Data_On_CAN_411();
    Send_Data_On_CAN_412();
    #if FOC_BENCH_FAST_TLM
    Send_Data_On_CAN_40A();
    #endif
    #endif

    #if VH_CAN_ID
    Send_on_CAN_705(d.Mtr_temp, d.Mtc_temp);
    /* RPM_TO_KMPH is auto-selected by the vehicle variant (vehicle_config.h) */
    Send_on_CAN_706(d.Vdc, fabsf(d.RPM * RPM_TO_KMPH), er.error_c1);
    Send_on_CAN_708(er.error_c2, er.error_c3);
    Send_on_CAN_709(0);
    Send_on_CAN_710(d.throttle_percent, d.throttle_v);
    Send_on_CAN_715(fabsf(d.RPM), d.Vdc, d.Aux_dc);
    Send_on_CAN_716(Current_Ref_Estimator_Y.V_s, d.irms);
    Send_on_CAN_717(fabsf(d.RPM));
    Send_on_CAN_724(Current_Ref_Estimator_Y.Actual_torque_from_ref);
    Send_on_CAN_726();
    Schedule_7Ax_Transmission();
    #endif
}

/* Public entry point — call from HAL_TIM_PeriodElapsedCallback when TIM17 fires. */
void tim17_1ms_tick(void)
{
    static uint16_t can_counter = CAN_BUS_CYCLE;  /* init to limit so first tick fires immediately */
    can_counter++;

    Vdc_Filter_Update_1ms();

    #if FOC_BENCH_FAST_TLM
    Fast_Tlm_Tick_1ms();
    #endif

    #if ENABLE_FAULTS
    run_fault_checks();
    #endif

    handle_offset_store();
    handle_reset_state();

    if (can_counter >= CAN_BUS_CYCLE)
    {
        can_counter = 0;
        queue_can_messages();
    }
}
