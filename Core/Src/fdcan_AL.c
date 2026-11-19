#include "fdcan.h"

extern FDCAN_HandleTypeDef hfdcan2;
FDCAN_RxHeaderTypeDef RxMessageBuf;
extern TIM_HandleTypeDef htim4;

extern image_hdr_t image_hdr;
extern uint8_t UID[12];

CAN_TxQueue_t can_tx_queue = {0};

/* ---- CAN payload packing --------------------------------------------------
 * Every multi-byte field on both matrices is a little-endian u16. Pack via
 * this helper instead of hand-rolled shift/mask pairs so byte order and
 * width live in exactly one place. Float fields are truncated to u16 at the
 * call site, same as the original casts. */
static inline void pack_u16_le(uint8_t *buf, uint8_t idx, uint16_t v)
{
	buf[idx]      = (uint8_t)(v & 0xFFU);
	buf[idx + 1U] = (uint8_t)((v >> 8) & 0xFFU);
}

/* 0x706 byte 0 — dashboard drive-state code (cluster protocol values) */
#define CAN706_STATE_NEUTRAL     0x01U
#define CAN706_STATE_FWD_ECO     0x02U
#define CAN706_STATE_REVERSE     0x04U
#define CAN706_STATE_FWD_SPORTS  0x08U

/* 0x706 byte 1 — status flag bit positions */
#define CAN706_BIT_REGEN         2
#define CAN706_BIT_LOW_VDC       3
#define CAN706_BIT_MTC_OT        4
#define CAN706_BIT_DRIVE_ON      5

/* Initialize the queue */
void CAN_Queue_Init(void)
{
    can_tx_queue.head = 0;
    can_tx_queue.tail = 0;
    can_tx_queue.count = 0;
}

/* Add message to queue - ISR safe.
 * Pushers run at TIM17 (prio 2) and TIM4 (prio 7), the drain at FDCAN (prio 6),
 * so any of them can preempt another mid-update: the whole push is a critical
 * section (a few dozen cycles), not just the pointer update. */
uint8_t CAN_Queue_Push(uint32_t arbitration_id, uint32_t format, uint8_t *data, uint8_t dlc)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (can_tx_queue.count >= CAN_TX_QUEUE_SIZE) {
        __set_PRIMASK(primask);
        return 0;  // Queue full
    }

    // Get current head position
    uint16_t current_head = can_tx_queue.head;

    // Copy message to queue buffer
    can_tx_queue.buffer[current_head].arbitration_id = arbitration_id;
    can_tx_queue.buffer[current_head].format = format;
    can_tx_queue.buffer[current_head].dlc = dlc;

    for (uint8_t i = 0; i < dlc && i < 8; i++) {
        can_tx_queue.buffer[current_head].data[i] = data[i];
    }

    can_tx_queue.head = (current_head + 1) % CAN_TX_QUEUE_SIZE;
    can_tx_queue.count++;

    __set_PRIMASK(primask);

    return 1;  // Success
}

/* Check if queue has messages */
uint8_t CAN_Queue_IsEmpty(void)
{
    return (can_tx_queue.count == 0);
}

/* Process queue - send one message if available. Called from the FDCAN TX
 * complete IRQ and from Push_And_Kickstart (TIM17/TIM4 ISRs); the whole body
 * is a critical section so a preempting push cannot race the count/tail
 * update or start a second, nested send of the same slot. */
void CAN_Queue_Process(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (can_tx_queue.count == 0) {
        __set_PRIMASK(primask);
        return;  // Nothing to send
    }

    // Check if FDCAN Tx FIFO has space
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0) {
        __set_PRIMASK(primask);
        return;  // FDCAN FIFO is full, try again later
    }
    
    // Get current tail position
    uint16_t current_tail = can_tx_queue.tail;
    
    // Read message data (safe to read without lock)
    CAN_TxMessage_t *msg = &can_tx_queue.buffer[current_tail];
    
    FDCAN_TxHeaderTypeDef TxMsg;
    TxMsg.IdType = msg->format;
    TxMsg.TxFrameType = FDCAN_DATA_FRAME;
    TxMsg.Identifier = msg->arbitration_id;
    TxMsg.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxMsg.BitRateSwitch = FDCAN_BRS_OFF;
    TxMsg.FDFormat = FDCAN_CLASSIC_CAN;
    TxMsg.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxMsg.MessageMarker = 0;
    TxMsg.DataLength = msg->dlc;
    
    // Try to send
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxMsg, msg->data) == HAL_OK) {
        can_tx_queue.tail = (current_tail + 1) % CAN_TX_QUEUE_SIZE;
        can_tx_queue.count--;
    }

    __set_PRIMASK(primask);
}

/* Get queue status */
uint16_t CAN_Queue_GetCount(void)
{
    return can_tx_queue.count;
}

/* Kick-start transmission when first message is queued */
void CAN_Queue_Push_And_Kickstart(uint32_t arbitration_id, uint32_t format, uint8_t *data, uint8_t dlc)
{
    /* Always try to send after a push. Gating this on "queue was empty" (read
     * before the push) could stall the queue for good: a TX-complete drain
     * landing between that read and the push leaves a frame queued with
     * nothing in flight, and every later push then sees "not empty".
     * Process is a critical section and no-ops when the hardware FIFO is full. */
    if (CAN_Queue_Push(arbitration_id, format, data, dlc)) {
        CAN_Queue_Process();
    }
}

void Schedule_7Ax_Transmission(void)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim4, (PEGASUS_DEVICE_ID * TICKS_PER_SLOT) - 1);
    HAL_TIM_Base_Start_IT(&htim4);
}

void _fdcan_filter_IDList(uint32_t can_receive_id, uint32_t format, uint32_t filter_bank, uint32_t fifo)
{
	FDCAN_FilterTypeDef sFilterConfig;

	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterIndex = filter_bank;

	if(format == 0){
		sFilterConfig.IdType = FDCAN_STANDARD_ID;
	}
	else if(format == 1){
		sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	}
	sFilterConfig.FilterID1 = can_receive_id;
	sFilterConfig.FilterID2 = (format == 1) ? 0x1FFFFFFF : 0x7FF;

	if (fifo == 0)
	{
		sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	}
	else if (fifo == 1)
  	{
   		sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	}

	if(HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) {
		//Error Handler Code to be added later to act as redundancy.
      Error_Handler();
    }

    /* Configure global filter to reject all non-matching frames */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

	#if DEBUG_CAN_ID
    /* Configure Rx FIFO 0 watermark to 10 */
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 10);
	#endif

	#if VH_CAN_ID
	/* Configure Rx FIFO 0 watermark to 3 (0x109 SOC/Vpack, 0x110 batt current, 0x12A DCLO/DCLI) */
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 8);
	#endif

    /* Activate Rx FIFO 0 watermark notification */
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void FDCAN_SETUP()
{
	/* Filter banks are unique per ID (DEBUG 0-9, VH 10-14, OTA 15-17) so the
	 * two ID sets and OTA can coexist. 0x109/0x110 clash with the BMS IDs, so
	 * the debug power-mode / throttle-voltage frames live on 0x113 / 0x114. */
	#if DEBUG_CAN_ID
	_fdcan_filter_IDList(0x102, 0, 0, 0);
	_fdcan_filter_IDList(0x103, 0, 1, 0);
	_fdcan_filter_IDList(0x104, 0, 2, 0);
	_fdcan_filter_IDList(0x105, 0, 3, 0);
	_fdcan_filter_IDList(0x106, 0, 4, 0);
	_fdcan_filter_IDList(0x107, 0, 5, 0);
	_fdcan_filter_IDList(0x108, 0, 6, 0);
	_fdcan_filter_IDList(0x113, 0, 7, 0);
	_fdcan_filter_IDList(0x114, 0, 8, 0);
	_fdcan_filter_IDList(0x111, 0, 9, 0);
	#endif
	#if VH_CAN_ID
	_fdcan_filter_IDList(0x109, 0, 10, 0);
	_fdcan_filter_IDList(0x110, 0, 11, 0);
	_fdcan_filter_IDList(0x12A, 0, 12, 0);
	_fdcan_filter_IDList(0x725, 0, 13, 0);
	#if CAN_AUTOTUNE_TRIGGER
	_fdcan_filter_IDList(0x729, 0, 14, 0);
	#endif
	#endif
	/* OTA firmware-upgrade IDs (OTA_Reintegrated) — standard IDs, own banks.
	 * Handled in main.c RX cb. */
	_fdcan_filter_IDList(0x6FA, 0, 15, 0);
	_fdcan_filter_IDList(0x7FE, 0, 16, 0);
	_fdcan_filter_IDList(0x7FF, 0, 17, 0);
	#if FOC_ISENSE_DELAY_COMP
	/* Bench: live current-sense delay tau (can_rx_handlers.c). Bank 18 needs
	 * hfdcan2.Init.StdFiltersNbr >= 19 (fdcan.c). */
	_fdcan_filter_IDList(0x115, 0, 18, 0);
	#endif
	#if FOC_BENCH_FAST_TLM
	/* Bench: fast telemetry on/off. Bank 19 needs StdFiltersNbr >= 20. */
	_fdcan_filter_IDList(0x116, 0, 19, 0);
	#endif
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_TX_COMPLETE, 0);

	if(HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
	{
		Error_Handler();
	}
	// In FDCAN setup after HAL_FDCAN_Start()
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0);
}

/* Quiet the CAN interface for the duration of an OTA session: reinit FDCAN2
 * and accept ONLY the firmware-upgrade IDs (0x6FA cmd/status, 0x7FE ISO-TP
 * data, 0x7FF ISO-TP flow control), rejecting all other bus traffic.
 * Without this, BMS/vehicle frames (0x109/0x110/0x12A/0x725/0x729) compete
 * with the ISO-TP stream for Rx FIFO0 slots and ISR time while Flash writes
 * run with IRQs disabled, and each dropped consecutive frame costs a full
 * Flash_Console packet timeout+retry. Restored from the pre-merge production
 * firmware ("Disabling BMS CAN IDs during OTA"). */
void OTA_FDCAN_SETUP(void)
{
	__disable_irq();
	HAL_FDCAN_DeInit(&hfdcan2);
	__HAL_RCC_FDCAN_FORCE_RESET();
	__HAL_RCC_FDCAN_RELEASE_RESET();
	HAL_FDCAN_Init(&hfdcan2);

	/* Drop any telemetry still sitting in the SW Tx queue from before OTA */
	CAN_Queue_Init();

	/* Reject everything that doesn't match a filter */
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
	                             FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

	FDCAN_FilterTypeDef sFilter;
	sFilter.IdType       = FDCAN_STANDARD_ID;
	sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilter.FilterType   = FDCAN_FILTER_DUAL;

	/* Filter 0: 0x6FA, 0x7FE */
	sFilter.FilterIndex = 0;
	sFilter.FilterID1   = 0x6FA;
	sFilter.FilterID2   = 0x7FE;
	HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilter);

	/* Filter 1: 0x7FF only */
	sFilter.FilterIndex = 1;
	sFilter.FilterID1   = 0x7FF;
	sFilter.FilterID2   = 0x7FF;
	HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilter);

	/* Message RAM survives the peripheral reset, so the vehicle-ID filters
	 * programmed by FDCAN_SETUP() would still match — disable every other bank */
	sFilter.FilterConfig = FDCAN_FILTER_DISABLE;
	for (uint32_t bank = 2; bank < hfdcan2.Init.StdFiltersNbr; bank++) {
		sFilter.FilterIndex = bank;
		sFilter.FilterID1   = 0x0;
		sFilter.FilterID2   = 0x0;
		HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilter);
	}

	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0);

	HAL_FDCAN_Start(&hfdcan2);

	__DSB();
	__ISB();
	__enable_irq();
}

/* Undo OTA_FDCAN_SETUP after a failed/aborted OTA session: full reinit, then
 * the normal application filter set. (On a successful OTA the MCU resets and
 * boots into the new image, so no restore is needed there.) */
void OTA_FDCAN_RESTORE(void)
{
	__disable_irq();
	HAL_FDCAN_DeInit(&hfdcan2);
	__HAL_RCC_FDCAN_FORCE_RESET();
	__HAL_RCC_FDCAN_RELEASE_RESET();
	HAL_FDCAN_Init(&hfdcan2);
	CAN_Queue_Init();
	__enable_irq();

	FDCAN_SETUP();	/* reprograms every application + OTA bank */
}

void _fdcan_transmit_on_can(uint32_t arbitration_id, uint32_t format, uint8_t * can_data, uint8_t dlc)
{
	FDCAN_TxHeaderTypeDef TxMsg;

	TxMsg.IdType 				= format;
	TxMsg.TxFrameType           = FDCAN_DATA_FRAME;
	TxMsg.Identifier            = arbitration_id;
	TxMsg.ErrorStateIndicator   = FDCAN_ESI_ACTIVE;
	TxMsg.BitRateSwitch         = FDCAN_BRS_OFF;
	TxMsg.FDFormat              = FDCAN_CLASSIC_CAN;
	TxMsg.TxEventFifoControl    = FDCAN_NO_TX_EVENTS;
	TxMsg.MessageMarker         = 0;
	TxMsg.DataLength			= dlc;

	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxMsg, can_data) != HAL_OK)
		__HAL_FDCAN_CLEAR_FLAG(&hfdcan2, FDCAN_TXBC_TFQM);
}

#if DEBUG_CAN_ID
void Send_Data_On_CAN_401(void)
{
	uint8_t can_data[8] = {0};

	uint16_t rpm_ref = (uint16_t)(FOC_Loop_U.Ref_Speed_mech_rpm);
	uint16_t rpm_fdbk = (uint16_t)(fabsf(d.RPM));
	uint16_t Id = (uint16_t)(FOC_Loop_Y.Id + 512.0f);
	uint16_t Iq = (uint16_t)(FOC_Loop_Y.Iq + 512.0f);

	pack_u16_le(can_data, 0, rpm_ref);
	pack_u16_le(can_data, 2, rpm_fdbk);
	pack_u16_le(can_data, 4, Id);
	pack_u16_le(can_data, 6, Iq);

	// _fdcan_transmit_on_can(0x401, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
	CAN_Queue_Push_And_Kickstart(0x401, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
}

void Send_Data_On_CAN_402(void)
{
	// Implement similar to Send_Data_On_CAN_401 with different data
	uint8_t can_data[8] = {0};
	uint16_t Vd = 0;
	uint16_t Vq = 0;
	uint16_t Iq_ref = 0;
	uint16_t Id_ref = 0;

	if (cS == ANGLE_CALIB) {
		if (d.Open_FOC) {
			Vd = (uint16_t)(Open_FOC0_Y.Vd + 159.0f);
			Vq = (uint16_t)(Open_FOC0_Y.Vq + 159.0f);
			Iq_ref = (uint16_t)(Open_FOC0_U.Iq_ref + 512.0f);
			Id_ref = (uint16_t)(Open_FOC0_U.Id_ref + 512.0f);
		} else if (d.Closed_FOC) {
			Vd = (uint16_t)(FOC_Loop_Y.Vd + 159.0f);
			Vq = (uint16_t)(FOC_Loop_Y.Vq + 159.0f);
			Iq_ref = (uint16_t)(Current_Ref_Estimator_Y.Iq_ref_final + 512.0f);
			Id_ref = (uint16_t)(Current_Ref_Estimator_Y.Id_ref_final + 512.0f);
		}
	} else if (cS == FOC_START) {
		Vd = (uint16_t)(FOC_Loop_Y.Vd + 159.0f);
		Vq = (uint16_t)(FOC_Loop_Y.Vq + 159.0f);
		Iq_ref = (uint16_t)(Current_Ref_Estimator_Y.Iq_ref_final + 512.0f);
		Id_ref = (uint16_t)(Current_Ref_Estimator_Y.Id_ref_final + 512.0f);
	}
	uint16_t enc_cnt = (uint16_t)(d.encoder_count);

	pack_u16_le(can_data, 0, Iq_ref);
	pack_u16_le(can_data, 2, Id_ref);
	can_data[4] = (uint8_t)(Vd);
	can_data[5] = (uint8_t)(Vq);
	pack_u16_le(can_data, 6, enc_cnt);

	// _fdcan_transmit_on_can(0x402, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
	CAN_Queue_Push_And_Kickstart(0x402, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
}

void Send_Data_On_CAN_403(void)
{
	uint8_t can_data[8] = {0};

	uint16_t bus_voltage = (uint16_t)(d.Vdc * 100.0f);
	uint16_t aux_voltage = (uint16_t)(d.Aux_dc * 100.0f);
	uint16_t motor_temp = (uint16_t)(d.Mtr_temp + 50.0f);
	uint16_t mc_temp = (uint16_t)(d.Mtc_temp + 50.0f);

	pack_u16_le(can_data, 0, bus_voltage);
	pack_u16_le(can_data, 2, aux_voltage);
	can_data[4] = (uint8_t)(motor_temp);
	can_data[5] = (uint8_t)(mc_temp);
	can_data[6] = (uint8_t)err;
	can_data[7] = (uint8_t)(FOC_Loop_Y.Actual_torque_from_meas + 80.0f);

	// _fdcan_transmit_on_can(0x403, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
	CAN_Queue_Push_And_Kickstart(0x403, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
}

void Send_Data_On_CAN_404(void) {
	uint8_t can_data[8] = {0};

	uint16_t Ld_LUT = (uint16_t)(Current_Ref_Estimator_Y.Ld * 1.0E8f);
	uint16_t Lq_LUT = (uint16_t)(Current_Ref_Estimator_Y.Lq * 1.0E7f);
	uint16_t lambda_LUT = (uint16_t)(Current_Ref_Estimator_Y.Lambda * 1.0E5f);
	uint16_t motor_Rs = (uint16_t)(Current_Ref_Estimator_Y.Rs * 1.0E6f);

	pack_u16_le(can_data, 0, Ld_LUT);
	pack_u16_le(can_data, 2, Lq_LUT);
	pack_u16_le(can_data, 4, lambda_LUT);
	pack_u16_le(can_data, 6, motor_Rs);

	// _fdcan_transmit_on_can(0x404, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
	CAN_Queue_Push_And_Kickstart(0x404, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
}

void Send_Data_On_CAN_405(void) {
	uint8_t can_data[8] = {0};

	uint16_t can_offsetAngle = (uint16_t)(d.offset_angle_mech_avg * 1000.0f);
	uint16_t can_thrVoltage = (uint16_t)(FOC_Loop_U.Throttle_input.Throttle_Inst_Voltage * 1000.0f);

	can_data[0] = (uint8_t)(FOC_Loop_Y.Throttle_State);
	can_data[1] = (uint8_t)(cS);
	if (rtmGetErrorStatus(FOC_Loop_M) == "Overrun") {
		can_data[2] = 0x01;
	} else {
		can_data[2] = 0x00;
	}
	can_data[3] = (uint8_t)(pw_state);
	pack_u16_le(can_data, 4, can_offsetAngle);
	pack_u16_le(can_data, 6, can_thrVoltage);

	CAN_Queue_Push_And_Kickstart(0x405, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
}

void Send_Data_On_CAN_406(void) {
	uint8_t can_data[8] = {0};

#if FOC_ISENSE_DELAY_COMP
	/* [0..1] current-sense delay tau in us instead of Kfw, which is dead
	 * while FW_CLOSED_LOOP compiles the generated FW chart out. */
	uint16_t can_tx_Kfw = (uint16_t)(can_d.isense_delay_s * 1.0E6f + 0.5f);
#else
	uint16_t can_tx_Kfw = (uint16_t)(Current_Ref_Estimator_U.FWC_values.Kfw * 1.0E3f);
#endif
	uint16_t can_tx_Kaw = (uint16_t)(Current_Ref_Estimator_U.FWC_values.Kaw * 1.0E3f);
	uint16_t can_Vs = (uint16_t)(Current_Ref_Estimator_Y.V_s * 1000.0f);
	uint16_t can_Vmax = (uint16_t)(Current_Ref_Estimator_Y.V_max * 1000.0f);

	pack_u16_le(can_data, 0, can_tx_Kfw);
	pack_u16_le(can_data, 2, can_tx_Kaw);
	pack_u16_le(can_data, 4, can_Vs);
	pack_u16_le(can_data, 6, can_Vmax);

	CAN_Queue_Push_And_Kickstart(0x406, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
}

void Send_Data_On_CAN_407(void) {
	uint8_t can_data[8] = {0};

	uint16_t can_tx_speed_kp = (uint16_t)(FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Kp_speed_PID_MTPA * 1.0E2f);
	uint16_t can_tx_speed_ki = (uint16_t)(FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Ki_speed_PID_MTPA * 1.0E2f);
	uint16_t can_tx_speed_kd = (uint16_t)(FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Kd_speed_PID_MTPA * 1.0E5f);
	uint8_t can_tx_M2F = (uint8_t)(Current_Ref_Estimator_U.FWC_values.M2F_factor * 100.0f);
	uint8_t can_tx_F2M = (uint8_t)(Current_Ref_Estimator_U.FWC_values.F2M_factor * 100.0f);

	pack_u16_le(can_data, 0, can_tx_speed_kp);
	pack_u16_le(can_data, 2, can_tx_speed_ki);
	pack_u16_le(can_data, 4, can_tx_speed_kd);
	can_data[6] = can_tx_M2F;
	can_data[7] = can_tx_F2M;

	CAN_Queue_Push_And_Kickstart(0x407, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
}

void Send_Data_On_CAN_408(void) {
	uint8_t can_data[8] = {0};

	uint16_t can_tx_speed_kp = (uint16_t)(FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Kp_flux_PID_MTPA * 1.0E2f);
	uint16_t can_tx_speed_ki = (uint16_t)(FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Ki_flux_PID_MTPA * 1.0E2f);
	uint16_t can_tx_speed_kd = (uint16_t)(FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Kd_flux_PID_MTPA * 1.0E5f);
	uint8_t can_tx_Rst_Factor = (uint8_t)(Current_Ref_Estimator_U.FWC_values.FW_Int_reset_factor * 100.0f);

	pack_u16_le(can_data, 0, can_tx_speed_kp);
	pack_u16_le(can_data, 2, can_tx_speed_ki);
	pack_u16_le(can_data, 4, can_tx_speed_kd);
	can_data[6] = can_tx_Rst_Factor;
	can_data[7] = (uint8_t)(FOC_Loop_U.Speed_1_Torque_0);

	CAN_Queue_Push_And_Kickstart(0x408, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
}

void Send_Data_On_CAN_409(void) {
	uint8_t can_data[8] = {0};

	uint16_t can_tx_speed_kp = (uint16_t)(FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Kp_torque_PID_MTPA * 1.0E2f);
	uint16_t can_tx_speed_ki = (uint16_t)(FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Ki_torque_PID_MTPA * 1.0E2f);
	uint16_t can_tx_speed_kd = (uint16_t)(FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Kd_torque_PID_MTPA * 1.0E5f);
	uint8_t can_tx_Vs 	 	 = (uint8_t)(Current_Ref_Estimator_Y.V_s);
	uint8_t can_tx_Vmax 	 = (uint8_t)(Current_Ref_Estimator_Y.V_max);

	pack_u16_le(can_data, 0, can_tx_speed_kp);
	pack_u16_le(can_data, 2, can_tx_speed_ki);
	pack_u16_le(can_data, 4, can_tx_speed_kd);
	can_data[6] = can_tx_Vs;
	can_data[7] = can_tx_Vmax;

	CAN_Queue_Push_And_Kickstart(0x409, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
}

void Send_Data_On_CAN_410(void) {
	uint8_t can_data[8] = {0};

	uint16_t isr_us = (uint16_t)(d.cycles / 200.0f); // Convert cycle count to microseconds

	can_data[0] = CONFIG_VERSION_MAJOR;
	can_data[1] = CONFIG_VERSION_MINOR;
	can_data[2] = CONFIG_VERSION_SUBMINOR;
	can_data[3] = FIRMWARE_VERSION_MAJOR;
	can_data[4] = FIRMWARE_VERSION_MINOR;
	can_data[5] = FIRMWARE_VERSION_SUBMINOR;
	can_data[6] = (uint8_t)(Current_Ref_Estimator_Y.MTPA_FWC_condition);
	can_data[7] = (uint8_t)(isr_us & 0x00FF); // Send microseconds

	CAN_Queue_Push_And_Kickstart(0x410, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
}

extern uint16_t injectedVal[3];
extern uint16_t currSensOff[3];

/* 0x411: U/V/W NTC temperatures. Each is int16 LE, 0.1 degC per count.
 * [0..1] U_TEMP (PA0)  [2..3] V_TEMP (PA1)  [4..5] W_TEMP (PA2)  [6..7] 0 */
void Send_Data_On_CAN_411(void) {
	uint8_t can_data[8] = {0};

	pack_u16_le(can_data, 0, (uint16_t)((int16_t)(d.temp_u * 10)));
	pack_u16_le(can_data, 2, (uint16_t)((int16_t)(d.temp_v * 10)));
	pack_u16_le(can_data, 4, (uint16_t)((int16_t)(d.temp_w * 10)));

	CAN_Queue_Push_And_Kickstart(0x411, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
}

/* 0x412: U/V/W phase currents. Each is int16 LE, 0.1 A per count, offset-corrected.
 * [0..1] U (PB1)  [2..3] V (PF11)  [4..5] W (PB0, display only)  [6..7] 0 */
void Send_Data_On_CAN_412(void) {
	uint8_t can_data[8] = {0};

	for (uint8_t i = 0; i < 3; i++) {
		float amps = (float)((int32_t)injectedVal[i] - (int32_t)currSensOff[i]) * ADC_TO_CURR;
		pack_u16_le(can_data, (uint8_t)(i * 2U), (uint16_t)((int16_t)(amps * 10.0f)));
	}

	CAN_Queue_Push_And_Kickstart(0x412, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
}

#if FOC_BENCH_FAST_TLM
/* Bench encoder diagnostics (10 Hz): where the Z index lands against the TIM2
 * count, and the PWM-derived count the encoder is seeded with at boot. */
void Send_Data_On_CAN_40A(void) {
	uint8_t can_data[8] = {0};

	pack_u16_le(can_data, 0, (uint16_t)(d.count_at_z & 0xFFFFU));
	pack_u16_le(can_data, 2, (uint16_t)(d.z_count & 0xFFFFU));
	pack_u16_le(can_data, 4, (uint16_t)(d.Count_From_Duty & 0xFFFFU));
	/* [6..7] reserved, 0 */

	CAN_Queue_Push_And_Kickstart(0x40A, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
}
#endif
#endif

#if VH_CAN_ID
/* Vehicle CAN Matrix */
/*****************************************************************************/
void Send_on_CAN_705(float Mtr_Temp, float Cntrl_Temp)
{
	uint8_t data[8] = {0};

	float mt_t = Mtr_Temp + 50.0f;
	float ct_t = Cntrl_Temp + 50.0f;
	float can_iq = FOC_Loop_Y.Iq + 512.0f;
	float can_id = FOC_Loop_Y.Id + 512.0f;
	float offset_angle = d.offset_angle_mech_cw * 100.0f;

	data[0] = (uint8_t)((uint16_t)mt_t);
	data[1] = (uint8_t)((uint16_t)ct_t);

	pack_u16_le(data, 2, (uint16_t)can_iq);
	pack_u16_le(data, 4, (uint16_t)can_id);
	pack_u16_le(data, 6, (uint16_t)offset_angle);

	CAN_Queue_Push_And_Kickstart(0x705, (uint32_t)FDCAN_STANDARD_ID, data, 0x08);
}

void Send_on_CAN_706(float bus_voltage, float Speed_KMPH, uint8_t error_c)
{
	uint8_t data[8] = {0};

	float offset_angle = d.offset_angle_mech_ccw * 100.0f;

	/* Byte 0 — drive-state code. FORWARD splits by power mode; any other
	 * pw_state value leaves 0x00 (unknown), matching original behaviour. */
	switch (fnr_state) {
	case REVERSE:
		data[0] = CAN706_STATE_REVERSE;
		break;
	case NEUTRAL:
		data[0] = CAN706_STATE_NEUTRAL;
		break;
	case FORWARD:
		if (pw_state == SPORTS) {
			data[0] = CAN706_STATE_FWD_SPORTS;
		} else if (pw_state == ECO) {
			data[0] = CAN706_STATE_FWD_ECO;
		}
		break;
	default:
		break;
	}

	if (d.regen_flag) {
		data[1] |= (1 << CAN706_BIT_REGEN);
	} else {
		data[1] &= ~(1 << CAN706_BIT_REGEN);
	}

	if (bus_voltage <= 40)
		data[1] |= (1 << CAN706_BIT_LOW_VDC);
	else
		data[1] &= ~(1 << CAN706_BIT_LOW_VDC);

	if (!er.drive_off) {
		data[1] |= (1 << CAN706_BIT_DRIVE_ON);
	} else {
		data[1] &= ~(1 << CAN706_BIT_DRIVE_ON);
	}

	if (er.mtc_temp_ot_error) {
		data[1] |= (1 << CAN706_BIT_MTC_OT);
	}

	data[3] = error_c;

	/* Hysteresis rounding: only step the displayed integer when raw speed
	* crosses the half-integer boundary by 0.3 kmph, preventing the display
	* from toggling (e.g. 45↔46) due to minor Filtered_Speed ripple. */
	{
		static float display_kmph = -1.0f;
		if (display_kmph < 0.0f || fabsf(Speed_KMPH - display_kmph) > 2.0f)
			display_kmph = roundf(Speed_KMPH);
		else if (Speed_KMPH >= display_kmph + 0.8f || Speed_KMPH <= display_kmph - 0.8f)
			display_kmph = roundf(Speed_KMPH);
		Speed_KMPH = display_kmph;
	}

	Speed_KMPH *= 10.0f;
	pack_u16_le(data, 4, (uint16_t)Speed_KMPH);
	pack_u16_le(data, 6, (uint16_t)offset_angle);

	// data[6] = (uint8_t)((uint16_t)(CAN_Counter) & 0x00FF);
	// data[7] = (uint8_t)(((uint16_t)(CAN_Counter) & 0xFF00) >> 8);

	CAN_Queue_Push_And_Kickstart(0x706, (uint32_t)FDCAN_STANDARD_ID, data, 0x08);
}

void Send_on_CAN_708(uint8_t error_c_1, uint8_t error_c_2)
{
	uint8_t data[8] = {0};

	float theta_elec = d.elec_angle * 100.0f;
	uint16_t encoder_count = (uint16_t)(TIM2->CNT);

	data[0] = error_c_1;
	data[1] = error_c_2;

	if (d.ctlr_derate_flag) {
		data[1] |= (1 << 3);
	} else {
		data[1] &= ~(1 << 3);
	}

	if (d.motor_derate_flag) {
		data[1] |= (1 << 4);
	} else {
		data[1] &= ~(1 << 4);
	}

	pack_u16_le(data, 2, (uint16_t)theta_elec);
	pack_u16_le(data, 4, encoder_count);

	// data[6] = (uint8_t)(CAN_Error_Occured);

	CAN_Queue_Push_And_Kickstart(0x708, (uint32_t)FDCAN_STANDARD_ID, data, 0x08);
}

void Send_on_CAN_709(uint8_t historic_errors)
{
	uint8_t data[8] = {0};
	UNUSED(historic_errors);

	pack_u16_le(data, 2 , d.Max_ph_current);

	CAN_Queue_Push_And_Kickstart(0x709, (uint32_t)FDCAN_STANDARD_ID, data, 0x08);
}

void Send_on_CAN_710(float throttle_percent, float throttle_voltage) {
	uint8_t data[8] = {0};

	data[0] = (uint8_t)((uint16_t)throttle_percent);
	data[2] = (uint8_t)(can_d.brake_command_reply);
	data[3] = (uint8_t)((uint16_t)(throttle_voltage * 10.0f));
	/* Hill-hold telemetry (module accessors — no plumbing; each is a single
	 * word, atomic to sample from this 1 kHz context):
	 *   [4] active flag  (1 = holding/releasing torque, 0 = idle)
	 *   [5] state code   (0 INACTIVE / 1 ACTIVE / 2 RELEASING / 3 COOLDOWN /
	 *                     4 STANDBY — driver took over, grade estimate kept)
	 *   [6..7] hold torque Nm x10, u16 LE */
	data[4] = (uint8_t)(HillHold_IsActive() ? 1U : 0U);
	data[5] = HillHold_GetState();
	pack_u16_le(data, 6, (uint16_t)(HillHold_GetTorque() * 10.0f));

	CAN_Queue_Push_And_Kickstart(0x710, (uint32_t)FDCAN_STANDARD_ID, data, 0x08);
}

void Send_on_CAN_715(float RPM, float bus_voltage, float lv_12v_voltage)
{
	uint8_t data[8] = {0};

	/* [0] commanded |Id|, 2 A/bit  ->  Id = -2 * data[0] A. Id is never
	 *     positive on this PMSM, so the sign is implicit and the whole byte is
	 *     magnitude; 2 A/bit keeps HiLoad's ~-314 A on scale (1 A/bit would
	 *     clip above -255 A).
	 * [1] PEAK modulation index x100 since the previous frame  ->  mod% =
	 *     data[1]. 100 = exactly on the SVM circle, >100 = the commanded dq
	 *     pair is outside it (see data_t.mod_index_peak). Read-and-clear, so
	 *     each frame reports its own window rather than a running maximum.
	 *     With FOC_VDQ_CIRCLE_LIMIT this is the DEMANDED (pre-limit) index;
	 *     the applied one is <= 100 by construction. */
	float id_mag = -Current_Ref_Estimator_Y.Id_ref_final * 0.5f;
	if (id_mag < 0.0f)     id_mag = 0.0f;
	if (id_mag > 255.0f)   id_mag = 255.0f;
	data[0] = (uint8_t)id_mag;

	float mod_pct = d.mod_index_peak * 100.0f;
	d.mod_index_peak = 0.0f;
	if (mod_pct > 255.0f) mod_pct = 255.0f;
	data[1] = (uint8_t)mod_pct;

	data[2] = (uint8_t)(Current_Ref_Estimator_Y.MTPA_FWC_condition);
#if FOC_VDQ_CIRCLE_LIMIT
	/* [3] circle-limit clip ratio, % of FOC steps since the previous frame in
	 *     which Vdq_Circle_Limit shortened the pair. Replaces the pack voltage
	 *     on this build (constant OP_VOLTAGE on the bench, no BMS). Same
	 *     read-and-clear rule as [1]. */
	uint32_t lim_cycles = d.vdq_lim_cycles, lim_clips = d.vdq_lim_clips;
	d.vdq_lim_cycles = 0U;
	d.vdq_lim_clips = 0U;
	/* The FOC ISR can land between the reads/clears: keep clips <= cycles. */
	if (lim_clips > lim_cycles) lim_clips = lim_cycles;
	data[3] = (lim_cycles > 0U) ? (uint8_t)(((uint64_t)lim_clips * 100U) / lim_cycles) : 0U;
#else
	data[3] = (uint8_t)((uint16_t)can_d.can_pack_voltage);
#endif
	data[4] = (uint8_t)((uint16_t)bus_voltage);
	data[5] = (uint8_t)((uint16_t)lv_12v_voltage);
	pack_u16_le(data, 6, (uint16_t)RPM);

	CAN_Queue_Push_And_Kickstart(0x715, (uint32_t)FDCAN_STANDARD_ID, data, 0x08);
}

void Send_on_CAN_716(float vrms, float irms)
{
	uint8_t data[8] = {0};

	float pack_dc = d.pack_current + 500.0f;

	pack_u16_le(data, 2, (uint16_t)pack_dc);
	data[4] = (uint8_t)((uint16_t)((vrms/d.Vdc) * 100.0f));
	data[5] = (uint8_t)((uint16_t)vrms);
	pack_u16_le(data, 6, (uint16_t)irms);

	CAN_Queue_Push_And_Kickstart(0x716, (uint32_t)FDCAN_STANDARD_ID, data, 0x08);
}

void Send_on_CAN_717(float rpm)
{
	uint8_t data[8] = {0};

	volatile float Hz = (rpm * POLEPAIRS * 2.0f) / 120.0f;

	pack_u16_le(data, 0, (uint16_t)Hz);

	// data[2] = (uint8_t)((uint16_t)L_g.Is_ref_woDCLO & 0x00FF);
	// data[3] = (uint8_t)(((uint16_t)L_g.Is_ref_woDCLO & 0xFF00) >> 8);

	// data[4] = (uint8_t)((uint16_t)L_g.Is_ref_wDCLO & 0x00FF);
	// data[5] = (uint8_t)(((uint16_t)L_g.Is_ref_wDCLO & 0xFF00) >> 8);

	CAN_Queue_Push_And_Kickstart(0x717, (uint32_t)FDCAN_STANDARD_ID, data, 0x08);
}

void Send_on_CAN_724(float torque_ref)
{
	uint8_t data[8] = {0};

	float can_vq = FOC_Loop_Y.Vq + 62.0f;
	float can_vd = FOC_Loop_Y.Vd + 62.0f;
	float can_trqRef = fabsf(torque_ref) * 10.0f;

	pack_u16_le(data, 0, (uint16_t)can_trqRef);
	pack_u16_le(data, 2, (uint16_t)can_vq);
	pack_u16_le(data, 4, (uint16_t)can_vd);

	#if CURR_APP1
		data[6] = (volatile uint8_t)0x01;
	#endif

	#if CURR_APP2
		data[6] = (volatile uint8_t)0x02;
	#endif

	data[7] = cS;

	CAN_Queue_Push_And_Kickstart(0x724, (uint32_t)FDCAN_STANDARD_ID, data, 0x08);
}

void Send_on_CAN_726()
{
	uint8_t data[8] = {0};

	data[0] = (uint8_t)(CONFIG_VERSION_MAJOR);
	data[1] = (uint8_t)(CONFIG_VERSION_MINOR);
	data[2] = (uint8_t)(CONFIG_VERSION_SUBMINOR);
	#if HILOAD_XR
	data[3] = 0xE2;
	data[4] = 0x86;
	#endif
	#if HIRANGE_XR
	data[3] = 0xE2;
	data[4] = 0x96;
	#endif
	#if HIRANGE_TR
	data[3] = 0xE2;
	data[4] = 0x76;
	#endif
	data[5] = (uint8_t)(FIRMWARE_VERSION_MAJOR);
	data[6] = (uint8_t)(FIRMWARE_VERSION_MINOR);
	data[7] = (uint8_t)(FIRMWARE_VERSION_SUBMINOR);

	CAN_Queue_Push_And_Kickstart(0x726, (uint32_t)FDCAN_STANDARD_ID, data, 0x08);
}

void Send_on_CAN_7A1(void)
{
	uint8_t can_data[8] = {0};

	can_data[0] = (uint8_t)(PEGASUS_DEVICE_ID);
	can_data[1] = (uint8_t)(FIRMWARE_VERSION_MAJOR);
	can_data[2] = (uint8_t)(FIRMWARE_VERSION_MINOR);
	can_data[3] = (uint8_t)(FIRMWARE_VERSION_SUBMINOR);
	#if ENABLE_DEBUG
	can_data[4] = (volatile uint8_t)(0x01);
	#endif
	#if !ENABLE_DEBUG
	can_data[4] = (volatile uint8_t)(0x00);
	#endif
	can_data[5] = (volatile uint8_t)(CONFIG_VERSION_MAJOR);
	#if CURR_APP1
		can_data[7] = (volatile uint8_t)(0x01);
	#endif
	#if CURR_APP2
		can_data[7] = (volatile uint8_t)(0x02);
	#endif

	CAN_Queue_Push_And_Kickstart(0x7A1, (uint32_t)FDCAN_STANDARD_ID, can_data, 0x08);
}

void Send_on_CAN_7A2(void)
{
	uint8_t can_data[8] = {0};

	can_data[0] = (uint8_t)(PEGASUS_DEVICE_ID);
	can_data[1] = (uint8_t)(HARDWARE_VERSION_MAJOR);
	can_data[2] = (uint8_t)(HARDWARE_VERSION_MINOR);
	can_data[3] = (uint8_t)(HARDWARE_VERSION_SUBMINOR);

	CAN_Queue_Push_And_Kickstart(0x7A2, (uint32_t)FDCAN_STANDARD_ID, can_data, 0x08);
}

void Send_on_CAN_7A3(void)
{
	uint8_t can_data[8] = {0};

	can_data[0] = (uint8_t)(PEGASUS_DEVICE_ID);
	can_data[1] = (uint8_t)(CONFIG_VERSION_MAJOR);
	can_data[2] = (uint8_t)(CONFIG_VERSION_MINOR);
	can_data[3] = (uint8_t)(CONFIG_VERSION_SUBMINOR);

	CAN_Queue_Push_And_Kickstart(0x7A3, (uint32_t)FDCAN_STANDARD_ID, can_data, 0x08);
}

void Send_on_CAN_7B0(void){
	uint8_t can_data[8] = {0};

	can_data[0] = (uint8_t)((image_hdr.git_sha >> 0)  & 0xFF);
	can_data[1] = (uint8_t)((image_hdr.git_sha >> 8)  & 0xFF);
	can_data[2] = (uint8_t)((image_hdr.git_sha >> 16) & 0xFF);
	can_data[3] = (uint8_t)((image_hdr.git_sha >> 24) & 0xFF);
	can_data[4] = (uint8_t)(PEGASUS_DEVICE_ID);

	CAN_Queue_Push_And_Kickstart(0x7B0, (uint32_t)FDCAN_STANDARD_ID, can_data, 0x08);
}

void Send_on_CAN_6F0(void){
	uint8_t can_data[8] = {0};

	can_data[0] = (uint8_t)(PEGASUS_DEVICE_ID);
	can_data[1] = (uint8_t)(HARDWARE_VERSION_MAJOR);
	can_data[2] = (uint8_t)(HARDWARE_VERSION_MINOR);
	can_data[3] = (uint8_t)(HARDWARE_VERSION_SUBMINOR);
	can_data[4] = (uint8_t)(FIRMWARE_VERSION_MAJOR);
	can_data[5] = (uint8_t)(FIRMWARE_VERSION_MINOR);
	can_data[6] = (uint8_t)(FIRMWARE_VERSION_SUBMINOR);
	can_data[7] = (uint8_t)(UID[0]);

	CAN_Queue_Push_And_Kickstart(0x6F0, (uint32_t)FDCAN_STANDARD_ID, can_data, 0x08);
}

void Send_on_CAN_6F1(void){
	uint8_t can_data[8] = {0};

	can_data[0] = (uint8_t)(PEGASUS_DEVICE_ID);
	can_data[1] = (uint8_t)(UID[1]);
	can_data[2] = (uint8_t)(UID[2]);
	can_data[3] = (uint8_t)(UID[3]);
	can_data[4] = (uint8_t)(UID[4]);
	can_data[5] = (uint8_t)(UID[5]);
	can_data[6] = (uint8_t)(UID[6]);
	can_data[7] = (uint8_t)(UID[7]);

	CAN_Queue_Push_And_Kickstart(0x6F1, (uint32_t)FDCAN_STANDARD_ID, can_data, 0x08);
}

void Send_on_CAN_6F2(void){
	uint8_t can_data[8] = {0};

	can_data[0] = (uint8_t)(PEGASUS_DEVICE_ID);
	can_data[1] = (uint8_t)(UID[8]);
	can_data[2] = (uint8_t)(UID[9]);
	can_data[3] = (uint8_t)(UID[10]);
	can_data[4] = (uint8_t)(UID[11]);
	can_data[5] = (uint8_t)(0x9A);
	can_data[6] = (uint8_t)(0xAB);
	can_data[7] = (uint8_t)(0xBC);

	CAN_Queue_Push_And_Kickstart(0x6F2, (uint32_t)FDCAN_STANDARD_ID, can_data, 0x08);
}

void Send_heartbeat(void) {
	Send_on_CAN_7B0();
	Send_on_CAN_6F0();
	Send_on_CAN_6F1();
	Send_on_CAN_6F2();
	Send_on_CAN_7A1();
	Send_on_CAN_7A2();
	Send_on_CAN_7A3();
}
#endif

/* ---- Bench fast telemetry (FOC_BENCH_FAST_TLM, see vehicle_config.h) -------
 * The 10 kHz ISR sums the loop signals; every FAST_TLM_PERIOD_MS the TIM17
 * tick swaps the sums out (ADC ISR is prio 0, so the swap is done with IRQs
 * off), sends their means on ext 0x420/0x421/0x422, and starts a new window.
 * 0x422 = the Vd/Vq feed-forward (fast_tlm_ff, captured in FOC_Loop.c). */
#if FOC_BENCH_FAST_TLM
typedef struct {
	float id_ref, iq_ref, id, iq, vd, vq;	/* 10 kHz sums */
	float vd_ff, vq_ff, vq_ff_unsat, ff_lim;
	uint32_t n;
	float vdc;								/* 1 kHz sum */
	uint32_t n_vdc;
} fast_tlm_acc_t;

static volatile fast_tlm_acc_t fast_tlm_acc;
fast_tlm_ff_t fast_tlm_ff;

static inline uint16_t fast_tlm_i16(float v, float lsb)
{
	float x = v / lsb;
	if (x >  32767.0f) x =  32767.0f;
	if (x < -32767.0f) x = -32767.0f;
	return (uint16_t)(int16_t)lrintf(x);
}

void Fast_Tlm_Accumulate(void)
{
	fast_tlm_acc.id_ref += Current_Ref_Estimator_Y.Id_ref_final;
	fast_tlm_acc.iq_ref += Current_Ref_Estimator_Y.Iq_ref_final;
	fast_tlm_acc.id     += FOC_Loop_Y.Id;
	fast_tlm_acc.iq     += FOC_Loop_Y.Iq;
	fast_tlm_acc.vd     += FOC_Loop_Y.Vd;
	fast_tlm_acc.vq     += FOC_Loop_Y.Vq;
	fast_tlm_acc.vd_ff        += fast_tlm_ff.vd_ff;
	fast_tlm_acc.vq_ff        += fast_tlm_ff.vq_ff;
	fast_tlm_acc.vq_ff_unsat  += fast_tlm_ff.vq_ff_unsat;
	fast_tlm_acc.ff_lim       += fast_tlm_ff.ff_lim;
	fast_tlm_acc.n++;
}

void Fast_Tlm_Tick_1ms(void)
{
	static uint8_t ms = 0;
	uint8_t can_data[8];

	fast_tlm_acc.vdc += d.vdc_raw;	/* only this tick writes vdc/n_vdc */
	fast_tlm_acc.n_vdc++;
	if (++ms < FAST_TLM_PERIOD_MS) return;
	ms = 0;

	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	fast_tlm_acc_t w = fast_tlm_acc;
	fast_tlm_acc = (fast_tlm_acc_t){0};
	__set_PRIMASK(primask);

	if (!can_d.fast_tlm_on || w.n == 0U || w.n_vdc == 0U) return;
	const float k = 1.0f / (float)w.n;

	pack_u16_le(can_data, 0, fast_tlm_i16(w.id_ref * k, 0.01f));
	pack_u16_le(can_data, 2, fast_tlm_i16(w.iq_ref * k, 0.01f));
	pack_u16_le(can_data, 4, fast_tlm_i16(w.id * k, 0.01f));
	pack_u16_le(can_data, 6, fast_tlm_i16(w.iq * k, 0.01f));
	CAN_Queue_Push_And_Kickstart(0x420, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);

	pack_u16_le(can_data, 0, fast_tlm_i16(w.vd * k, 0.01f));
	pack_u16_le(can_data, 2, fast_tlm_i16(w.vq * k, 0.01f));
	pack_u16_le(can_data, 4, fast_tlm_i16(d.RPM, 1.0f));
	pack_u16_le(can_data, 6, fast_tlm_i16(w.vdc / (float)w.n_vdc, 0.01f));
	CAN_Queue_Push_And_Kickstart(0x421, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);

	pack_u16_le(can_data, 0, fast_tlm_i16(w.vd_ff * k, 0.01f));
	pack_u16_le(can_data, 2, fast_tlm_i16(w.vq_ff * k, 0.01f));
	pack_u16_le(can_data, 4, fast_tlm_i16(w.vq_ff_unsat * k, 0.01f));
	pack_u16_le(can_data, 6, fast_tlm_i16(w.ff_lim * k, 0.01f));
	CAN_Queue_Push_And_Kickstart(0x422, (uint32_t)FDCAN_EXTENDED_ID, can_data, 0x08);
}
#endif
