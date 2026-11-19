#include "can_rx_handlers.h"

/* -----------------------------------------------------------------------
 * Per-message-ID handlers (all static — private to this translation unit).
 * Add a new handler + one table entry below to support a new CAN message.
 * ----------------------------------------------------------------------- */

static void can_rx_0x102(const uint8_t *msg) {
  if (FOC_Loop_U.Speed_1_Torque_0 == true) {
    d.speed_ref = (float)((msg[1] << 8) | msg[0]);
  } else {
    d.torque_ref = (float)((msg[1] << 8) | msg[0]);
  }
}

static void can_rx_0x103(const uint8_t *msg) {
  can_d.can_Lambda = (float)((msg[1] << 8) | msg[0]) * 1.0E-5f;
  can_d.can_Ld     = (float)((msg[3] << 8) | msg[2]) * 1.0E-8f;
  can_d.can_Lq     = (float)((msg[5] << 8) | msg[4]) * 1.0E-7f;
  can_d.can_Rs     = (float)((msg[7] << 8) | msg[6]) * 1.0E-6f;
  Current_Ref_Estimator_U.Motor_Parameters.Lambda = can_d.can_Lambda;
  Current_Ref_Estimator_U.Motor_Parameters.Ld     = can_d.can_Ld;
  Current_Ref_Estimator_U.Motor_Parameters.Lq     = can_d.can_Lq;
  Current_Ref_Estimator_U.Motor_Parameters.Rs     = can_d.can_Rs;
}

static void can_rx_0x104(const uint8_t *msg) {
  can_d.canFW_params.Kfw = (float)((msg[1] << 8) | msg[0]) * 1.0E-3f;
  can_d.canFW_params.Kaw = (float)((msg[3] << 8) | msg[2]) * 1.0E-3f;
  can_d.canFW_params.M_to_F = (float)(msg[4] * 0.01f);
  can_d.canFW_params.F_to_M = (float)(msg[5] * 0.01f);
  can_d.canFW_params.Rst_Factor = (float)(msg[6] * 0.01f);
  Current_Ref_Estimator_U.FWC_values.Kfw = can_d.canFW_params.Kfw;
  Current_Ref_Estimator_U.FWC_values.Kaw = can_d.canFW_params.Kaw;
  Current_Ref_Estimator_U.FWC_values.M2F_factor = can_d.canFW_params.M_to_F;
  Current_Ref_Estimator_U.FWC_values.F2M_factor = can_d.canFW_params.F_to_M;
  Current_Ref_Estimator_U.FWC_values.FW_Int_reset_factor = can_d.canFW_params.Rst_Factor;
}

static void can_rx_0x105(const uint8_t *msg) {
  can_d.canSpeed_PID.Kp = (float)((msg[1] << 8) | msg[0]) * 1.0E-2f;
  can_d.canSpeed_PID.Ki = (float)((msg[3] << 8) | msg[2]) * 1.0E-2f;
  can_d.canSpeed_PID.Kd = (float)((msg[5] << 8) | msg[4]) * 1.0E-5f;
  FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Kp_speed_PID_MTPA = can_d.canSpeed_PID.Kp;
  FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Ki_speed_PID_MTPA = can_d.canSpeed_PID.Ki;
  FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Kd_speed_PID_MTPA = can_d.canSpeed_PID.Kd;
  FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Kp_speed_PID_MTPA_low = can_d.canSpeed_PID.Kp * LOW_SPEED_SCALING;
  FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Ki_speed_PID_MTPA_low = can_d.canSpeed_PID.Ki * LOW_SPEED_SCALING;
  FOC_Loop_U.MTPA_PID.Speed_PID_MTPA.Kd_speed_PID_MTPA_low = can_d.canSpeed_PID.Kd * LOW_SPEED_SCALING;
}

static void can_rx_0x106(const uint8_t *msg) {
  can_d.canId_PID.Kp = (float)((msg[1] << 8) | msg[0]) * 1.0E-2f;
  can_d.canId_PID.Ki = (float)((msg[3] << 8) | msg[2]) * 1.0E-2f;
  can_d.canId_PID.Kd = (float)((msg[5] << 8) | msg[4]) * 1.0E-5f;
  FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Kp_flux_PID_MTPA = can_d.canId_PID.Kp;
  FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Ki_flux_PID_MTPA = can_d.canId_PID.Ki;
  FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Kd_flux_PID_MTPA = can_d.canId_PID.Kd;
  FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Kp_flux_PID_MTPA_low = can_d.canId_PID.Kp * LOW_SPEED_SCALING;
  FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Ki_flux_PID_MTPA_low = can_d.canId_PID.Ki * LOW_SPEED_SCALING;
  FOC_Loop_U.MTPA_PID.Flux_PID_MTPA.Kd_flux_PID_MTPA_low = can_d.canId_PID.Kd * LOW_SPEED_SCALING;
}

static void can_rx_0x107(const uint8_t *msg) {
  can_d.canIq_PID.Kp = (float)((msg[1] << 8) | msg[0]) * 1.0E-2f;
  can_d.canIq_PID.Ki = (float)((msg[3] << 8) | msg[2]) * 1.0E-2f;
  can_d.canIq_PID.Kd = (float)((msg[5] << 8) | msg[4]) * 1.0E-5f;
  FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Kp_torque_PID_MTPA = can_d.canIq_PID.Kp;
  FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Ki_torque_PID_MTPA = can_d.canIq_PID.Ki;
  FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Kd_torque_PID_MTPA = can_d.canIq_PID.Kd;
  FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Kp_torque_PID_MTPA_low = can_d.canIq_PID.Kp * LOW_SPEED_SCALING;
  FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Ki_torque_PID_MTPA_low = can_d.canIq_PID.Ki * LOW_SPEED_SCALING;
  FOC_Loop_U.MTPA_PID.Torque_PID_MTPA.Kd_torque_PID_MTPA_low = can_d.canIq_PID.Kd * LOW_SPEED_SCALING;
}

static void can_rx_0x108(const uint8_t *msg) {
#if CAN_SPEED_ONLY
  (void)msg;   /* direction fixed to Forward in main.c */
#else
  can_d.direction = msg[0];
#endif
}
static void can_rx_0x109(const uint8_t *msg) { can_d.power_mode = msg[0]; }

#if CAN_BASED_THRV
static void can_rx_0x110(const uint8_t *msg) {
#if CAN_SPEED_ONLY
  (void)msg;   /* throttle voltage fixed to THR_MIN_VAL in main.c */
#else
  can_d.can_instV = (float)((msg[1] << 8) | msg[0]) * 0.001f;
  FOC_Loop_U.Throttle_input.Throttle_Inst_Voltage = can_d.can_instV;
#endif
}
#endif

#if FOC_ISENSE_DELAY_COMP
/* Current-sense delay comp tau, u16 LE microseconds (FOC_Loop.c Park). */
static void can_rx_0x115(const uint8_t *msg) {
  float tau_s = (float)((msg[1] << 8) | msg[0]) * 1.0E-6f;
  can_d.isense_delay_s = fminf(tau_s, FOC_ISENSE_DELAY_MAX_S);
}
#endif

#if FOC_BENCH_FAST_TLM
/* Bench fast telemetry on/off, byte0 = 1 / 0 (fdcan_AL.c 0x420/0x421). */
static void can_rx_0x116(const uint8_t *msg) {
  can_d.fast_tlm_on = (msg[0] == 1U) ? 1U : 0U;
}
#endif

static void can_rx_0x111(const uint8_t *msg) {
  can_d.reset_flag = msg[0];
  if (can_d.reset_flag == (uint8_t)1 && fabsf(d.RPM) <= 10.0f) {
    d.speed_ref = FOC_Loop_U.Ref_Speed_mech_rpm = 0.0f;
    fnr_state = NEUTRAL;
    pw_state  = ECO;
    cS        = RESET_STATE;
  }
}

void can_rx_0x729(const uint8_t *msg) {
  if (msg[0] == (uint8_t)1 && fabsf(d.RPM) <= 10.0f) {
    can_d.autotune_request = 1;
  }
}

/* -----------------------------------------------------------------------
 * Dispatch table — maps each CAN ID to its handler.
 * To add a new message: write a handler above, add one row here.
 * ----------------------------------------------------------------------- */
typedef void (*can_handler_t)(const uint8_t *msg);
typedef struct { uint32_t id; can_handler_t fn; } can_dispatch_t;

static const can_dispatch_t can_dispatch_table[] = {
  { 0x102, can_rx_0x102 },
  { 0x103, can_rx_0x103 },
  { 0x104, can_rx_0x104 },
  { 0x105, can_rx_0x105 },
  { 0x106, can_rx_0x106 },
  { 0x107, can_rx_0x107 },
  { 0x108, can_rx_0x108 },
  { 0x113, can_rx_0x109 },  /* power mode (0x109 is the BMS frame in VH mode) */
#if CAN_BASED_THRV
  { 0x114, can_rx_0x110 },  /* throttle voltage mV (0x110 is batt current in VH mode) */
#endif
  { 0x111, can_rx_0x111 },
#if FOC_ISENSE_DELAY_COMP
  { 0x115, can_rx_0x115 },
#endif
#if FOC_BENCH_FAST_TLM
  { 0x116, can_rx_0x116 },
#endif
  { 0x729, can_rx_0x729 },
};
#define CAN_DISPATCH_SIZE (sizeof(can_dispatch_table) / sizeof(can_dispatch_table[0]))

/* Public entry point — called from HAL_FDCAN_RxFifo0Callback in main.c */
void can_rx_dispatch(uint32_t id, const uint8_t *msg)
{
  for (uint8_t i = 0; i < CAN_DISPATCH_SIZE; i++) {
    if (id == can_dispatch_table[i].id) {
      can_dispatch_table[i].fn(msg);
      break;
    }
  }
}
