/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* Configuration layers — order matters:
 *   board_config  — hardware/board constants (no deps)
 *   motor profile — motor-specific (depends on math constants from board)
 *   vehicle       — application-level (depends on motor profile for PID limits) */
#include "board_config.h"
#include "motor_profiles/motor_select.h"
#include "vehicle_config.h"

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"
#include "isr_handlers.h"
#include "can_rx_handlers.h"
#include "../Libs/Include/Autotune101.h"
#include "Speed_Sensing.h"
#include "NTC_Temp_Reading.h"
#if PROTECTION_MODEL
#include "MCU_Protections.h"
#endif
// #include "FOC_MTPA_FWC_FF.h"
#include "FOC_Loop.h"
#include "Current_Ref_Estimator.h"
#include "Open_FOC0.h"
#include "fdcan.h"
#include "Rate_Limiter.h"
#include "Eeprom.h"
#include "HelperFuncs.h"
#include "Sensor_Disconnection_Checks.h"
#include "Limit_Errors.h"
#include "Helper_Functions.h"
#include "Peripheral_Init.h"
#include "Torque_Management.h"
#include "Hill_Hold.h"
#if THROTTLE_BASED_REF
#include "throttle_map.h"
#endif
#include "tim17_handlers.h"

/* OTA / bootloader support (OTA_Reintegrated) */
#include "bootloaderFunctions.h"
#include "firmware_upgrade_app.h"
#include "enums.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* Shared drive state. Single global instance `d` (declared at the bottom of
 * this header, defined in main.c) acts as the blackboard between the 10 kHz
 * ADC ISR, the 1 kHz TIM17 tick, the CAN Rx callbacks and the superloop.
 * Units are noted per field — they are NOT uniform (angles are radians but
 * EEPROM copies are milli-radians; Vdc is a whole-volt integer). */
typedef struct data_t {
	/* --- Status / mode flags (0 or 1 unless noted) --- */
	uint8_t ctlr_derate_flag;      /* controller thermal derate active      */
	uint8_t motor_derate_flag;     /* motor thermal derate active           */
	uint8_t throttle_percent;      /* pedal position, 0..100 %              */
	uint8_t regen_flag;            /* regen braking active (CAN 0x706 bit)  */
	uint8_t regen_active;          /* regen permitted by supervisor         */
	uint8_t canBus_ok;             /* 0 while recovering from CAN BUS-OFF   */

	/* --- Encoder signal health (raw pin states, see Sensor_Disconnection_Checks.c) --- */
	uint8_t z_pulse;               /* Z index toggle, flipped in EXTI       */
	uint8_t A_Pulse;               /* encoder A level                       */
	uint8_t B_Pulse;               /* encoder B level                       */
	uint8_t A_B_XOR;               /* A^B — both-stuck detector             */

	/* --- Startup / alignment sequencing --- */
	uint8_t motor_start;           /* latched once drive has spun up        */
	uint8_t start_alignment;       /* 1 = angle calibration required        */
	uint8_t end_alignment;         /* 1 = angle calibration complete        */
	uint8_t init_check;            /* HAL_OK / !HAL_OK from Initial_Fault_Check */
	uint8_t Open_FOC;              /* autotune running open-loop model      */
	uint8_t Closed_FOC;            /* autotune running closed-loop model    */

	/* --- Raw switch inputs (GPIO_PinState; active-low with pull-ups) --- */
	uint8_t power_pin;             /* PD2  LOW = SPORTS, HIGH = ECO         */
	uint8_t forward_pin;           /* PD3  LOW = FORWARD selected           */
	uint8_t reverse_pin;           /* PD4  LOW = REVERSE selected           */

	/* --- Measured analogue values --- */
	uint8_t Vdc;                   /* DC bus voltage, V (whole volts)       */
	float vdc_filt;                /* low-passed DC bus, V (1 kHz, FOC use) */
	float vdc_raw;                 /* unfiltered DC bus, V (1 kHz sample)   */
	uint8_t Aux_dc;                /* 12 V aux rail — sense removed, always 0 */
	int16_t Mtr_temp;              /* motor temperature — not sensed, always 0 */
	int16_t Mtc_temp;              /* controller temperature = max(U,V,W NTC), degC */
	uint16_t mtc_analog_val;       /* unused (kept for layout)              */
	uint16_t mtr_analog_val;       /* unused (kept for layout)              */
	int16_t temp_u, temp_v, temp_w;          /* U/V/W NTC, degC             */
	uint16_t temp_u_analog, temp_v_analog, temp_w_analog; /* raw ADC counts */
	float pack_current;            /* battery pack current, A (from BMS)    */
	float vrms;                    /* phase voltage, V RMS                  */
	float irms;                    /* phase current, A RMS                  */

	/* --- Inverter outputs --- */
	uint16_t pwm_a;                /* TIM1 CH1 compare, counts              */
	uint16_t pwm_b;                /* TIM1 CH2 compare, counts              */
	uint16_t pwm_c;                /* TIM1 CH3 compare, counts              */
	float Vmax_SVM;                /* SVM clamp limit, V                    */
	float Va_SVM;                  /* SVM phase A output, V                 */
	float Vb_SVM;                  /* SVM phase B output, V                 */
	float Vc_SVM;                  /* SVM phase C output, V                 */

	/* --- Commutation offsets. cw/ccw are per rotation direction and are
	 * selected by Throttle_State (electrical frame) in isr_handlers.c; avg
	 * is used in NEUTRAL. The *_16bit pair is the EEPROM representation. --- */
	uint16_t offset_angle_mech_cw_16bit;   /* EEPROM copy, milli-rad mech   */
	uint16_t offset_angle_mech_ccw_16bit;  /* EEPROM copy, milli-rad mech   */
	float offset_angle_mech_cw;    /* CW  commutation offset, rad mech      */
	float offset_angle_mech_ccw;   /* CCW commutation offset, rad mech      */
	float offset_angle_mech_avg;   /* (cw+ccw)/2, used in NEUTRAL, rad mech */
	float ATA_Offset;              /* offset in use during autotune, rad mech */

	/* --- Encoder position (TIM2 counts unless noted) --- */
	uint32_t encoder_count;        /* live TIM2 counter                     */
	uint32_t Count_From_Duty;      /* absolute count seeded from PWM duty   */
	uint32_t count_at_z;           /* TIM2 count at last Z index            */
	uint32_t count_diff_at_z;      /* drift observed at Z                   */
	uint32_t z_count;              /* Z pulses seen                         */
	uint32_t prev_z_count;         /* previous Z sample, for delta          */
	uint32_t curr_z_count;         /* current  Z sample, for delta          */
	uint32_t z_count_diff;         /* counts between consecutive Z pulses   */

	/* --- Absolute-angle PWM input (TIM5 input capture) --- */
	float ICvalue;                 /* captured period, timer counts         */
	float Duty;                    /* measured duty, 0..100 %               */
	float Frequency;               /* measured PWM frequency, Hz            */
	float Angle_From_Duty;         /* angle decoded from duty, rad mech     */
	float abs_Angle_pwm;           /* wrapped absolute angle, rad mech      */

	/* --- Derived angles and speed --- */
	float mech_angle;              /* mechanical angle, rad [0, 2pi)        */
	float elec_angle;              /* electrical angle, rad [0, 2pi)        */
	float omega_e;                 /* electrical speed, rad/s               */
	float raw_rad_s;               /* unfiltered mech speed, rad/s          */
	float rad_s;                   /* filtered mech speed, rad/s            */
	float RPM;                     /* mechanical speed, signed RPM          */

	/* --- References / commands --- */
	float speed_ref;               /* speed command, RPM                    */
	float torque_ref;              /* torque command from CAN, Nm           */
	float torque_final;            /* torque after all scaling/derates, Nm  */
	float throttle_v;              /* throttle input, V                     */
	uint32_t thr_v_mv;             /* throttle input, mV (throttle_map in)  */

	/* --- Housekeeping --- */
	float Op_ts;                   /* model sample time, s (1e-4 = 10 kHz)  */
	uint32_t cycles;               /* FOC ISR duration, DWT CPU cycles      */

	/* --- Voltage-headroom diagnostic (CAN 0x715 byte[1]) ---
	 * |V| of the commanded dq pair over the achievable SVM circle: 1.0 = on
	 * the circle, >1.0 = the pair is outside it. The per-axis clamps are
	 * +/-Vmax EACH, so a pair can pass them both and still be unreachable;
	 * apply_SVM_voltages() then truncates it and the PI anti-windup cannot
	 * see that truncation. PEAK-held: the CAN frame rate is ~3 orders of
	 * magnitude below the 10 kHz FOC loop, so an instantaneous sample would
	 * alias straight past the transients this exists to catch.
	 * Written by the FOC ISR; read-and-cleared by Send_on_CAN_715 so each
	 * frame reports its own window. A sample landing between that read and
	 * the clear is lost -- acceptable for a peak-hold at 10 kHz. The
	 * vdq_lim_* counters below follow the same read-and-clear rule.
	 * With FOC_VDQ_CIRCLE_LIMIT the pair is limited to the circle inside the
	 * model, so this is sampled from vdq_demand (the pre-limit pair) and
	 * still reports the DEMANDED index, comparable with older logs. */
	float mod_index_peak;          /* peak |V|/Vmax_SVM since last CAN read */
	float vdq_demand;              /* |Vd,Vq| before the circle limit, V    */
	uint32_t vdq_lim_cycles;       /* FOC steps since last 0x715 (clip %)   */
	uint32_t vdq_lim_clips;        /* ...of which the circle limit acted    */

	float Max_ph_current;
	float Min_DC_V;
	float Max_DC_V;
} data_t;

typedef struct foc_pid_t {
	float Kp;
	float Ki;
	float Kd;
	float Kd_Filter;
	float Output_Up_Limit;
	float Output_Low_Limit;
	float Back_Kaw;
} foc_pid_t;

typedef struct foc_fw_t {
	float Kaw;
	float Kfw;
	float M_to_F;
	float F_to_M;
	float Rst_Factor;
} foc_fw_t;

typedef struct can_data_t {
	uint8_t direction;
	uint8_t power_mode;
	uint8_t reset_flag;
	uint8_t autotune_request;
	uint8_t brake_command_active;
	uint8_t brake_command_reply;
	float can_Ld;
	float can_Lq;
	float can_Lambda;
	float can_Rs;
	float can_instV;
	float can_battCurr;
	float can_dclo;
	float can_dcli;
	uint32_t t_rx_0x110_ms;   /* HAL tick of last 0x110 Rx (battery current) */
	uint32_t t_rx_0x12A_ms;   /* HAL tick of last 0x12A Rx (DCLI/DCLO)       */
	float can_soc;            /* Pack SOC, per-unit [0.0, 1.0] */
	float can_pack_voltage;   /* Pack voltage from BMS (V)     */
	float isense_delay_s;     /* current-sense delay comp, s (0x115) */
	uint8_t fast_tlm_on;      /* bench 200 Hz telemetry enable (0x116) */
	foc_fw_t canFW_params;
	foc_pid_t canSpeed_PID;
	foc_pid_t canId_PID;
	foc_pid_t canIq_PID;
} can_data_t;

typedef struct errors {
	uint8_t error_c1;
	uint8_t error_c2;
	uint8_t error_c3;
	char curr_sens_error;
	char bus_voltage_ov_error;
	char bus_voltage_uv_error;
	char aux_voltage_ov_error;
	char aux_voltage_uv_error;
	char id_iq_oc_error;
	char phase_curr_error;
	char stall_protection;
	char overspeed_error;
	char mtr_temp_ot_error;
	char mtc_temp_ot_error;
	char mtr_temp_disconnection_error;
	char mtc_temp_disconnection_error;
	char eeprom_write_error;
	char eeprom_read_error;
	char a_b_error;
	char z_error;
	char pwm_error;
	char enc_5v_error;
	char error_triggered;
	char drive_off;
} errors;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* Board pin map (hand-maintained; the CubeMX .ioc carries no user labels).
 * FNR switch inputs are active-low with pull-ups (gpio.c). */
#define FNR_FWD_GPIO_Port       GPIOD
#define FNR_FWD_Pin             GPIO_PIN_3   /* LOW = FORWARD selected */
#define FNR_REV_GPIO_Port       GPIOD
#define FNR_REV_Pin             GPIO_PIN_4   /* LOW = REVERSE selected */
#define PWR_MODE_GPIO_Port      GPIOD
#define PWR_MODE_Pin            GPIO_PIN_2   /* LOW = SPORTS; HIGH (pull-up default) = ECO */

/* PD15 — drive-inhibit / fault indicator. Contract at every write site:
 * SET = drive inhibited (boot, fault latch, reset path), RESET = drive
 * running (FOC or angle-calib active). */
#define DRIVE_INHIBIT_GPIO_Port GPIOD
#define DRIVE_INHIBIT_Pin       GPIO_PIN_15

/* PD10..PD12 — status LEDs, lit together once inverter PWM starts (main.c) */
#define STATUS_LED_GPIO_Port    GPIOD
#define STATUS_LED_PINS         (GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12)

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* ---------------------------------------------------------------------------
 * SHARED-STATE / CONCURRENCY MAP
 *
 * d, er and can_d are plain (NON-volatile) globals shared between interrupt
 * contexts and the background superloop; cS, fnr_state and pw_state are
 * volatile (app_types.h). Nothing is lock-protected and interrupts are never
 * masked around these accesses — correctness rests entirely on the ownership
 * rules below. Check them before adding a write from a new context.
 *
 * Contexts, highest NVIC priority first (lower number preempts higher):
 *
 *  prio 0  ADC_IRQn @10 kHz — HAL_ADCEx_InjectedConvCpltCallback (main.c)
 *          dispatches on cS to isr_curr_sens_calib / isr_angle_calib /
 *          isr_foc_start (isr_handlers.c); those reach rt_OneStep ->
 *          Speed_Sense (Speed_Sensing.c), HillHold_Apply (Hill_Hold.c) and
 *          Initial_Fault_Check (Sensor_Disconnection_Checks.c).
 *          Writes: FOC_Loop_U phase currents, mech_angle, elec_angle,
 *          encoder_count, V*_SVM, pwm_a/b/c, raw_rad_s, rad_s, RPM, omega_e,
 *          regen_flag, motor_start, offset_angle_mech_cw/ccw, ATA_Offset,
 *          cycles, init_check, Open_FOC, Closed_FOC, start/end_alignment,
 *          er.* (via Initial_Fault_Check only), cS.
 *  prio 1  TIM2 — encoder counter; its IRQ writes no shared state.
 *  prio 2  TIM17_IRQn @1 kHz — tim17_1ms_tick (tim17_handlers.c):
 *          Sensor_Disconnection_Check + Error_Check (er.*, A_Pulse, B_Pulse,
 *          cS), handle_offset_store (eeprom_*_pending, offset_angle_mech_avg),
 *          queue_can_messages (canBus_ok; otherwise reads d/er and transmits).
 *          See invariant 6.
 *  prio 3  EXTI9_5 — Z index, HAL_GPIO_EXTI_Callback (main.c):
 *          z_pulse, z_count, count_at_z, prev_z_count.
 *  prio 4  TIM5 — encoder PWM capture, HAL_TIM_IC_CaptureCallback (main.c):
 *          ICvalue, Duty, Frequency, abs_Angle_pwm.
 *  prio 5  DMA1_Stream0 — ADC1 circular DMA (priority set in dma.c). The ISR
 *          itself writes no shared state: stm32h7xx_it.c only calls
 *          HAL_DMA_IRQHandler and no HAL_ADC_ConvCpltCallback is defined
 *          anywhere. The DMA engine is the real writer here and it needs no
 *          CPU context at all — see invariant 5.
 *  prio 6  FDCAN2_IT0/IT1 — Rx, HAL_FDCAN_RxFifo0Callback (main.c) and
 *          can_rx_dispatch (can_rx_handlers.c): can_d.*, and under
 *          CAN_BASED_REF also speed_ref / torque_ref.
 *  prio 7  TIM4 — one-shot 0x7Ax heartbeat; transmits only.
 *  (none)  Superloop, main() while(1) — lowest, preempted by all of the
 *          above: ADC1_Analog_Val_Update (Vdc, Aux_dc, Mtr/Mtc_temp,
 *          thr_v_mv, throttle_v, throttle_percent, vrms, irms), throttle ->
 *          torque path (speed_ref under THROTTLE_BASED_REF, torque_final),
 *          derate flags (Torque_Management.c), EEPROM offsets (Eeprom.c),
 *          power_mode_fnr_switch (fnr_state, pw_state, *_pin), cS, er.*.
 *
 * NOTE: Sensor_Disconnection_Checks.c spans three contexts — its
 * Initial_Fault_Check runs in the prio-0 ADC ISR, Sensor_Disconnection_Check
 * in the prio-2 TIM17 ISR, and ADC1_Analog_Val_Update in the superloop. The
 * file name does not tell you which; check the caller.
 *
 * Load-bearing invariants that the code does NOT enforce:
 *
 *  1. er.error_cN is updated with read-modify-write (|=) from two different
 *     priorities: Initial_Fault_Check (prio 0) and Error_Check /
 *     Sensor_Disconnection_Check (prio 2). A lost update is avoided only
 *     because the two are mutually exclusive by state — Initial_Fault_Check
 *     runs solely while cS == CURR_SENS_CALIB, the others only while
 *     cS == FOC_START or ANGLE_CALIB. Adding an er.* bit-set anywhere in the
 *     running FOC path would create a genuine lost-update race.
 *  2. speed_ref / torque_ref have a single writer only because CAN_BASED_REF
 *     and THROTTLE_BASED_REF are mutually exclusive build flags. Enabling
 *     both would give the CAN Rx ISR and the superloop the same target.
 *  3. d is not volatile, so the superloop may cache ISR-written fields across
 *     an iteration. Each field is <= 32 bits so individual reads do not tear,
 *     but several related fields read in the superloop are NOT a coherent
 *     snapshot of one ISR tick.
 *  4. The one handoff done explicitly is the deferred EEPROM write: the
 *     volatile eeprom_*_pending flags (tim17_handlers.c) are set in the TIM17
 *     ISR and consumed by the superloop, which performs the blocking I2C
 *     write. Follow that pattern for any new ISR -> superloop work item.
 *  5. adc1_buffer[5] (main.c) is the ADC1 DMA target, started once at boot
 *     (HAL_ADC_Start_DMA) in DMA_CIRCULAR mode. Hardware therefore rewrites
 *     it continuously and asynchronously to every context above, with no
 *     interrupt marking the update. Two consequences:
 *       - It is declared volatile in main.c, and the extern in
 *         Sensor_Disconnection_Checks.c carries the qualifier too. That is
 *         load-bearing rather than decorative: the build enables -flto, so
 *         both are visible to a single optimiser that cannot see the DMA
 *         writes. Dropping volatile from either would let reads be cached.
 *       - Independently of volatile, the five entries are never a coherent
 *         single-sweep snapshot: a DMA burst can land between two element
 *         reads, so Vdc and throttle_v may come from different sweeps.
 *     There is no cache-coherency hazard: main.c enables I-cache only
 *     (SCB_EnableICache), and D-cache is left disabled at reset. Enabling
 *     D-cache later would make this buffer a genuine stale-read bug unless
 *     it is placed in a non-cacheable region or invalidated before each read.
 *  6. The speed estimate (raw_rad_s, rad_s, RPM) and Speed_Sense / Speed_Filter's
 *     static derivative+filter history have a SINGLE writer: the prio-0 ADC ISR
 *     via rt_OneStep's subrate (ANGLE_CALIB / FOC_START), feeding at 1 kHz as the
 *     DERIVATIVE_CONSTANT and SPEED_FILTER_CONSTANT_* (T_S = 0.001) assume.
 *     A second CONT_ERROR-time caller (update_speed_while_faulted, prio-2 TIM17)
 *     was REMOVED 2026-07-29: sharing that static history across two contexts and
 *     cadences corrupted the derivative and spiked rad_s (clamped at +/-733 rad/s),
 *     driving the live FOC into field-weakening voltage saturation. Do NOT add a
 *     second caller of Speed_Sense / Speed_Filter. If faulted-speed telemetry is
 *     ever wanted again, compute it from independent local state and write d.RPM
 *     only — never rad_s (a FOC input) and never those shared statics.
 * ------------------------------------------------------------------------- */

extern data_t d;
extern can_data_t can_d;
extern errors er;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
