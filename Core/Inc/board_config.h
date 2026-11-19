#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

/* Controller/firmware identity */
#define PEGASUS_DEVICE_ID			0x03
#define HARDWARE_VERSION_MAJOR		0x04
#define HARDWARE_VERSION_MINOR		0x03
#define HARDWARE_VERSION_SUBMINOR	0x01

/* Vdc-sense voltage divider — HARDWARE per controller board, NOT per vehicle
 * variant (independent of HIRANGE_XR / HILOAD_XR / HIRANGE_TR). Some controllers
 * carry the old divider, some the new; set exactly one of these to 1 to match
 * THIS unit's board. It selects BUS_VDC_SCALE below, which scales the locally
 * sensed DC-link voltage d.Vdc. d.Vdc drives the bus OV/UV protection trips
 * (Limit_Errors.c, Sensor_Disconnection_Checks.c) and the 0x706/0x715 Vdc
 * telemetry — it does NOT feed the FOC voltage model (that runs off the BMS pack
 * voltage). So a MISMATCH mis-calibrates the OV/UV trips: the old scale on a
 * new-divider board under-reads ~23%, leaving overvoltage protection dangerously
 * lenient (and undervoltage nuisance-tripping); the reverse trips OV early.
 * Match the flag to the board at flash time. */
#define OLD_VOLT_DIV		0
#define NEW_VOLT_DIV		1
#if (OLD_VOLT_DIV + NEW_VOLT_DIV) != 1
#error "board_config.h: set exactly one of OLD_VOLT_DIV / NEW_VOLT_DIV to match the controller's Vdc-sense divider"
#endif

/* CURR_APP1 / CURR_APP2 are supplied per build target by the build system
 * (-DCURR_APP1 for App1, -DCURR_APP2 for App2) so exactly one is defined.
 * Do NOT define them here — doing so makes both #ifdef CURR_APP1/CURR_APP2
 * branches active and breaks per-app identity (and redefines the -D value). */
// #define CURR_APP1					1
// #define CURR_APP2					2

/* Math constants */
#define TWO_PI					6.283185f
#define TWO_ROOT2				2.828427f
#define ROOT2					1.414213f
#define	ROOT3					1.732051f
#define ONE_BY_ROOT2			0.70710678f
#define DEG_TWO_PI_3			2.094395f
#define DEG_4_PI_3				4.188790f
#define RAD_S_TO_RPM   			9.549296f

/* Timer configuration (board/MCU level)
 * TIM1 runs at two prescalers: CALIB is the init rate used while averaging
 * current-sensor offsets; RUN is the operational FOC PWM rate. The switch
 * happens in isr_curr_sens_calib() once NO_OF_SAMPLES have been collected. */
#define TIM1_PSC_CALIB		9		/* ~2 kHz PWM  @ 100 MHz timer clock, ARR=2499, centre-aligned */
#define TIM1_PSC_RUN		1		/* ~10 kHz PWM @ 100 MHz timer clock, ARR=2499, centre-aligned */
#define TIM1_ARR			2499
#define TIM1_ARR_HALF		1250.0f
#define TIM1_DEAD_TIME		100
#define TIM5_PSC			99
#define TIM5_ARR			0xFFFFFFFF
#define TIM17_PSC			39
#define TIM17_ARR			2499
#define HIGH_PULSE16_ERROR	0.024574f

/* ADC / sampling */
#define ADC_TO_V			0.00005030822f	/* V per 16-bit count (Vref ~3.297 V) */
/* Current sensor: ACS758LCB-050B-PFF-T, +/-50 A bidirectional, powered from 3.3 V.
 * Ratiometric: zero-current output = Vcc/2 = 1.65 V (~32768 counts; the real zero is
 * still measured at boot by the offset calibration), sensitivity = 60 mV/A * 3.3/5
 * = 39.6 mV/A. A per count = ADC_TO_V / 0.0396. Linear range is about +/-41.7 A
 * (output clips at the 0 V / 3.3 V rails). */
#define ADC_TO_CURR			(ADC_TO_V / 0.0396f)	/* = 0.001270 A/count */
#define NO_OF_SAMPLES 		4096
/* DC-link voltage scale, selected by OLD_VOLT_DIV / NEW_VOLT_DIV above (exactly
 * one is set — enforced by the guard there). Must match the controller's fitted
 * divider or the bus OV/UV trips read wrong (see the macro comment). */
#if OLD_VOLT_DIV
#define BUS_VDC_SCALE		0.002675f	/* old Vdc-sense divider */
#endif
#if NEW_VOLT_DIV
#define BUS_VDC_SCALE		0.00374597f	/* new Vdc-sense divider */
#endif
#define AUX_VDC_SCALE		0.000188658f

/* Aux rail */
#define AUX_OP_VOLTAGE		12.0f
#define AUX_UV_LIMIT		9

/* Controller NTC (motor NTC lives in motor profile) */
#define MTC_NTC_R25			10000.0f

/* EEPROM */
#define EEPROM_ADDRESS			0xA0
#define EEPROM_MAGIC_NUM		0x21
#define EEPROM_NO_MAGIC_NUM		2
#define EEPROM_PAGE_SIZE		8
#define EEPROM_WRITE_TIMEOUT	5

/* CAN transmit scheduling */
#define CAN_BUS_CYCLE		100
#define CAN_TX_QUEUE_SIZE	32
#define FRAME_SLOT_MS		2U
#define TICKS_PER_SLOT		4U

/* Speed sensing (filter constants derived from sample time) */
#define T_S						0.001f
#define DERIVATIVE_CONSTANT		(1.0f / T_S)
#define T_F_ROTOR_SPEED			0.03f
#define SPEED_FILTER_CONSTANT_1	((2.0f * T_F_ROTOR_SPEED - T_S) / (2.0f * T_F_ROTOR_SPEED + T_S))
#define SPEED_FILTER_CONSTANT_2	(T_S / (2.0f * T_F_ROTOR_SPEED + T_S))

/* Controller-scope protection limits */
#define MAX_PHASE_CURRENT		IQ_UP_LIMIT
#define CONTRL_TEMP_OT_LIMIT	60	/* trips if ANY of the U/V/W NTCs reaches this */
#define PHASE_OC_COUNT			5
#define ID_IQ_OC_COUNT			5
#define STALL_COUNT             6000
#define OVERSPEED_COUNT			10		/* 1 ms checks, see OVERSPEED_TRIP_RPM */
#define BUS_DC_OV_COUNT			1
#define BUS_DC_UV_COUNT			1
#define CURR_SEN_ANLG_HIGH		40000
#define CURR_SEN_ANLG_LOW		14000
#define TEMP_SENS_FAULT_COUNT	65000
#define ENCODER_CHECK_MS		20
#define ENCODER_FAULT_MAX_COUNT	5
#define AUTOTUNE_UPDATE_MS		10

#endif /* __BOARD_CONFIG_H */
