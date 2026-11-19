#ifndef __APP_TYPES_H_
#define __APP_TYPES_H_

typedef enum errors_nums {
	NO_ERROR,
	CURR_SENS_ERROR,
	BUS_VOLTAGE_OV_ERROR,
	BUS_VOLTAGE_UV_ERROR,
	AUX_VOLTAGE_OV_ERROR,
	AUX_VOLTAGE_UV_ERROR,
	ID_IQ_OC_ERROR,
	PHASE_CURR_ERROR,
	MTR_TEMP_OT_ERROR,
	MTC_TEMP_OT_ERROR,
	MTR_TEMP_DISCONNECTION_ERROR,
	MTC_TEMP_DISCONNECTION_ERROR,
	EEPROM_WRITE_ERROR,
	EEPROM_READ_ERROR,
	ENCODER_A_B_ERROR,
	ENCODER_Z_ERROR,
	ENCODER_PWM_ERROR,
	ENCODER_5V_ERROR,
	STALL_PROTECTION,
	OVERSPEED_ERROR = 20
} errors_nums;

typedef enum currSession {
	INIT,
	ANGLE_CALIB,
	ANGLE_CALIB_DONE,
	ANGLE_OFFSET_STORE,
	OPEN_FOC_START,
	CURR_SENS_CALIB,
	FOC_START,
	CONT_ERROR,
	RESET_STATE
} currSession;

typedef enum fnr_state_t {
	FORWARD = 1,
	REVERSE,
	NEUTRAL
} fnr_state_t;

typedef enum power_mode_t {
	ECO,
	SPORTS
} power_mode_t;

typedef enum ADC2_CHANNELS {
	PHASE_U,
	PHASE_V,
	PHASE_W,   /* sensed for CAN display only, not used by FOC */
} ADC2_CHANNELS;

/* ADC1 DMA buffer order == regular rank order in adc.c.
 * PF12 throttle, PA0 U_TEMP, PA1 V_TEMP, PA2 W_TEMP, PA3 bus DC.
 * (Old PC4/PC5 NTCs and PA1 12 V aux sense no longer exist on this board.) */
typedef enum ADC1_CHANNELS {
	THROTTLE,
	U_TEMP,
	V_TEMP,
	W_TEMP,
	BUS_DC,
} ADC1_CHANNELS;

extern volatile fnr_state_t fnr_state;
extern volatile power_mode_t pw_state;
extern volatile currSession cS;
extern errors_nums err;

#endif /* end of file __APP_TYPES_H_ */