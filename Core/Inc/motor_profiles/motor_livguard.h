#ifndef __MOTOR_LIVGUARD_H
#define __MOTOR_LIVGUARD_H

/* Motor profile — LivGuard (96 V pack, 512 PPR encoder)
 * Values sourced from dev/PEG_v4/FOC_Model_Based/LivGuard/v1.0.1.
 * Re-tune PID gains (in vehicle_config.h) when activating this profile. */

/* Electrical */
#define POLEPAIRS				3.0f
#define RS						17.76E-3
#define LQ						0.2E-3f
#define LD						0.09E-3f
#define LAMBDA					33.76E-3f
#define LD_PERCENTAGE_CHANGE	15.0f
#define LQ_PERCENTAGE_CHANGE	15.0f
#define FLUX_PERCENTAGE_CHANGE	0.0f

/* Current / torque */
#define MAX_TORQUE_LG           80.0f
#define MOTOR_PEAK_ARMS			290.0f
#define MOTOR_PEAK_AC			(MOTOR_PEAK_ARMS * ROOT2)
#define CURR_TORQUE_RATIO		(MAX_TORQUE_LG / MOTOR_PEAK_AC)

/* Id/Iq reference clamp + measured-OC fault backstop (peak A), sized off
 * MOTOR_PEAK_AC — see motor_default.h for the full rationale. At 290 Arms this
 * is ~410 A peak Is_max, so 120% lands at ~492 A peak — below the board
 * phase-OC fault (MAX_PHASE_CURRENT 550 A), so it acts as a real motor-level
 * backstop. Was a shared hard-coded 560 A. */
#define IDQ_OC_MARGIN_PCT		120
#if IDQ_OC_MARGIN_PCT < 100
#error "IDQ_OC_MARGIN_PCT must be >= 100 or the Id/Iq clamp throttles rated current"
#endif
#define IQ_UP_LIMIT				((float)IDQ_OC_MARGIN_PCT * 0.01f * MOTOR_PEAK_AC)
#define IQ_LOW_LIMIT			(-IQ_UP_LIMIT)
#define ID_UP_LIMIT				IQ_UP_LIMIT
#define ID_LOW_LIMIT			(-IQ_UP_LIMIT)

/* Pack voltage */
#define OP_VOLTAGE				86.4f
#define SVM_VOLTAGE_LIMIT		(OP_VOLTAGE / ROOT3)
#define V_F_RATIO				0.2875f

/* Bus protection thresholds */
#define BUS_DC_OV_LIMIT			105
#define BUS_DC_UV_LIMIT			65

/* Motor thermal */
#define MOTOR_TEMP_OT_LIMIT		150
#define MTR_NTC_R25				45000.0f

/* Speed envelope */
#define SPEED_REF_RPM_MAX		500.0f
#define MIN_RPM_FOR_MOTOR_START	80.0f

/* Encoder */
#define ENCODER_PPR				512
#define TIM2_ARR				((ENCODER_PPR * 4) - 1)
#define COUNTS_TO_RADS			(TWO_PI / (float)(TIM2_ARR + 1))

/* Alignment routine */
#define ALIGN_DUTY				1000
#define ALIGNMENT_PULSE_MS		40
#define ALIGNMENT_REP_COUNT		5

/* Seed offsets — confirm by running alignment after first flash */
#define OFFSET_CALC_ELEC		0.15f
#define OFFSET_CALC_MECH_CW		0.6211f
#define OFFSET_CALC_MECH_CCW	0.6502f
#define OFFSET_CALC_MECH_AVG	0.63565f

/* ------------------------------------------------------------------
 * FOC tuning — values below are carried over from the LivGuard branch.
 * LOW_SPEED_* regime-switch wasn't present on that branch; defaults
 * mirror motor_default.h and may need re-tuning.
 * ------------------------------------------------------------------ */

/* Low-speed / high-speed regime switch */
#define LOW_SPEED_TO_HIGH_SPEED_SWITCH_RPM	400.0f
#define LOW_SPEED_I_LIMIT					60.0f
#define LOW_SPEED_T_LIMIT					10.0f
#define LOW_SPEED_I_T_SWITCH				1
#define LOW_SPEED_SCALING					0.8f

/* Field weakening — see motor_default.h for the mechanism and the
 * RST <= F2M constraint. The previous values had INVERTED hysteresis
 * (F2M 0.95 > M2F 0.9): the exit line sat above the entry line, so on
 * FW entry the exit condition was already true -> MTPA<->FW toggles every
 * cycle at the boundary (Id chatter, whine, torque sag). Ordering corrected
 * to the proven pattern (M2F 0.9 entry > F2M 0.7 exit > RST 0.65 reset).
 * Kfw/Kaw are LivGuard's own; all five are STARTING POINTS — this motor
 * (Ld/Lq/lambda and 86.4 V pack differ from default) needs its own FW
 * tuning pass. */
#define FW_KFW				0.002f
#define FW_KAW				13.0f
#define FW_M2F_PCT			90
#define FW_F2M_PCT			70
#define FW_RST_FACTOR_PCT	65
#define FW_M2F				((float)FW_M2F_PCT * 0.01f)
#define FW_F2M				((float)FW_F2M_PCT * 0.01f)
#define FW_RST_FACTOR		((float)FW_RST_FACTOR_PCT * 0.01f)

#if FW_RST_FACTOR_PCT > FW_F2M_PCT
#error "FW_RST_FACTOR must be <= FW_F2M (integrator reset only after FW exit)"
#endif
#if FW_F2M_PCT >= FW_M2F_PCT
#error "FW_F2M must be < FW_M2F (hysteresis: exit line below entry line)"
#endif

/* Torque-ref ramp inside the FOC model — see motor_default.h for the
 * ordering constraints. Values mirrored, profile untuned. */
#define FOC_TRQ_RAMP_UP_NM_S	100.0f
#define FOC_TRQ_RAMP_DOWN_NM_S	150.0f

/* Speed PID */
#define SPEED_KP			4.0f
#define SPEED_KI			6.0f
#define SPEED_KD			0.00001f
#define SPEED_KD_FILTER		5.0f
#define SPEED_PID_OUT_UPL	MOTOR_PEAK_AC
#define SPEED_PID_OUT_LOWL	0.0f
#define SPEED_PID_BACK_KAW	1.0f

/* Id PID */
#define ID_KP				0.3f
#define ID_KI				2.5f
#define ID_KD				0.00001f
#define ID_KD_FILTER		5.0f
#define ID_PID_OUT_UPL		SVM_VOLTAGE_LIMIT
#define ID_PID_OUT_LOWL		-SVM_VOLTAGE_LIMIT
#define ID_PID_BACK_KAW		1.0f

/* Iq PID */
#define IQ_KP				0.2f
#define IQ_KI				3.5f
#define IQ_KD				0.00001f
#define IQ_KD_FILTER		5.0f
#define IQ_PID_OUT_UPL		SVM_VOLTAGE_LIMIT
#define IQ_PID_OUT_LOWL		-SVM_VOLTAGE_LIMIT
#define IQ_PID_BACK_KAW		1.0f

#endif /* __MOTOR_LIVGUARD_H */
