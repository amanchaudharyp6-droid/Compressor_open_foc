#ifndef __MOTOR_DEFAULT_H
#define __MOTOR_DEFAULT_H

/* Motor profile — Pegasus default (96 V pack, 1024 PPR encoder) */

/* Electrical */
#define POLEPAIRS				3.0f
#define RS						0.01047f
#define LQ						0.000386f
#define LD						0.000148f
#define LAMBDA					0.0263f
#define LD_PERCENTAGE_CHANGE	15.0f
#define LQ_PERCENTAGE_CHANGE	15.0f
#define FLUX_PERCENTAGE_CHANGE	0.0f

/* Current / torque */
#define MAX_TORQUE_NDK          80.0f
#define MOTOR_PEAK_ARMS			250.0f
#define MOTOR_PEAK_AC			(MOTOR_PEAK_ARMS * ROOT2)
#define CURR_TORQUE_RATIO		(MAX_TORQUE_NDK / MOTOR_PEAK_AC)		/* Max_Torque(Nm) / Max_Peak_Current(A) */

/* Id/Iq reference clamp + measured-OC fault backstop (peak A), sized off
 * MOTOR_PEAK_AC. The old firmware hard-coded 560 A for every motor — a stale
 * value sized for a 384-Arms motor, far above this 250-Arms motor's rating and
 * even above the board phase-OC fault (MAX_PHASE_CURRENT 550 A), so it never
 * acted as a backstop. Deriving from MOTOR_PEAK_AC makes it scale per profile
 * and sit a fixed margin above the Is_max current circle: the clamp never
 * throttles rated torque (a pure-q command reaches MOTOR_PEAK_AC on the q-axis)
 * and the measured-OC fault keeps transient headroom while landing below the
 * board fault. Integer percent so the >=100 floor is preprocessor-checkable
 * (float in _Static_assert is non-conforming — same reason as the FW factors).
 * +Id is never commanded on this PMSM, so ID_UP_LIMIT is kept symmetric only
 * for now and could be tightened toward 0 later (bench-validate transient +Id
 * excursions first). At 250 Arms these land at ~424 A peak (was 560). */
#define IDQ_OC_MARGIN_PCT		120
#if IDQ_OC_MARGIN_PCT < 100
#error "IDQ_OC_MARGIN_PCT must be >= 100 or the Id/Iq clamp throttles rated current"
#endif
#define IQ_UP_LIMIT				((float)IDQ_OC_MARGIN_PCT * 0.01f * MOTOR_PEAK_AC)
#define IQ_LOW_LIMIT			(-IQ_UP_LIMIT)
#define ID_UP_LIMIT				IQ_UP_LIMIT
#define ID_LOW_LIMIT			(-IQ_UP_LIMIT)

/* Pack voltage — 96 V nominal, ~110 V full charge, ~80 V discharged */
#define OP_VOLTAGE				96.0f
#define SVM_VOLTAGE_LIMIT		(OP_VOLTAGE / ROOT3)
#define V_F_RATIO				0.2875f

/* Bus protection thresholds (sized to the pack voltage) */
#define BUS_DC_OV_LIMIT			120
#define BUS_DC_UV_LIMIT			75

/* Motor thermal */
#define MOTOR_TEMP_OT_LIMIT		150
#define MTR_NTC_R25				10000.0f

/* Speed envelope */
#define SPEED_REF_RPM_MAX		500.0f
#define MIN_RPM_FOR_MOTOR_START	80.0f

/* Encoder — single source of truth for quadrature math */
#define ENCODER_PPR				1024
#define TIM2_ARR				((ENCODER_PPR * 4) - 1)
#define COUNTS_TO_RADS			(TWO_PI / (float)(TIM2_ARR + 1))

/* Alignment routine */
#define ALIGN_DUTY				1000
#define ALIGNMENT_PULSE_MS		40
#define ALIGNMENT_REP_COUNT		5

/* Seed offsets — EEPROM-calibrated per unit, these are first-boot fallbacks */
#define OFFSET_CALC_ELEC		0.15f
#define OFFSET_CALC_MECH_CW		0.6211f
#define OFFSET_CALC_MECH_CCW	0.6502f
#define OFFSET_CALC_MECH_AVG	0.63565f

/* ------------------------------------------------------------------
 * FOC tuning — gains depend on Rs/Ld/Lq/Lambda and must be re-tuned
 * per motor. Output limits reference motor-profile constants.
 * ------------------------------------------------------------------ */

/* Low-speed / high-speed regime switch */
#define LOW_SPEED_TO_HIGH_SPEED_SWITCH_RPM	400.0f
#define LOW_SPEED_I_LIMIT					60.0f
#define LOW_SPEED_T_LIMIT					10.0f
#define LOW_SPEED_I_T_SWITCH				1
#define LOW_SPEED_SCALING					0.8f

/* Field weakening.
 * Constraint: FW_RST_FACTOR <= FW_F2M. The FW integrator is edge-reset to
 * "no weakening" when V_s dips below RST*V_max (Current_Ref_Estimator.c);
 * with RST on the M2F entry line (both 0.9) the reset fired the moment FW
 * succeeded in pulling V_s down, snapping the current angle back to MTPA —
 * an Id sawtooth in the marginal-FW band (~2300-3400 RPM at SPORTS torque:
 * audible whine + torque sag, clearing once deep in FW). RST below F2M
 * means the reset can only fire after the MTPA<->FW state machine has
 * already exited to MTPA, never mid-regulation.
 *
 * The three factors are defined from integer percents so the constraint is
 * checkable by the preprocessor (strict C11 forbids float arithmetic in a
 * _Static_assert); the CAN echo transmits them as x100 integers anyway. */
#define FW_KFW				0.002f
#define FW_KAW				12.0f
#define FW_M2F_PCT			85
#define FW_F2M_PCT			55
#define FW_RST_FACTOR_PCT	45
#define FW_M2F				((float)FW_M2F_PCT * 0.01f)
#define FW_F2M				((float)FW_F2M_PCT * 0.01f)
#define FW_RST_FACTOR		((float)FW_RST_FACTOR_PCT * 0.01f)

#if FW_RST_FACTOR_PCT > FW_F2M_PCT
#error "FW_RST_FACTOR must be <= FW_F2M (integrator reset only after FW exit)"
#endif
#if FW_F2M_PCT >= FW_M2F_PCT
#error "FW_F2M must be < FW_M2F (hysteresis: exit line below entry line)"
#endif

/* Torque-ref ramp inside the FOC model ('Torque_gen' rate limiter,
 * applied to T_ref -> T_ref_FNR at Op_ts = 10 kHz; set in FOC_Loop.c /
 * Current_Ref_Estimator.c init). Innermost guard of the torque chain:
 * UP must stay ABOVE every TRQ_SLEW_ACCEL_* so it never binds on feel or
 * protection recovery (it only guards hill-hold / FNR / regen steps);
 * DOWN must stay ABOVE TRQ_SLEW_DECEL_NM_S so the declared cut rate is
 * real (the old -50 silently halved every protective cut to 50 Nm/s).
 * UP is 150 to clear HiRange XR's command slew, raised to 80 Nm/s in the
 * throttle-response retune (this default profile is the one HiRange XR builds
 * against), while staying a pure guard above it. Ordering enforced for
 * whichever variant is active by the static asserts in Torque_Management.h. */
#define FOC_TRQ_RAMP_UP_NM_S	150.0f
#define FOC_TRQ_RAMP_DOWN_NM_S	50.0f

/* Speed PID */
#define SPEED_KP			4.0f
#define SPEED_KI			8.0f
#define SPEED_KD			0.00001f
#define SPEED_KD_FILTER		5.0f
#define SPEED_PID_OUT_UPL	MOTOR_PEAK_AC
#define SPEED_PID_OUT_LOWL	0.0f
#define SPEED_PID_BACK_KAW	ROOT2

/* Id PID */
#define ID_KP				0.4f
#define ID_KI				6.0f
#define ID_KD				0.00001f
#define ID_KD_FILTER		5.0f
#define ID_PID_OUT_UPL		SVM_VOLTAGE_LIMIT
#define ID_PID_OUT_LOWL		-SVM_VOLTAGE_LIMIT
#define ID_PID_BACK_KAW		3.87298f

/* Iq PID */
#define IQ_KP				0.3f
#define IQ_KI				7.0f
#define IQ_KD				0.00001f
#define IQ_KD_FILTER		5.0f
#define IQ_PID_OUT_UPL		SVM_VOLTAGE_LIMIT
#define IQ_PID_OUT_LOWL		-SVM_VOLTAGE_LIMIT
#define IQ_PID_BACK_KAW		4.8304589f

#endif /* __MOTOR_DEFAULT_H */
