#ifndef __MOTOR_HIRANGE_TR_H
#define __MOTOR_HIRANGE_TR_H

/* Motor profile — HiRange TR (76.8 V pack, 1024 PPR encoder)
 *
 * Same physical motor as the 96 V HiRange XR (default profile): identical
 * Rs/Ld/Lq/Lambda/pole-pairs/encoder. Only the pack and the current rating
 * differ — 76.8 V bus and a 384 Arms peak (vs 96 V / 250 Arms on XR).
 *
 * Because it is the same motor, the FOC PID gains and the FW hysteresis are
 * seeded verbatim from the tuned default profile. The FW thresholds are
 * fractions of the SVM voltage limit (which tracks OP_VOLTAGE), so they
 * self-scale to 76.8 V and the FW onset simply moves to a lower speed. The
 * current-loop gains depend on Rs/Ld/Lq (unchanged) and transfer directly.
 * Still: verify FW and Id/Iq stability on the 76.8 V vehicle before hard
 * driving — the operating voltage moved, even if the motor didn't. */

/* Electrical — BENCH: TW-7 (HMC) IPMSM, 48 V, 10 poles. Datasheet: Rs = 12.32 mOhm
 * L-L / 2 @30 C, Ld/Lq 0.0635/0.0879 mH at rated (0.04791/0.06803 mH at peak ->
 * 25 % / 23 % de-rate), flux linkage 11.21 mWb, rated 77.79 Arms, peak 200.6 Arms
 * (60 s), peak torque 25 Nm. Lambda is taken as peak phase flux. */
#define POLEPAIRS				5.0f
#define RS						0.00616f
#define LQ						0.0000879f
#define LD						0.0000635f
#define LAMBDA					0.01121f
#define LD_PERCENTAGE_CHANGE	15.0f
#define LQ_PERCENTAGE_CHANGE	15.0f
#define FLUX_PERCENTAGE_CHANGE	0.0f

/* Current / torque — 384 Arms peak (only the current rating differs from XR).
 * MAX_TORQUE_NDK is the motor's mechanical peak (unchanged); the higher peak
 * current raises MOTOR_PEAK_AC, so CURR_TORQUE_RATIO and the speed-loop
 * current ceiling (SPEED_PID_OUT_UPL) scale with it automatically. */
#define MAX_TORQUE_NDK          25.0f
#define MOTOR_PEAK_ARMS			200.6f
#define MOTOR_PEAK_AC			(MOTOR_PEAK_ARMS * ROOT2)
#define CURR_TORQUE_RATIO		(MAX_TORQUE_NDK / MOTOR_PEAK_AC)		/* Max_Torque(Nm) / Max_Peak_Current(A) */

/* Id/Iq reference clamp + measured-OC fault backstop (peak A), sized off
 * MOTOR_PEAK_AC — see motor_default.h for the full rationale. NOTE: at 384 Arms
 * MOTOR_PEAK_AC is 543 A peak, already within ~7 A of the board phase-OC fault
 * (MAX_PHASE_CURRENT 550 A), so 120% lands at ~652 A — ABOVE the board fault.
 * That is intentional: it leaves transient headroom above Is_max (the old
 * shared 560 A left only ~17 A and risked nuisance trips on this motor) and
 * lets the board phase-OC be the real overcurrent catch for the 384-Arms
 * motor. Verify Id/Iq stability on the 76.8 V vehicle before hard driving. */
#define IDQ_OC_MARGIN_PCT		120
#if IDQ_OC_MARGIN_PCT < 100
#error "IDQ_OC_MARGIN_PCT must be >= 100 or the Id/Iq clamp throttles rated current"
#endif
#define IQ_UP_LIMIT				((float)IDQ_OC_MARGIN_PCT * 0.01f * MOTOR_PEAK_AC)
#define IQ_LOW_LIMIT			(-IQ_UP_LIMIT)
#define ID_UP_LIMIT				IQ_UP_LIMIT
#define ID_LOW_LIMIT			(-IQ_UP_LIMIT)

/* Pack voltage — BENCH: 48 V nominal (TW-7) */
#define OP_VOLTAGE				51.0f
#define SVM_VOLTAGE_LIMIT		(OP_VOLTAGE / ROOT3)
#define V_F_RATIO				0.2875f

/* Bus protection thresholds — RESCALED for the 76.8 V pack. The default's
 * 120/75 are sized to 96 V (x1.25 / x0.78); at 76.8 V a 75 V undervoltage
 * floor sits just below nominal and would fault under any load sag. Kept at
 * the same proportional margins: 76.8 x1.25 = 96, 76.8 x0.78 = 60.
 * Confirm against the actual pack's cell count / BMS cutoffs. */
#define BUS_DC_OV_LIMIT			80
#define BUS_DC_UV_LIMIT			25

/* Motor thermal — identical to the default (same motor) */
#define MOTOR_TEMP_OT_LIMIT		150
#define MTR_NTC_R25				10000.0f

/* Speed envelope */
#define SPEED_REF_RPM_MAX		500.0f
#define MIN_RPM_FOR_MOTOR_START	80.0f

/* Overspeed trip (Limit_Errors.c) — BENCH: TW-7 datasheet max is 10000 rpm
 * (833 Hz electrical); the bench target is 10500. SPORTS_MAX_SPEED caps the
 * reference at 10500; this trips on measured |RPM| above it for
 * OVERSPEED_COUNT consecutive 1 ms checks. Must stay below
 * SPEED_SENSE_CLAMP_RAD_S (Speed_Sensing.c, 10982 rpm), or the clamped speed
 * estimate can never reach it. */
#define OVERSPEED_TRIP_RPM		10800.0f

/* Encoder — same 1024 PPR as the default */
#define ENCODER_PPR				1024
#define TIM2_ARR				((ENCODER_PPR * 4) - 1)
#define COUNTS_TO_RADS			(TWO_PI / (float)(TIM2_ARR + 1))

/* Alignment routine */
#define ALIGN_DUTY				1000
#define ALIGNMENT_PULSE_MS		40
#define ALIGNMENT_REP_COUNT		5

/* Seed offsets — EEPROM-calibrated per unit; these are the fallbacks used
 * when the EEPROM read fails (Read_EEPROM_at_init). BENCH: TW-7 autotune
 * result (8000rpm_3 log: avg 1.128, CW/CCW 1.12 at the 0.01 telemetry
 * resolution; earlier autotunes gave 1.148). The old XR seeds (0.62/0.65)
 * are ~0.5 rad mech = ~2.5 rad elec off on this motor, which is what made
 * initial_try lose all current tracking after an EEPROM_READ_ERROR boot. */
#define OFFSET_CALC_ELEC		0.15f
#define OFFSET_CALC_MECH_CW		1.128f
#define OFFSET_CALC_MECH_CCW	1.128f
#define OFFSET_CALC_MECH_AVG	1.128f

/* ------------------------------------------------------------------
 * FOC tuning — seeded from the tuned 96 V default (same motor). See
 * motor_default.h for the mechanism notes and the RST <= F2M constraint.
 * ------------------------------------------------------------------ */

/* Low-speed / high-speed regime switch */
#define LOW_SPEED_TO_HIGH_SPEED_SWITCH_RPM	400.0f
#define LOW_SPEED_I_LIMIT					60.0f
#define LOW_SPEED_T_LIMIT					10.0f
#define LOW_SPEED_I_T_SWITCH				1
#define LOW_SPEED_SCALING					0.8f

/* Field weakening — fractions of SVM_VOLTAGE_LIMIT, so they track the 76.8 V
 * bus automatically (FW onset simply moves to a lower speed than on XR). */
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

/* Field weakening — closed-loop path (FW_CLOSED_LOOP, Current_Ref_Estimator.c).
 * dId/dt = FW_CL_KI * (FW_CL_VREF_PCT% * Vbus/sqrt3 - |Vd,Vq|), dId <= 0.
 * The FW_KFW/KAW/M2F/F2M/RST block above is unused while FW_CLOSED_LOOP is set.
 *
 * FW_CL_VREF_PCT: voltage the loop holds |Vd,Vq| at, as % of the model's
 *   linear SVM limit (model Vbus = max(OP_VOLTAGE, pack) / sqrt3). 90 leaves
 *   headroom for the current PIs and for DC-link sag: the dyno supply sagged
 *   from 66 V to 60-62 V under load while the model stayed pinned at 66 V
 *   (real linear limit ~3-9% below the model's).
 * FW_CL_KI (A per V.s): loop gain is ~FW_CL_KI * we*Ld, i.e. ~20 rad/s (~3 Hz)
 *   at 2500 rpm (we ~785 rad/s, Ld ~0.00013 H), rising with speed. Worst case
 *   with both PIs pinned (|V| ~ 50 V, error ~ -16 V) is ~3.2 kA/s, so -200 A
 *   in ~60 ms. Slow enough not to fight the current PIs, fast enough to beat
 *   the speed loop (Kp 4 / Ki 8).
 * FW_CL_ID_FLOOR_K: Id floor = -K * Lambda/Ld, also clamped to -Is_max and the
 *   model's Id_low_limit. Lambda/Ld is where the d-axis flux reaches zero
 *   (~-180 A nominal, ~-210 A with the degraded Ld). Because Lq >> Ld the
 *   minimum-voltage point for a given torque lies BEYOND it: at 2500 rpm /
 *   42 Nm it is Id ~ -310..-380 A, Iq ~ 95..110 A. K = 1 would cap FW right at
 *   zero margin there; 1.6 gives ~-285..-335 A. Raise only with the motor's
 *   demagnetisation limit in hand. UNVALIDATED on the vehicle/dyno: check
 *   Id/Iq/|V| on 0x705/0x724 and the FW flag on 0x715.
 *   Dyno so_close_yet_so_far (K 1.6, floor -335 A, 41 Nm then collapse): a
 *   fit at the plateau (Ld ~113 uH, Lq ~190 uH) puts MTPV at Id ~-382 A,
 *   i.e. K ~1.83 with the degraded Ld 125.8 uH. 1.8 (-376 A) sits on it;
 *   beyond ~1.85 FW only costs current and LOSES torque at the voltage limit.
 *
 * Iq ceiling at the voltage edge (stops the cliff, not a torque gain):
 * FW_CL_ID_ERR_A: measured Id minus Id_ref (A) above which, while |V| is above
 *   the FW target, the d-axis loop is taken to be losing authority. The
 *   stable 41 Nm plateau ran at +8..+23 A; the collapse began at +88 A.
 * FW_CL_IQ_KI (1/s): Iq ceiling cut rate per amp of error above that, e.g.
 *   20 A excess -> 400 A/s.
 * FW_CL_IQ_RELEASE_A_S: rate the ceiling rises back when Id tracks again.
 *   Slow, so it does not hunt against the cut. */
#define FW_CL_VREF_PCT		85
#define FW_CL_KI			200.0f
/* TW-7 BENCH: 1.7 above was sized for the XR machine, whose lambda/Ld sits
 * beyond its current rating. The TW-7 is the opposite: lambda/Ld = 177 A
 * (234 A with the peak-current Ld) is BELOW its 284 A peak, so it reaches
 * zero d-axis flux inside the rating and 1.7 would allow ~-300 A — current
 * spent past the zero-flux point, and a demagnetisation risk. 1.1 gives
 * ~-194 A nominal / ~-259 A with fully degraded Ld (Is_max still caps at
 * 284 A). Datasheet envelope at 48 V: ~-170 A is needed at 10000 rpm with
 * the dyno's ~6 Nm load. Confirm the demagnetisation current with HMC. */
#define FW_CL_ID_FLOOR_K	1.1f
#define FW_CL_ACTIVE_A		2.0f	/* |dId| above this reports FW active on 0x715 */
#define FW_CL_ID_ERR_A		40.0f
#define FW_CL_IQ_KI			20.0f
#define FW_CL_IQ_RELEASE_A_S	50.0f

/* Torque-ref ramp inside the FOC model — see motor_default.h / Torque_Management.h
 * for the ordering constraints (UP above every TRQ_SLEW_ACCEL_*, DOWN above DECEL). */
#define FOC_TRQ_RAMP_UP_NM_S	100.0f
#define FOC_TRQ_RAMP_DOWN_NM_S	150.0f

/* Speed PID — output ceiling is MOTOR_PEAK_AC, which now reflects 384 Arms */
#define SPEED_KP			4.0f
#define SPEED_KI			8.0f
#define SPEED_KD			0.00001f
#define SPEED_KD_FILTER		5.0f
#define SPEED_PID_OUT_UPL	MOTOR_PEAK_AC
#define SPEED_PID_OUT_LOWL	0.0f
#define SPEED_PID_BACK_KAW	ROOT2

/* Id PID */
#define ID_KP				0.17f
#define ID_KI				3.5f
#define ID_KD				0.00001f
#define ID_KD_FILTER		5.0f
#define ID_PID_OUT_UPL		SVM_VOLTAGE_LIMIT
#define ID_PID_OUT_LOWL		-SVM_VOLTAGE_LIMIT
#define ID_PID_BACK_KAW		3.87298f

/* Iq PID */
#define IQ_KP				0.15f
#define IQ_KI				4.0f
#define IQ_KD				0.00001f
#define IQ_KD_FILTER		5.0f
#define IQ_PID_OUT_UPL		SVM_VOLTAGE_LIMIT
#define IQ_PID_OUT_LOWL		-SVM_VOLTAGE_LIMIT
#define IQ_PID_BACK_KAW		4.8304589f

#endif /* __MOTOR_HIRANGE_TR_H */
