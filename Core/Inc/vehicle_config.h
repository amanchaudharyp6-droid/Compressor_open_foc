#ifndef __VEHICLE_CONFIG_H
#define __VEHICLE_CONFIG_H

#include <stdbool.h>   /* true/false — used by the config-consistency #if guards below */

/* Application/vehicle-level configuration: feature flags, drive modes,
 * regen, throttle shaping. Motor-dependent tuning (FOC PID gains, FW)
 * lives in the motor profile, not here. */

/* Feature / debug toggles */
#define EST_CYC_CNT			1
#define PROTECTION_MODEL	0
/* Mutually exclusive boot fault-check selection (isr_handlers.c). Both set
 * would run the check and then ignore its verdict; both clear would leave the
 * state machine stuck in CURR_SENS_CALIB forever, with no transition to
 * ANGLE_CALIB or FOC_START and no diagnostic. */
#define ENABLE_FAULTS		1
#define DISABLE_FAULTS		0
#if (ENABLE_FAULTS + DISABLE_FAULTS) != 1
#error "vehicle_config.h: set exactly one of ENABLE_FAULTS / DISABLE_FAULTS"
#endif
#define ENABLE_ENC_ERRORS	1
#define FORCE_ANGLETUNE		0
/* Inverter dead-time compensation (FOC_Loop.c init -> model input).
 * Cancels the dead-time voltage error by adding +/-(t_dead*f_sw*Vdc) per
 * phase, signed by phase-current direction. Enabled 2026-07-21 to attack
 * the 6*f_e ripple whine that appears only inside field weakening — below
 * FW the current loop has voltage headroom and rejects the distortion; in
 * FW it is voltage-saturated and cannot, so the ripple reaches the torque.
 * Symptom fingerprint: pitch rises with speed (locked to electrical freq)
 * and fades as speed climbs (harmonic impedance 6*we*L rises). */
#define DEAD_TIME_COMP_ENABLE	0
/* Salient-torque inverse fix (Current_Ref_Estimator.c, commit 1de0f51). Gates
 * the hand-added salient-locus solve that makes COMMANDED torque equal
 * DELIVERED torque.
 *   1 = fix ON  — honest axis: a 40 Nm SPORTS command delivers 40 Nm and draws
 *                 ~2.2x less current. REQUIRES the cap rescale in
 *                 HANDOFF_HILLHOLD_AND_FW.md Issue 3, else the vehicle is
 *                 correct but slow. This is the production target.
 *   0 = fix OFF — the generated non-salient inverse stands (a 40 Nm command
 *                 delivers ~90 Nm), matching every tune/test done before the
 *                 fix landed.
 * Set 0 to A/B the regen / decel-jerk changes on the original torque axis
 * (currents and over-modulation match the HANDOFF_DECEL_JERK.md tables). */
#define FOC_SALIENT_TORQUE_INVERSE	0
/* Accept 0x729 (angle auto-calib trigger) on vehicle CAN. Default OFF:
 * if another ECU transmits 0x729 on this bus it could start angle
 * calibration unintentionally. Enable only for bench calibration. Gates
 * both the FDCAN hardware filter (fdcan_AL.c) and the Rx dispatch (main.c). */
#define CAN_AUTOTUNE_TRIGGER	1

/* CAN / throttle source selection */
#define VH_CAN_ID			1
#define DEBUG_CAN_ID		1
#define CAN_BASED_REF		1
#define CAN_BASED_DIR		1
#define THROTTLE_BASED_DIR	0
#define THROTTLE_BASED_REF	0
#define CAN_BASED_THRV		1
#define THR_BASED_THRV		0

/* Each pair above selects the single source for one input, and the code
 * #if-includes both alternatives independently — so an inconsistent pair
 * fails silently rather than loudly:
 *   REF  both set -> the CAN Rx ISR (prio 6) and the superloop both write
 *                    speed_ref / torque_ref (see the concurrency map in
 *                    main.h, invariant 2); both clear -> no reference is
 *                    ever produced and the drive never commands torque.
 *   DIR  both set -> two blocks in power_mode_fnr_switch() fight over
 *                    fnr_state / Drive_State; both clear -> direction stays
 *                    NEUTRAL forever.
 *   THRV both set -> throttle voltage written from CAN and from the ADC;
 *                    both clear -> throttle_v is never updated.
 * Exactly one of each pair must be set. */
#if (CAN_BASED_REF + THROTTLE_BASED_REF) != 1
#error "vehicle_config.h: set exactly one of CAN_BASED_REF / THROTTLE_BASED_REF"
#endif
#if (CAN_BASED_DIR + THROTTLE_BASED_DIR) != 1
#error "vehicle_config.h: set exactly one of CAN_BASED_DIR / THROTTLE_BASED_DIR"
#endif
#if (CAN_BASED_THRV + THR_BASED_THRV) != 1
#error "vehicle_config.h: set exactly one of CAN_BASED_THRV / THR_BASED_THRV"
#endif

/* Vehicle speed conversion — feeds the CAN 0x706 dashboard speed only
 * (control uses motor RPM directly, so this factor never affects
 * dynamics). Platform factors below; the active RPM_TO_KMPH is selected
 * automatically by the vehicle variant (see below the variant guards) —
 * no manual flag to forget when switching vehicles. */
#define RPM_TO_KMPH_HLP			0.01072f
#define RPM_TO_KMPH_HI_CITY		0.01023f
#define RPM_TO_KMPH_HI_RANGE	0.01044f

#define HIRANGE_XR          0
#define HILOAD_XR           0
#define HIRANGE_TR          1

#if HILOAD_XR
#define CONFIG_VERSION_MAJOR 		0x02
#define CONFIG_VERSION_MINOR 		0x00
#define CONFIG_VERSION_SUBMINOR 	0x00
#define FIRMWARE_VERSION_MAJOR  	0x00
#define FIRMWARE_VERSION_MINOR 		0x0D
#define FIRMWARE_VERSION_SUBMINOR 	0x00
#endif
#if HIRANGE_XR
#define CONFIG_VERSION_MAJOR 		0x05
#define CONFIG_VERSION_MINOR 		0x00
#define CONFIG_VERSION_SUBMINOR 	0x00
#define FIRMWARE_VERSION_MAJOR  	0x00
#define FIRMWARE_VERSION_MINOR 		0x0C
#define FIRMWARE_VERSION_SUBMINOR 	0x01
#endif
#if HIRANGE_TR
/* 76.8 V variant. Version numbers are a fresh line — set these to your
 * release scheme; placeholders below (config 1.0.0 / fw 0.0C, XR-derived). */
#define CONFIG_VERSION_MAJOR 		0x02
#define CONFIG_VERSION_MINOR 		0x00
#define CONFIG_VERSION_SUBMINOR 	0x00
#define FIRMWARE_VERSION_MAJOR  	0x00
#define FIRMWARE_VERSION_MINOR 		0x0B
#define FIRMWARE_VERSION_SUBMINOR 	0x01
#endif

/* Exactly one variant must be selected — the blocks below (and in
 * Torque_Management.h/.c, main.c) are mutually exclusive definitions. */
#if (HIRANGE_XR + HILOAD_XR + HIRANGE_TR) != 1
#error "vehicle_config.h: select exactly one vehicle variant (HIRANGE_XR / HILOAD_XR / HIRANGE_TR)"
#endif

/* Closed-loop field weakening (Current_Ref_Estimator.c). The generated FW
 * regulator drives its integrator from V_s, which is computed from the MTPA
 * references and so never sees the FW correction: above base speed V_s/V_max
 * stays > 1, the integrator never settles, and (at the Kfw values that are
 * safe) the vector never leaves the MTPA curve. Dyno peak_power_2/3 (HiRange
 * TR, 66 V supply): Id_ref tracked MTPA within ~1-2 A while FW was "active",
 * Vd pinned at -Vbus/sqrt3, then Id ran positive and speed collapsed.
 * This replaces it with a voltage-feedback loop on the commanded |Vd,Vq| that
 * adds -Id on top of MTPA and trims Iq to the Is_max circle. HIRANGE_TR only
 * for now; the other variants keep the generated path unchanged. Tuning:
 * FW_CL_* in the motor profile. Set 0 to A/B against the generated path. */
#define FW_CLOSED_LOOP		HIRANGE_TR

/* Motor/vehicle pairing guards. MOTOR_PROFILE comes from motor_select.h
 * (included just before this file in main.h).
 *  - HiLoad XR and LivGuard are bound BOTH ways (confirmed 2026-08-02: HiLoad
 *    XR always runs the LivGuard motor). The reverse direction is the one that
 *    matters for safety: building HiLoad XR against the 96 V default profile
 *    would take BUS_DC_OV_LIMIT to 120 on an 86.4 V pack — a 15 V-too-lenient
 *    overvoltage trip — plus a 28 %-wrong Lambda through the whole FOC model.
 *  - HiRange TR is the 76.8 V pack: it carries its own OP_VOLTAGE / bus OV-UV
 *    thresholds, so it is bound BOTH ways to the HiRange TR profile — neither
 *    may pair with anything else (a 96 V profile on the 76.8 V pack, or vice
 *    versa, would mis-set the undervoltage floor). */
#if defined(MOTOR_PROFILE) && (MOTOR_PROFILE == MOTOR_PROFILE_LIVGUARD) && !HILOAD_XR
#error "LivGuard motor profile must be paired with HILOAD_XR (see motor_select.h)"
#endif
#if HILOAD_XR && defined(MOTOR_PROFILE) && (MOTOR_PROFILE != MOTOR_PROFILE_LIVGUARD)
#error "HILOAD_XR requires the LivGuard motor profile (set MOTOR_PROFILE = MOTOR_PROFILE_LIVGUARD in motor_select.h) — the 96 V default profile would set BUS_DC_OV_LIMIT 120 on an 86.4 V pack"
#endif
#if defined(MOTOR_PROFILE) && (MOTOR_PROFILE == MOTOR_PROFILE_HIRANGE_TR) && !HIRANGE_TR
#error "HiRange TR (76.8 V) motor profile must be paired with HIRANGE_TR (see motor_select.h)"
#endif
#if HIRANGE_TR && defined(MOTOR_PROFILE) && (MOTOR_PROFILE != MOTOR_PROFILE_HIRANGE_TR)
#error "HIRANGE_TR requires the 76.8 V motor profile (set MOTOR_PROFILE = MOTOR_PROFILE_HIRANGE_TR in motor_select.h)"
#endif

/* RPM->km/h for CAN 0x706, auto-selected by variant.
 * User-confirmed mapping (2026-07-20): HiRange XR -> 0.01044 (HI_RANGE),
 * HiLoad XR -> 0.01072 (HLP). HiRange TR SEEDED to the HiRange factor
 * (0.01044) on the assumption it shares wheel size / gearing with HiRange XR
 * — this depends only on driveline, not pack voltage. CONFIRM on the TR
 * vehicle; if the rolling radius or ratio differs, give it its own factor. */
#if HIRANGE_XR
#define RPM_TO_KMPH			RPM_TO_KMPH_HI_RANGE
#endif
#if HILOAD_XR
#define RPM_TO_KMPH			RPM_TO_KMPH_HLP
#endif
#if HIRANGE_TR
#define RPM_TO_KMPH			RPM_TO_KMPH_HI_RANGE
#endif

/* Drive mode envelopes */
#if HIRANGE_XR
#define ECO_MAX_SPEED		4500.0f
#define SPORTS_MAX_SPEED	6000.0f
#define REV_MAX_SPEED		1700.0f
#define ECO_MAX_TORQUE		18.0f
/* HiRange XR street-safe launch tune (vehicle-tested): 40 Nm equals the XR
 * torque-table peak — steady-state launch torque, not transient shaping,
 * was what made SPORTS unsafe (wheel lift unladen on gradient). Keep
 * >= ~38 Nm so laden hill-start pull-away retains margin over the 32 Nm
 * hill-hold ceiling. The XR table tracks the 5.655 kW power line from
 * ~2500 RPM, so retune SPORTS_MAX_POWER and the table together. */
#define SPORTS_MAX_TORQUE	34.0f
#define MAX_TORQUE          34.0f
#define REV_MAX_TORQUE		18.0f
#define MIN_POWER			2000.0f
#define MAX_POWER			6284.0f
#define ECO_MAX_POWER		2514.0f
#define SPORTS_MAX_POWER	6284.0f
#endif
#if HILOAD_XR
#define ECO_MAX_SPEED		4400.0f
#define SPORTS_MAX_SPEED	4850.0f
#define REV_MAX_SPEED		1700.0f
#define ECO_MAX_TORQUE		35.0f
/* HiLoad XR (cargo variant): full 80 Nm motor-peak launch with the 9.215 kW
 * envelope — the higher GVW is expected to absorb the launch torque that
 * was unsafe on the lighter HiRange XR. NOT yet vehicle-tested: re-check
 * unladen wheel lift on gradient before releasing this variant. */
#define SPORTS_MAX_TORQUE	70.0f
#define MAX_TORQUE          70.0f
#define REV_MAX_TORQUE		35.0f
#define MIN_POWER			2000.0f
#define MAX_POWER			9215.0f
#define ECO_MAX_POWER		3686.0f
#define SPORTS_MAX_POWER	9215.0f
#endif
#if HIRANGE_TR
/* HiRange TR (76.8 V) — same HiRange chassis and motor as XR, on the 76.8 V
 * pack with the 384 Arms current rating. Power-limited SPORTS envelope: a flat
 * 72 Nm launch (= SPORTS_MAX_TORQUE = torque-table peak) up to the constant-
 * power knee at ~750 RPM, then the 5.655 kW power line (torque = 5655 / omega).
 * Same peak power as XR (SPORTS 5.655 kW / ECO 2.262 kW) but a higher launch
 * (72 vs XR's 40 Nm) using the 384 A head-room — TR deliberately holds back
 * from the motor's full 80 Nm. NOT yet vehicle-tested: re-check unladen
 * wheel-lift on gradient.
 *  - The >=~5000 RPM table tail follows the power-line ideal; at 76.8 V the
 *    back-EMF/voltage limit sits below it and binds first — re-map the tail on
 *    logged TR data.
 *  - BENCH (TW-7 on the HiRange TR profile): SPORTS_MAX_SPEED is the TW-7
 *    datasheet maximum, 10000 rpm. It also caps the CAN speed reference
 *    (Helper_Functions.c), so the dyno GUI cannot command past the motor's
 *    rating; OVERSPEED_TRIP_RPM in the motor profile is the hard trip above
 *    it. Restore 5900 before this build goes back on a TR vehicle.
 *    10500 = bench overspeed target, 5 % above the datasheet maximum. */
#define ECO_MAX_SPEED		4500.0f
#define SPORTS_MAX_SPEED	10500.0f
#define REV_MAX_SPEED		1700.0f
#define ECO_MAX_TORQUE		32.0f
#define SPORTS_MAX_TORQUE	72.0f
#define MAX_TORQUE          72.0f
#define REV_MAX_TORQUE		32.0f
#define MIN_POWER			2000.0f
#define MAX_POWER			5655.0f
#define ECO_MAX_POWER		2262.0f
#define SPORTS_MAX_POWER	5655.0f
#endif
#define FOC_MODE_TORQUE		false
#define FOC_MODE_SPEED      true
#define OPERATION_MODE      FOC_MODE_SPEED
/* Dyno / bench: the firmware itself holds Forward + minimum throttle voltage
 * (and regen off), so the only CAN input needed is the 0x102 speed reference.
 * Same conditions the angle-autotune closed-loop phase already runs under
 * (isr_handlers.c). 0x108 / 0x114 are then ignored. Needs speed mode and
 * CAN_BASED_REF / DIR / THRV. Set 0 for normal F/N/R + throttle over CAN. */
#define CAN_SPEED_ONLY		1
#if CAN_SPEED_ONLY && !((OPERATION_MODE == FOC_MODE_SPEED) && CAN_BASED_REF && CAN_BASED_DIR && CAN_BASED_THRV)
#error "CAN_SPEED_ONLY needs OPERATION_MODE=FOC_MODE_SPEED and CAN_BASED_REF/DIR/THRV all 1"
#endif
#define DIR_FLIP			1
#define DIR_NORMAL			0
/* Motor mounting/wiring sense is per-vehicle. HiRange XR (gearbox reposition
 * 2026-07-29) and HiRange TR run DIR_NORMAL; HiLoad XR runs DIR_FLIP.
 * REGEN_POLARITY (regen block below) MUST track this — the consistency guard
 * further down enforces it. */
#if HILOAD_XR
#define MOTOR_DIRECTION		DIR_FLIP
#endif
#if HIRANGE_XR || HIRANGE_TR
#define MOTOR_DIRECTION		DIR_NORMAL
#endif

/* Regen — per drive mode (see update_regen_fade in Torque_Management.c).
 *   ECO    : NO fade — regen active across the whole speed range at
 *            REGEN_MAX_TORQUE_ECO, for consistent, proper deceleration.
 *   SPORTS : fades regen out at high speed (REGEN_FADE_*) AND uses a reduced
 *            REGEN_MAX_TORQUE_SPORTS, because a larger regen command re-engaging
 *            at speed triggers a current-angle excursion (the decel jerk).
 * REGEN_MAX_TORQUE is the FOC model init default only (pre-drive; overwritten
 * by update_regen_fade every loop once driving). Tune the two per-mode values. */
#if HIRANGE_XR
#define REGEN_MAX_TORQUE_ECO	2.5f	/* ECO regen (no fade) — raise for stronger ECO deceleration */
#define REGEN_MAX_TORQUE_SPORTS	3.0f	/* SPORTS regen (faded) — reduced to kill the decel jerk      */
#endif
#if HILOAD_XR
#define REGEN_MAX_TORQUE_ECO	5.0f
#define REGEN_MAX_TORQUE_SPORTS	5.0f
#endif
#if HIRANGE_TR
#define REGEN_MAX_TORQUE_ECO	8.0f	/* seeded = HiRange XR */
#define REGEN_MAX_TORQUE_SPORTS	8.0f
#endif
#define REGEN_MAX_TORQUE	REGEN_MAX_TORQUE_SPORTS	/* FOC init default only */
#define REGEN_MIN_RPM_EN	400.0f
/* Coupled to MOTOR_DIRECTION (see guard below): DIR_FLIP -> true,
 * DIR_NORMAL -> false. HiRange XR/TR run DIR_NORMAL, so regen arms in the
 * opposite electrical Drive_State from HiLoad. */
#if HILOAD_XR
#define REGEN_POLARITY		true
#endif
#if HIRANGE_XR || HIRANGE_TR
#define REGEN_POLARITY		false
#endif
#define REGEN_CAP_RPM		1300.0f

/* Less-eager regen arming (regen-latch fix, 2026-07-29). The FOC chart arms
 * regen after ~100 ms of coast (pedal below the 1.0 V FNR threshold), which
 * sits only ~20 mV above the torque-zero point — so a slight lift arms regen
 * and grabs. When enabled, the app (main.c) permits arming only while the
 * driver is NOT commanding drive torque (<= REGEN_ARM_TORQUE_NM), so a small
 * lift coasts instead of braking. Gates ARMING only; the regen exit path is
 * untouched (that is the FOC_Loop.c polarity-correct-exit fix). */
#define REGEN_LESS_EAGER_ARM	1
#define REGEN_ARM_TORQUE_NM		0.5f	/* arm regen only below this drive torque (Nm) */

/* --- Direction / regen-polarity consistency guard -------------------------
 * MOTOR_DIRECTION and REGEN_POLARITY are coupled and MUST move together:
 *   - MOTOR_DIRECTION remaps FORWARD<->REVERSE at the Drive_State boundary
 *     (Helper_Functions.c).
 *   - REGEN_POLARITY selects which electrical Drive_State is allowed to arm
 *     regen (FOC_Loop.c): Forward arms when false, Reverse arms when true.
 * Regen must arm in the state that corresponds to the driver's forward-driving
 * direction, so the valid pairings are:
 *       MOTOR_DIRECTION == DIR_FLIP    <->  REGEN_POLARITY == true
 *       MOTOR_DIRECTION == DIR_NORMAL  <->  REGEN_POLARITY == false
 * Setting one without the other loses regen while driving forward. These
 * checks fail the build on any inconsistent (or invalid) pairing. */
#if (MOTOR_DIRECTION != DIR_FLIP) && (MOTOR_DIRECTION != DIR_NORMAL)
#error "MOTOR_DIRECTION must be DIR_FLIP or DIR_NORMAL."
#endif
#if (MOTOR_DIRECTION == DIR_FLIP) && (REGEN_POLARITY != true)
#error "Inconsistent config: MOTOR_DIRECTION==DIR_FLIP requires REGEN_POLARITY==true. Flip both together (regen must arm in the driver's forward direction)."
#endif
#if (MOTOR_DIRECTION == DIR_NORMAL) && (REGEN_POLARITY != false)
#error "Inconsistent config: MOTOR_DIRECTION==DIR_NORMAL requires REGEN_POLARITY==false. Flip both together (regen must arm in the driver's forward direction)."
#endif

/* High-speed regen fade (Torque_Management.c writes the FOC model's regen
 * cap every loop). Commanded regen ramps smoothly to zero as speed rises
 * into the field-weakening region, and back in below it — a hard on/off
 * boundary would only relocate the lurch, so this is a smoothstep with zero
 * slope at both edges. The band brackets the FW-exit knee; note the true FW
 * boundary moves with pack voltage/SOC, which is why this is a taper.
 *
 * The knee is VOLTAGE-DEPENDENT, so the band is per-variant: at 96 V the knee
 * was ~3200 RPM (XR aggressive-decel log). FW onset scales roughly with pack
 * voltage, so HiRange TR's 76.8 V knee is ~3200 x 76.8/96 = ~2560 RPM and its
 * band is the XR band scaled x0.8 (2240-2800) so the fade completes as FW
 * begins rather than after it. SEED — confirm the TR knee on logged data and
 * re-center the band.
 *
 * DIAGNOSTIC VALUE: commanded regen is only REGEN_MAX_TORQUE (1.5 Nm on XR)
 * while the observed decel braking is ~20 Nm / 135 A. If the sharp decel is
 * unchanged with this active, the braking is NOT coming from the commanded
 * regen path -> uncontrolled generation in FW -> the fix must be FW-side
 * (Id management on decel), not a torque/current reference limit. */
#define REGEN_FADE_ENABLE		1
#if HIRANGE_XR || HILOAD_XR
#define REGEN_FADE_START_RPM	2000.0f	/* full regen at/below this        */
#define REGEN_FADE_END_RPM		3000.0f	/* zero commanded regen above this */
#endif
#if HIRANGE_TR
#define REGEN_FADE_START_RPM	2240.0f	/* full regen at/below this (76.8V) */
#define REGEN_FADE_END_RPM		2800.0f	/* zero commanded regen above this */
#endif

/* Throttle */
#define THR_MIN_VAL			0.8f
#define THR_MAX_VAL			3.8f
#define THR_HYS_BAND		0.1f
#define MIN_RPM_CHANGE_LIM	10.0f

/* Pedal progressivity (expo) per mode, applied in ThrottleMap_GetTorque:
 *   torque = cap * ((1-k)*x + k*x*x),  x = normalized pedal [0..1]
 * k = 0 -> linear, k = 1 -> full quadratic. WOT (x = 1) is unchanged at any
 * k, so peak torque and gradeability are unaffected — only partial-pedal
 * sensitivity is reshaped. Half-pedal fraction of cap = 0.5 - 0.25k
 * (e.g. XR SPORTS k = 0.95 -> 26% of the 40 Nm cap ~= 10.5 Nm). */
#if HIRANGE_XR
#define THR_EXPO_ECO		0.0f
#define THR_EXPO_SPORTS		0.95f
#define THR_EXPO_REVERSE	0.95f
#endif
#if HILOAD_XR
#define THR_EXPO_ECO		0.0f
#define THR_EXPO_SPORTS		0.8f
#define THR_EXPO_REVERSE	0.0f
#endif
#if HIRANGE_TR
#define THR_EXPO_ECO		0.0f	/* seeded = HiRange XR */
#define THR_EXPO_SPORTS		0.95f
#define THR_EXPO_REVERSE	0.95f
#endif

/* Throttle-request rate limit (torqueRate in main.c) — THE launch-feel
 * knob. Applied to the driver torque request BEFORE the torque-management
 * caps; timebase is ISR ticks, so the Nm/s here is honest (the downstream
 * command slew's DWT dt floors at 100 us and can under-read). Must stay
 * BELOW TRQ_SLEW_ACCEL_* (Torque_Management.h) so this limiter is the
 * binding one for driver transients — enforced by a static assert there.
 * XR 25.0 Nm/s: 0->40 Nm in 1.6 s. Raised from the original 5.0 Nm/s (8 s
 * gentle launch) for a sportier pickup; the command slew (TRQ_SLEW_ACCEL_*)
 * and the FOC-model ramp (FOC_TRQ_RAMP_*) were raised in lockstep so the
 * request limiter stays the binding layer. Raise for snappier, lower gentler. */
#if HIRANGE_XR
#define TRQ_REQ_SLEW_UP_NM_S	25.0f
#define TRQ_REQ_SLEW_DOWN_NM_S	50.0f
#endif
#if HILOAD_XR
#define TRQ_REQ_SLEW_UP_NM_S	30.0f
#define TRQ_REQ_SLEW_DOWN_NM_S	100.0f
#endif
#if HIRANGE_TR
#define TRQ_REQ_SLEW_UP_NM_S	5.0f	/* seeded = HiRange XR */
#define TRQ_REQ_SLEW_DOWN_NM_S	100.0f
#endif

/* ---- Dynamic commutation-angle advance (applied in isr_foc_start) ---------
 * The rotor-alignment offset that is optimal at standstill OVER-LAGS at speed:
 * the ISR->PWM latency (~1.5 control cycles) lets the true rotor advance past
 * the applied voltage vector by ~w_e*dt, and high current shifts the angle
 * further. Correcting it (REDUCING the offset -> advancing the commanded angle)
 * recovered the whole 26-45 km/h noise band, the accel pull-back, and top speed
 * (driver proved a fixed -0.1 rad-mech did it; 0.1 rad-mech = 0.3 rad-elec ~=
 * 1.6 cycles of latency at 6000 RPM).
 *
 * A FIXED advance jerks the vehicle on hard decel, so it is GATED to apply only
 * while MOTORING at speed:
 *   advance = ADV_MECH_MAX * ramp(|RPM|; LO..HI) * gate(d.torque_final; LO..HI)
 * - RPM ramp: zero at launch (below ADV_RPM_LO), full across the band + top.
 * - torque gate: a near-sign gate on the driver drive-torque -> full while
 *   driving, backs off to the calibrated offset on release/braking (kills the
 *   decel jerk). Calibrated for 0.1 rad-mech at SPORTS_MAX_SPEED.
 * Set FOC_ANGLE_ADVANCE 0 to disable and use the raw calibrated offset.
 *
 * PER MOTOR PROFILE, not per vehicle. The advance corrects an angle error, and
 * both how big that error is (pole pairs, the high-current angle shift) and
 * whether it hurts (how close the machine runs to its voltage limit) are
 * properties of the motor and its pack, not of the chassis. It is DISABLED on
 * LivGuard: that machine showed none of the symptoms this fixes — no field-
 * weakening trouble, no 26-45 km/h noise, no pull-back or accel slowdown — so
 * there is nothing to correct and applying an uncalibrated 0.1 rad-mech
 * advance to it would inject the very error this removes elsewhere. The
 * ADV_* values below are calibrated for the DEFAULT profile (HiRange XR,
 * 96 V) and have not been characterised on any other machine.
 *
 * Gated on the profile rather than on HILOAD_XR. The two are equivalent in
 * practice — HiLoad XR always runs LivGuard, and the pairing guard further up
 * now enforces that both ways — but the profile is the correct thing to key
 * on, because it is the motor and its pack that decide whether an advance is
 * needed at all. If another machine is ever added, it inherits the advance by
 * default and has to be characterised, rather than silently opting out. */
#if defined(MOTOR_PROFILE) && (MOTOR_PROFILE == MOTOR_PROFILE_LIVGUARD)
#define FOC_ANGLE_ADVANCE	0	/* LivGuard: symptom absent — see above */
#elif defined(MOTOR_PROFILE) && (MOTOR_PROFILE == MOTOR_PROFILE_HIRANGE_TR)
#define FOC_ANGLE_ADVANCE	0	/* TW-7 bench: replaced by FOC_VOUT_DELAY_COMP below */
#else
#define FOC_ANGLE_ADVANCE	1
#endif

/* ---- Output-voltage delay compensation (FOC_Loop.c inverse Park) ----------
 * The voltage computed from the currents sampled in this ISR lands on the
 * motor about one PWM period later, by which time the rotor has turned
 * w_e * FOC_VOUT_DELAY_S further. Uncompensated, the applied vector lags by
 * that angle: Vd reads ~ -Vq * tan(w_e * delay) below base speed, and the
 * error reaches ~25 deg_e at 10000 rpm on the 5-pole-pair TW-7.
 *
 * Unlike FOC_ANGLE_ADVANCE this advances ONLY the inverse-Park angle — the
 * currents stay in the frame they were sampled in — and it is proportional to
 * signed speed with no torque gate, so it is correct in speed mode, at light
 * load, and in either direction (d.omega_e carries the sign).
 *
 * BENCH-MEASURED on the TW-7 dyno (Bringup_gen4 logs, 7000_rpm_run_3): Vd vs
 * Vq at Id ~ -1 A from 1000-4300 rpm gives a constant ~85 us (0.85 PWM
 * periods). Verify after flashing: below base speed at light load, Vd should
 * fall to ~ -w_e*Lq*Iq (< 1 V at 4000 rpm) instead of ~ -5 V. */
#if defined(MOTOR_PROFILE) && (MOTOR_PROFILE == MOTOR_PROFILE_HIRANGE_TR)
#define FOC_VOUT_DELAY_COMP	1
#else
#define FOC_VOUT_DELAY_COMP	0
#endif
#define FOC_VOUT_DELAY_S	85.0e-6f	/* s, measured ISR->PWM latency */

/* ---- FOC bus voltage from the measured DC link (Sensor_Disconnection_Checks.c)
 * Default: the FOC voltage model is floored at OP_VOLTAGE (raised only by the
 * BMS pack voltage), so duty normalisation (Vmax_SVM), the current-PI clamps
 * and the FW voltage target all assume >= 48 V whatever the link really is.
 * On the dyno supply the link swings 44-53 V at 8.7-9 krpm; at the voltage
 * edge the drive then applies ~8 % less voltage than it thinks on every sag,
 * which fed the 2 Hz speed/torque limit cycle in 8000_revert.
 *
 * With FOC_VBUS_FROM_VDC the model instead uses a first-order low-pass of the
 * sensed link (1 kHz update, FOC_VDC_LPF_TAU_S), floored at the UV threshold
 * and CAPPED AT OP_VOLTAGE. Duty is then normalised to the real link and FW
 * weakens more when it sags. The cap matters on a supply that cannot sink
 * regen: uncapped, a regen spike (28_10/first: 50 -> 61 V) raised the FW
 * voltage target, FW released 25 A of Id in one frame, the back-EMF braked
 * harder, and the link ran away to 90 V (OV trip). Set OP_VOLTAGE at or just
 * below the supply setpoint. Bench only for now: the vehicle build pins OP_VOLTAGE on
 * purpose (see the comment at the model_busv assignment) — feeding the real
 * pack lowers top speed as SOC drops. Needs NEW_VOLT_DIV/OLD_VOLT_DIV set
 * correctly, since a wrong divider now mis-scales the FOC, not just the trips. */
#if defined(MOTOR_PROFILE) && (MOTOR_PROFILE == MOTOR_PROFILE_HIRANGE_TR)
#define FOC_VBUS_FROM_VDC	1
#else
#define FOC_VBUS_FROM_VDC	0
#endif
#define FOC_VDC_LPF_TAU_S	0.010f	/* s; ~16 Hz corner, passes the 2 Hz swing */

/* ---- Circular |Vd,Vq| limit + anti-windup (FOC_Loop.c hand-edit)
 * The generated model clamps Vd and Vq to +/-Vbus/sqrt3 EACH, so the pair can
 * reach sqrt2 x the SVM circle; apply_SVM_voltages() then cuts it per phase
 * and the current PIs never learn the voltage was not there. Their back-calc
 * anti-windup also sees only the PI's own clamp, not the clamp after the
 * feed-forward is added. At 9 krpm (27_10/8500_oscillation) the demanded
 * index reached 1.03-1.10, the integrators wound up, and the FW loop lost Id
 * control: Vq went negative, Id/Iq passed 100 A, OV trip.
 *
 * With FOC_VDQ_CIRCLE_LIMIT the final pair is shortened to the circle
 * (radius d.Vmax_SVM) with d-axis priority (Vdq_Circle_Limit.h), and both
 * integrators back-calculate against the final limited voltage minus the
 * feed-forward, with the existing ID/IQ_PID_BACK_KAW. 0x715 byte1 keeps
 * reporting the DEMANDED index (pre-limit, can exceed 1.00) and byte3 becomes
 * the clip ratio (% of 10 kHz cycles the limit was active) on this build.
 * Bench only for now. Future work, not built: q-axis priority (keep torque,
 * give up flux) as an alternative for below-base-speed transients.
 *
 * FOC_VDQ_LIMIT_VD_MAX_FRAC: when the pair is shortened, |Vd| is held to this
 * fraction of the radius, so Vq always keeps sqrt(1 - f^2) of it (0.9 ->
 * 44 %). Pure d priority would set Vq = 0 whenever Vd sits on its rail, which
 * at speed is a hard brake into the link. Same idea as the MaxVd limit in
 * ST's MCSDK circle limitation. The closed-loop FW regulator reads the
 * DEMANDED magnitude (d.vdq_demand), not the limited one, so it still sees
 * how far outside the circle the demand is. */
#if defined(MOTOR_PROFILE) && (MOTOR_PROFILE == MOTOR_PROFILE_HIRANGE_TR)
#define FOC_VDQ_CIRCLE_LIMIT	1
#else
#define FOC_VDQ_CIRCLE_LIMIT	0
#endif
#define FOC_VDQ_LIMIT_VD_MAX_FRAC	0.9f

/* ---- Current-sense delay compensation (FOC_Loop.c hand-edit)
 * Above base speed the measured dq current is rotated against the voltage
 * frame by ~w_e * 170-200 us (REPORT.md 4.1; 28_10/first: 41-43 deg at 8000
 * rpm). On the free-running bench rotor that reads as Iq ~56 A where the
 * voltage equations put it at ~12 A, corrupts the decoupling feed-forward,
 * and turns every FW move into a torque step. The firmware reads currents
 * and encoder back to back, so the lag is upstream (current-sense path).
 *
 * With FOC_ISENSE_DELAY_COMP the Park transform uses theta - w_e * tau, i.e.
 * the angle the rotor had when the currents were really sampled. tau is set
 * LIVE over CAN 0x115 (u16 LE, microseconds, clamped to FOC_ISENSE_DELAY_MAX_S)
 * and reported back in 0x406 bytes 0-1; it boots at FOC_ISENSE_DELAY_S.
 * Tune at a 6000 rpm hold until measured Iq falls to the ~5 A loss level.
 * Needs FOC_VOUT_DELAY_COMP: the shifted sin/cos are only safe because that
 * hand-edit recomputes them for the inverse Park. */
#if defined(MOTOR_PROFILE) && (MOTOR_PROFILE == MOTOR_PROFILE_HIRANGE_TR)
#define FOC_ISENSE_DELAY_COMP	1
#else
#define FOC_ISENSE_DELAY_COMP	0
#endif
#define FOC_ISENSE_DELAY_S		0.0f		/* s, boot value; tuned live */
#define FOC_ISENSE_DELAY_MAX_S	400.0e-6f	/* s, CAN-set ceiling */
#if FOC_ISENSE_DELAY_COMP && !FOC_VOUT_DELAY_COMP
#error "FOC_ISENSE_DELAY_COMP needs FOC_VOUT_DELAY_COMP (it relies on the inverse Park recomputing sin/cos)"
#endif

/* Bench fast telemetry: every FAST_TLM_PERIOD_MS, the means over that window
 * of Id_ref, Iq_ref, Id, Iq (ext 0x420), Vd, Vq, Vdc plus the instantaneous
 * rpm (ext 0x421) and the Vd/Vq feed-forward: Vd_FF, Vq_FF (both as applied,
 * i.e. clamped), Vq_FF before the clamp and the clamp limit 0.577*Vbus
 * (ext 0x422, captured inside FOC_Loop.c by a HAND-EDIT); int16 LE, 0.01 A /
 * 0.01 V / 1 rpm. Off at boot; CAN 0x116 byte0 = 1 turns it on, 0 off. The
 * 10 Hz frames alias anything above 5 Hz: the ~9 Hz speed-loop mode, and
 * whether Id/Iq follow the ~3.8 Hz reference alternation. ~85 kbit/s when on
 * (~46 % of the 250 kbit/s bus with the slow frames). The 10 Hz ext 0x40A
 * (Z-pulse count_at_z, z_count, boot Count_From_Duty) rides along. */
#if defined(MOTOR_PROFILE) && (MOTOR_PROFILE == MOTOR_PROFILE_HIRANGE_TR)
#define FOC_BENCH_FAST_TLM		1
#else
#define FOC_BENCH_FAST_TLM		0
#endif
#define FAST_TLM_PERIOD_MS		5			/* ms, window and send period */
#define ADV_MECH_MAX		0.1f	/* max advance, rad mech (driver-proven at top speed)   */
#define ADV_RPM_LO			500.0f	/* advance ramps in above this |RPM|                    */
#define ADV_RPM_HI			2350.0f	/* full advance above this |RPM| (covers band + top)    */
#define ADV_TRQ_LO			7.0f	/* motoring gate: advance off below this drive torque Nm */
#define ADV_TRQ_HI			15.0f	/* full advance above this drive torque Nm              */

#if HILOAD_XR
/* Hill hold — speed PI that stops rollback while a direction is engaged
 * and the brake is released (see Hill_Hold.c). Speeds are motor mech RPM
 * (~0.01 km/h per RPM). KP/KI are starting points — tune on vehicle. */
#define HILL_HOLD_ENABLE	1
#define HH_ENTRY_RPM		80.0f	/* rollback speed that engages the hold      */
#define HH_EXIT_RPM			150.0f	/* fwd speed for driver-takeover release     */
#define HH_DEADBAND_RPM		15.0f	/* |speed| below this = held: integ freezes  */
#define HH_UNWIND_GAIN		1.0f	/* integ unwind multiplier on slow fwd creep */
#define HH_UNWIND_MAX_RPM	60.0f	/* unwind only in (CREEP, this): above = a
									 * transient (blip coast / arrest overshoot),
									 * freeze the estimate instead of dumping it  */
#define HH_CREEP_RPM		5.0f	/* forward-trim floor. Below HH_DEADBAND_RPM
									 * on purpose: the arrest surplus that makes
									 * the vehicle creep forward sits under the
									 * wind deadband, so trim must reach lower to
									 * remove it. Asymmetric — the backward side
									 * still rejects noise at HH_DEADBAND_RPM.    */
#define HH_KP				0.15f	/* Nm per RPM of rollback                    */
#define HH_KI				1.7f	/* Nm per (RPM*s)                            */
#define HH_MAX_TORQUE		55.0f	/* hold torque ceiling (Nm)                  */
#define HH_MAX_HOLD_MS		5000U	/* max continuous hold; 0 = unlimited        */
#define HH_RELEASE_RATE_NM_S 8.0f	/* torque bleed-off rate after timeout       */
#define HH_SLEW_NM_S		200.0f	/* max hold-torque rate of change (Nm/s)     */
#define HH_STANDBY_DECAY_NM_S 5.0f	/* grade-estimate decay while driver drives  */
#define HH_TS				0.001f	/* PI update period (1 kHz)                  */
#endif

#if HIRANGE_XR
/* Hill hold — speed PI that stops rollback while a direction is engaged
 * and the brake is released (see Hill_Hold.c). Speeds are motor mech RPM
 * (~0.01 km/h per RPM). KP/KI are starting points — tune on vehicle. */
#define HILL_HOLD_ENABLE	0
#define HH_ENTRY_RPM		100.0f	/* rollback speed that engages the hold      */
#define HH_EXIT_RPM			150.0f	/* fwd speed for driver-takeover release     */
#define HH_DEADBAND_RPM		15.0f	/* |speed| below this = held: integ freezes  */
#define HH_UNWIND_GAIN		1.0f	/* integ unwind multiplier on slow fwd creep */
#define HH_UNWIND_MAX_RPM	60.0f	/* unwind only in (CREEP, this): above = a
									 * transient (blip coast / arrest overshoot),
									 * freeze the estimate instead of dumping it  */
#define HH_CREEP_RPM		5.0f	/* forward-trim floor. Below HH_DEADBAND_RPM
									 * on purpose: the arrest surplus that makes
									 * the vehicle creep forward sits under the
									 * wind deadband, so trim must reach lower to
									 * remove it. Asymmetric — the backward side
									 * still rejects noise at HH_DEADBAND_RPM.    */
#define HH_KP				0.05f	/* Nm per RPM of rollback                    */
#define HH_KI				1.1f	/* Nm per (RPM*s)                            */
#define HH_MAX_TORQUE		23.0f	/* hold torque ceiling (Nm)                  */
#define HH_MAX_HOLD_MS		5000U	/* max continuous hold; 0 = unlimited        */
#define HH_RELEASE_RATE_NM_S 8.0f	/* torque bleed-off rate after timeout       */
#define HH_SLEW_NM_S		200.0f	/* max hold-torque rate of change (Nm/s)     */
#define HH_STANDBY_DECAY_NM_S 5.0f	/* grade-estimate decay while driver drives  */
#define HH_TS				0.001f	/* PI update period (1 kHz)                  */
#endif

#if HIRANGE_TR
/* Hill Hold is intentionally disabled on HiRange TR. Keep the tuning
 * constants defined because Hill_Hold.c is still compiled into both apps. */
#define HILL_HOLD_ENABLE	0
#define HH_ENTRY_RPM		100.0f
#define HH_EXIT_RPM			150.0f
#define HH_DEADBAND_RPM		15.0f
#define HH_UNWIND_GAIN		1.0f
#define HH_UNWIND_MAX_RPM	60.0f
#define HH_CREEP_RPM		5.0f
#define HH_KP				0.05f
#define HH_KI				1.1f
#define HH_MAX_TORQUE		23.0f
#define HH_MAX_HOLD_MS		5000U
#define HH_RELEASE_RATE_NM_S 8.0f
#define HH_SLEW_NM_S		200.0f
#define HH_STANDBY_DECAY_NM_S 5.0f
#define HH_TS				0.001f
#endif

/* Hill hold reads the driver's demand from d.torque_final, which only the
 * THROTTLE_BASED_REF superloop path maintains. Under CAN_BASED_REF there is
 * no driver-demand variable separate from RefTrq, and feeding the module its
 * own output turns the max() floor into a torque ratchet (see Hill_Hold.h). */
#if HILL_HOLD_ENABLE && !THROTTLE_BASED_REF
#error "vehicle_config.h: HILL_HOLD_ENABLE requires THROTTLE_BASED_REF"
#endif

#endif /* __VEHICLE_CONFIG_H */
