/**
 * @file    torque_management.h
 * @brief   Torque scaling based on speed and temperature derating
 *          for Pegasus 96V Motor Controller
 *
 * @note    DriveMode enum is NOT redefined here.
 *          Uses power_mode_t { ECO = 0, SPORTS = 1 } from main.h.
 */

#ifndef TORQUE_MANAGEMENT_H
#define TORQUE_MANAGEMENT_H

#include <math.h>
#include "main.h"   /* power_mode_t, project-wide defines */
/* =========================================================================
 * MACROS
 * ========================================================================= */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define RPM_TO_RADS(x)      ((x) * 2.0f * (float)M_PI / 60.0f)
#define RADS_TO_RPM(x)      ((x) * 60.0f / (2.0f * (float)M_PI))

/* =========================================================================
 * TORQUE-SPEED CHARACTERISTIC LOOKUP TABLE
 * ========================================================================= */

/**
 * Number of points in the torque-speed characteristic table.
 * Add or remove points to match your motor datasheet curve.
 */
#define TORQUE_SPEED_TABLE_SIZE     14U

/**
 * @brief  Torque-speed characteristic lookup table entry.
 *
 * Populate TORQUE_SPEED_TABLE in torque_management.c with actual values.
 * Points MUST be in ascending speed order.
 *
 * Example curve (replace with actual motor data):
 *    500 RPM → 80 Nm
 *   1000 RPM → 75 Nm
 *   1500 RPM → 68 Nm
 *   2000 RPM → 55 Nm
 *   2500 RPM → 45 Nm
 *   3500 RPM → 35 Nm
 *   4500 RPM → 25 Nm
 *   6000 RPM → 15 Nm
 */
typedef struct {
    float speed_rpm;        /* Speed breakpoint (RPM)            */
    float max_torque_nm;    /* Maximum torque at this speed (Nm) */
} TorqueSpeedPoint_t;

/* =========================================================================
 * MODE SPEED LIMITS
 * ========================================================================= */

#define SPEED_LIMIT_ECO_RPM         ECO_MAX_SPEED
#define SPEED_LIMIT_SPORTS_RPM      SPORTS_MAX_SPEED
#define SPEED_LIMIT_REVERSE_RPM     REV_MAX_SPEED   /* Fixed — ECO/SPORTS ignored in REVERSE */

/* =========================================================================
 * TEMPERATURE DERATING CONFIGURATION
 * ========================================================================= */

#define MOTOR_TEMP_DERATE_START     130.0f
#define MOTOR_TEMP_DERATE_END       150.0f

#define CTRL_TEMP_DERATE_START      75.0f
#define CTRL_TEMP_DERATE_END        80.0f

/* IIR smoothing on sampled temperatures; lower alpha -> slower response */
#define TEMP_IIR_ALPHA              0.01f
/* Hysteresis below derate_start that a locked state must cool past to release */
#define TEMP_HYSTERESIS_C           2.0f
/* Power floor when temp derate is fully engaged (blended from mode Pmax) */
#define MIN_POWER_AT_FULL_DERATE_W  2000.0f

/* =========================================================================
 * PER-MODE TORQUE SCALE
 * Applied multiplicatively to the envelope before the vehicle_config hard
 * clamps (ECO_MAX_TORQUE / SPORTS_MAX_TORQUE).
 * ========================================================================= */

#if HIRANGE_XR
#define TORQUE_SCALE_ECO            0.529f
#define TORQUE_SCALE_SPORTS         1.0f
#endif

#if HILOAD_XR
#define TORQUE_SCALE_ECO            0.5f
#define TORQUE_SCALE_SPORTS         1.0f
#endif
#if HIRANGE_TR
#define TORQUE_SCALE_ECO            0.529f
#define TORQUE_SCALE_SPORTS         1.0f
#endif
/* =========================================================================
 * TIME-LIMITED POWER BOOST (overboost) — CURRENTLY INERT
 *
 * With PWR_CONT_W == PWR_BOOST_W == SPORTS_MAX_POWER the boost gate in
 * apply_torque_scaling (nominal_max_power_w > PWR_CONT_W) never opens:
 * every mode runs at its nominal Pmax, and the budget below updates but
 * caps nothing. The mechanism is kept for re-enabling later.
 *
 * To re-enable: set PWR_CONT_W to the motor's continuous rating (4.6 kW on
 * the spec sheet) and PWR_BOOST_W to the short-term ceiling. The budget
 * models above-continuous thermal capacity: it depletes while delivered
 * power exceeds PWR_CONT_W and refills below it; when empty, SPORTS/REVERSE
 * fall back to PWR_CONT_W until it recovers past PWR_BOOST_REARM_J
 * (hysteresis stops chattering). ECO (< PWR_CONT_W) is never boosted; the
 * motor-temp derate downstream remains the hard backstop.
 *
 * PWR_BOOST_E_MAX_J sizing came from the motor spec point:
 *   (11.3 kW - 4.6 kW) * 180 s = 1.206 MJ of above-continuous energy.
 * PWR_BOOST_W MUST stay <= MAX_POWER (the FOC current-ref / FWC power
 * limit) or the FOC estimator clips the boost regardless of this cap —
 * enforced by the _Static_assert below.
 * ========================================================================= */

#define PWR_BOOST_ENABLE            1
#define PWR_CONT_W                  SPORTS_MAX_POWER /* motor continuous rating       */
#define PWR_BOOST_W                 SPORTS_MAX_POWER /* boost ceiling = SPORTS peak    */
#define PWR_BOOST_E_MAX_J           1206000.0f       /* above-continuous thermal cap  */
#define PWR_BOOST_REARM_J           (0.5f * PWR_BOOST_E_MAX_J)  /* re-arm at 50%      */

/* Invariant: the boost ceiling must not exceed the FOC estimator's power limit,
 * else the current-ref / FWC model clips the boost regardless of this cap.
 * (floating constants as immediate cast operands are a valid ICE in C11). */
_Static_assert((int)(PWR_BOOST_W) <= (int)(MAX_POWER),
               "PWR_BOOST_W (SPORTS_MAX_POWER) must be <= MAX_POWER");

/* =========================================================================
 * SPEED LIMITER — PI controller with anti-windup (torque-domain)
 *
 * Replaces the old open-loop taper. Acts on the speed error (vmax - speed)
 * and outputs a torque CAP. The integrator converges to the steady torque
 * that holds the vehicle at vmax (e.g. coasting down a flyover), which kills
 * the cut-to-zero / slow-recovery limit cycle the taper produced: on a
 * downgrade the taper could only freewheel (0 Nm), so speed fell well below
 * the limit before drive returned, then overshot — repeatedly.
 *
 * Output is floored at 0 Nm (no regen here): the downstream FOC model owns
 * regen via its own state machine, so we never command a negative RefTrq. On
 * a grade shallow enough that some positive torque holds vmax (the usual case,
 * incl. the reported flyover) the PI holds speed cleanly; on a grade so steep
 * that even 0 Nm can't, the vehicle settles at the drag-limited speed without
 * oscillating (the integrator parks at 0 via anti-windup).
 *
 * Tuning — start conservative:
 *   KP : tighter speed hold / faster reaction to overspeed (too high -> buzz)
 *   KI : faster convergence to the holding torque   (too high -> overshoot)
 * ========================================================================= */
#if HIRANGE_XR
/* Retuned 2026-07-20 for the fast-recovery slew architecture, then settled
 * on-vehicle. The old 0.09/2.4 was tuned against the 2 Nm/s command slew's
 * recovery lag; with that lag gone the high KI made the loop severely
 * underdamped (zeta~0.15, integrator traversing the ~5 Nm ECO cap in ~20 ms)
 * -> sustained +/-1 km/h limit cycle riding the ECO 4500 RPM cap.
 *
 * A bench second-order model (frictionless J*s^2 + KP*s + KI) puts zeta ~0.75
 * at 0.15/0.25, but on the real driveline the extra damping (aero/rolling drag,
 * load, the DWT-timed torque slew, DCLO) makes the loop far better damped, so
 * the VEHICLE-VALIDATED gains are 0.05/0.25: stable at the limit with negligible
 * push-back (confirmed on vehicle 2026-07-30). Do NOT raise KP back toward the
 * bench 0.15 to "fix" push-back without vehicle evidence — that figure
 * over-predicts the damping the hardware actually needs.
 * If it ever hunts: lower KI toward 0.15. If it overshoots on downgrades: nudge
 * KP up (watch for torque buzz from speed noise above ~0.25). */
#define SPEED_PI_KP                 0.05f    /* Nm per RPM of overspeed         */
#define SPEED_PI_KI                 0.25f    /* Nm per (RPM*s) — integral rate  */
#endif
#if HILOAD_XR
#define SPEED_PI_KP                 0.05f    /* Nm per RPM of overspeed         */
#define SPEED_PI_KI                 0.20f    /* Nm per (RPM*s) — integral rate  */
#endif
#if HIRANGE_TR
/* Seeded from the vehicle-validated HiRange XR gains (0.05/0.25). The plant
 * gain (motor-RPM/s per Nm) differs on the 76.8 V / 384 A driveline, so
 * re-check damping on the TR vehicle; use the zeta sizing method if it hunts. */
#define SPEED_PI_KP                 0.05f    /* Nm per RPM of overspeed         */
#define SPEED_PI_KI                 0.25f    /* Nm per (RPM*s) — integral rate  */
#endif

/* Speed-cap easing on a mode DOWNGRADE. Switching to a mode whose speed cap
 * sits BELOW the current speed (SPORTS -> ECO above ECO's cap, at full throttle)
 * used to enforce the new cap in one tick: SPEED_PI_KP * overspeed knocked the
 * drive-torque cap down instantly and the command slewed to ~0 — a lurch when
 * the driver is expecting thrust. Instead the EFFECTIVE cap is eased down from
 * the current speed to the new cap at this rate, so the drive bleeds off
 * smoothly and the car settles onto the lower cap. Upgrades apply immediately.
 * 500 rpm/s ≈ 5.2 km/h/s (~0.15 g) on HiRange XR — firm but civil; lower it for
 * a gentler settle. Not per-variant: it is a comfort rate, not a plant gain. */
#define SPEED_CAP_SLEW_DOWN_RPM_S   500.0f

/* Mode-downgrade DEFERRAL above the voltage-limited top speed.
 *
 * The easing above is sufficient on level ground for a reason that is not
 * obvious: the mode the driver is switching TO still has voltage headroom
 * there. At 47 km/h ECO sits at ~75% modulation while SPORTS was >100%, so the
 * current-loop integrators — latched by over-modulation in SPORTS — land
 * somewhere they can unwind, and the loop recovers.
 *
 * Downhill that stops being true. Gravity pushes the vehicle past the flat
 * voltage-limited top speed, and up there ECO is saturated too: ~92%
 * modulation at 60 km/h, ~99% at 65. The downgrade then moves the loop from one
 * saturated operating point to another with no headroom to unwind, the slope
 * holds it there, and the result is a hard grab followed by a sustained limit
 * cycle. Slew shaping cannot fix that — the problem is the destination, not the
 * rate.
 *
 * So above this speed a downgrade is HELD until the speed is back inside the
 * requested mode's own cap, where that mode is controllable again. Set ABOVE
 * the flat voltage-limited top speed so the branch is unreachable on level
 * ground and the vehicle-validated flat behaviour (488fe53) is untouched — this
 * only governs the overspeed region no tuning has ever visited. Applies to the
 * regen fade as well, which incidentally removes the ECO regen cap step
 * (0 -> REGEN_MAX_TORQUE_ECO) at high speed without reintroducing the onset
 * ramp that regressed the SPORTS fixes.
 *
 * Per-variant because it is a physical property of the pack/motor, not a
 * comfort rate. HiRange XR tops out at 55-56 km/h (~5270-5365 rpm) on the flat,
 * so 5500 rpm (~57.4 km/h) clears it. */
#if HIRANGE_XR
#define MODE_DOWNGRADE_DEFER_RPM    5500.0f
#endif
#if HILOAD_XR
/* Above SPORTS_MAX_SPEED (4850), so only reachable downhill. Flat top speed on
 * this variant is not yet measured — re-check once it is. */
#define MODE_DOWNGRADE_DEFER_RPM    4900.0f
#endif
#if HIRANGE_TR
/* Seeded from HiRange XR; confirm against the TR's voltage-limited top speed. */
#define MODE_DOWNGRADE_DEFER_RPM    5500.0f
#endif

/* =========================================================================
 * TORQUE SLEW (Nm / s) — applied to the final command with DWT-measured dt.
 *
 * Role split with the REQUEST limiter (torqueRate in main.c, rates from
 * TRQ_REQ_SLEW_* in vehicle_config.h):
 *   - request limiter = launch feel. Binding for driver transients, runs
 *     on the honest ISR-tick timebase. Tune drivability THERE.
 *   - this command slew = protection-release smoothing only: how fast
 *     torque returns when the speed-PI releases after a crest (too slow
 *     -> downhill speed dip), when DCLO lets go, or when a temp derate
 *     eases. Fast enough not to color driving feel, slow enough that a
 *     cap release is a ramp, not a step.
 * Ordering: TRQ_REQ_SLEW_UP_NM_S must stay below both ACCEL rates or the
 * request limiter goes inert (enforced by the _Static_assert below).
 *
 * dt here is floored at DT_MIN_S (100 us): a superloop faster than that
 * inflates the effective rate by (100 us / actual period). With this slew
 * non-binding for feel, the distortion no longer affects drivability
 * tuning — it only makes protection recovery somewhat quicker.
 *
 * ACCEL also gates how fast drive torque overtakes a hill hold before its
 * HH_MAX_HOLD_MS timeout (the request limiter's ramp dominates there).
 * Decel stays fast for a responsive throttle-release cut.
 * ========================================================================= */
#if HIRANGE_XR
/* Raised 40 -> 80 in lockstep with the request limiter's throttle-response
 * retune (TRQ_REQ_SLEW_UP_NM_S 5 -> 25); still well above the 25 Nm/s request
 * rate, so it stays non-binding for driver feel and only smooths cap-release. */
#define TRQ_SLEW_ACCEL_ECO_NM_S     80.0f
#define TRQ_SLEW_ACCEL_SPORTS_NM_S  40.0f
#endif
#if HILOAD_XR
#define TRQ_SLEW_ACCEL_ECO_NM_S     40.0f
#define TRQ_SLEW_ACCEL_SPORTS_NM_S  80.0f
#endif
#if HIRANGE_TR
#define TRQ_SLEW_ACCEL_ECO_NM_S     40.0f	/* seeded = HiRange XR */
#define TRQ_SLEW_ACCEL_SPORTS_NM_S  40.0f
#endif
#define TRQ_SLEW_DECEL_NM_S         50.0f

/* Speed-dependent DECEL rate in SPORTS.
 *
 * Above ~37 km/h (3550 rpm) SPORTS commands a voltage vector OUTSIDE the
 * achievable SVM circle — ~100% modulation there, ~113% at 55 km/h, ~119% at
 * 65. The per-axis clamps are +/-Vmax each so the pair passes them, the
 * per-phase clamp then truncates it, and the anti-windup back-calculation
 * cannot see that truncation, so both current-loop integrators wind to their
 * rails holding an error that can never be satisfied. A fast release then asks
 * them to unwind at only Ki*err — roughly 100 ms on Iq, 215 ms on Id — during
 * which the applied voltage bears no relation to the demand. That is the
 * throttle-release grab, and downhill the speed does not fall so it does not
 * self-clear.
 *
 * The empirical constraint: a GRADUAL release never triggers it (reported
 * repeatedly on the vehicle). Small error keeps the integrators tracking and the
 * loop stays in its linear region. So ease the decel rate down across the
 * over-modulation band instead of cutting at the flat-ground rate. 10 Nm/s
 * takes ~1.1 s to clear the ~10.9 Nm available at 5300 rpm — inside the
 * "gradual" regime that is known to be safe, where 50 Nm/s is not.
 *
 * This is independent of the root cause: if the loop is fixed properly
 * (circular limit + anti-windup that sees the final applied voltage) this
 * becomes unnecessary, and it can be reverted by setting TRQ_DECEL_HS_ENABLE 0.
 * That flag is also the on-vehicle A/B.
 *
 * SCOPE — deliberately narrow. Applied only when ALL of these hold:
 *   - SPORTS (the effective mode; ECO never exceeds ~90% modulation, and its
 *     deceleration feel is deliberate),
 *   - the DRIVER is the binding constraint. A reduction from the cap (speed
 *     limit, thermal derate) or from DCLO is PROTECTIVE and keeps the full
 *     declared cut rate,
 *   - the brake is NOT applied. A brake command arrives as torque_ref = 0 and
 *     must never be slewed.
 *
 * ENABLE is per-variant and OFF for HiLoad: its modulation-vs-speed curve has
 * not been measured (different lambda and pack), so the band below would be a
 * guess. Measure with 0x715 byte[1] first, then set the band and enable. */
#if HIRANGE_XR
#define TRQ_DECEL_HS_ENABLE         1
#define TRQ_SLEW_DECEL_HS_NM_S      10.0f   /* decel rate at/above END_RPM      */
#define TRQ_DECEL_HS_START_RPM      3500.0f /* ~37 km/h: over-modulation onset  */
#define TRQ_DECEL_HS_END_RPM        5300.0f /* ~55 km/h: flat top speed         */
#endif
#if HILOAD_XR
#define TRQ_DECEL_HS_ENABLE         0       /* modulation curve not yet measured */
#define TRQ_SLEW_DECEL_HS_NM_S      10.0f
#define TRQ_DECEL_HS_START_RPM      3500.0f
#define TRQ_DECEL_HS_END_RPM        4800.0f
#endif

#if TRQ_DECEL_HS_ENABLE
_Static_assert((int)(TRQ_SLEW_DECEL_HS_NM_S) <= (int)(TRQ_SLEW_DECEL_NM_S),
               "TRQ_SLEW_DECEL_HS_NM_S must be <= TRQ_SLEW_DECEL_NM_S (it slows the cut, never speeds it up)");
_Static_assert((int)(TRQ_DECEL_HS_START_RPM) < (int)(TRQ_DECEL_HS_END_RPM),
               "TRQ_DECEL_HS_START_RPM must be < TRQ_DECEL_HS_END_RPM (the ramp divides by END-START)");
_Static_assert((int)(TRQ_SLEW_DECEL_HS_NM_S) > 0,
               "TRQ_SLEW_DECEL_HS_NM_S must be > 0 or a driver release can never reach zero torque");
#endif

/* Request limiter must be the binding one for driver transients.
 * (floating constants as immediate cast operands are a valid ICE in C11) */
_Static_assert((int)(TRQ_REQ_SLEW_UP_NM_S) <= (int)(TRQ_SLEW_ACCEL_ECO_NM_S) &&
               (int)(TRQ_REQ_SLEW_UP_NM_S) <= (int)(TRQ_SLEW_ACCEL_SPORTS_NM_S),
               "TRQ_REQ_SLEW_UP_NM_S must be <= TRQ_SLEW_ACCEL_* (request limiter binds)");

/* High-speed regen fade band must be a real, ascending interval (the fade
 * divides by END-START). Tunables live in vehicle_config.h. */
#if REGEN_FADE_ENABLE
_Static_assert((int)(REGEN_FADE_START_RPM) < (int)(REGEN_FADE_END_RPM),
               "REGEN_FADE_START_RPM must be < REGEN_FADE_END_RPM");
#endif

/* Innermost FOC-model torque ramp (FOC_TRQ_RAMP_*, motor profile) must be
 * strictly the fastest layer so it acts only as a guard: UP above both
 * ACCEL slews, DOWN above the DECEL slew (else it silently halves every
 * protective cut). TRQ_SLEW_ACCEL_* resolve to the one active variant, so
 * these asserts cover whichever variant is selected. */
_Static_assert((int)(FOC_TRQ_RAMP_UP_NM_S) >= (int)(TRQ_SLEW_ACCEL_ECO_NM_S) &&
               (int)(FOC_TRQ_RAMP_UP_NM_S) >= (int)(TRQ_SLEW_ACCEL_SPORTS_NM_S),
               "FOC_TRQ_RAMP_UP_NM_S must be >= TRQ_SLEW_ACCEL_* (model ramp is a guard, not a shaper)");
_Static_assert((int)(FOC_TRQ_RAMP_DOWN_NM_S) >= (int)(TRQ_SLEW_DECEL_NM_S),
               "FOC_TRQ_RAMP_DOWN_NM_S must be >= TRQ_SLEW_DECEL_NM_S (declared cut rate must be real)");

/* =========================================================================
 * DCLO — CLOSED-LOOP DC CURRENT LIMITING
 *
 * Fast path (every apply_torque_scaling call): the battery current is
 * estimated from local FOC quantities and regulated against the BMS
 * discharge limit by a feedforward power cap plus a PI that outputs a
 * torque cap (same conditional anti-windup structure as the speed limiter):
 *
 *   P_ac   = 1.5 (Vd·Id + Vq·Iq)            inverter output power (W)
 *   I_dc^  = max(0, P_ac) / (eta^ · Vdc)    estimated pack draw (A)
 *   T_ff   = eta^ · target_A · Vdc / omega  feedforward torque cap (Nm)
 *
 * Measured Id/Iq capture field weakening and power factor exactly, and Vdc
 * is the controller's own ADC measurement, so the only model unknown is the
 * scalar eta^ (inverter efficiency + any constant estimation bias). The
 * estimate updates at loop rate — feedback latency is effectively zero, the
 * margin is small, and nothing re-learns when the operating point moves.
 *
 * Slow path (each new 0x110 frame, ~308 ms): eta^ is calibrated so the
 * estimate tracks the BMS-measured battery current (absorbs efficiency
 * drift, sensor gain, steady aux loads). A tighten-only backstop trim
 * integrates the raw measurement against 100% of DCLO — it engages only if
 * the estimator itself is wrong (Vdc ADC / current-sensor fault), since the
 * fast loop regulates to (1 - DCLO_PI_MARGIN)·DCLO, below the backstop line.
 *
 * Staleness (DCLO_BMS_TIMEOUT_MS): if 0x12A stops arriving the DC limit
 * falls back to min(last, DCLO_DEFAULT) — never loosens on silence; if
 * 0x110 stops, eta^ and the trim freeze and the loop keeps limiting on the
 * estimator alone.
 * ========================================================================= */

#define DCLO_DEFAULT                100.0f   /* boot / stale-BMS DC limit (A)     */

/* Fast loop */
#define DCLO_PI_MARGIN              0.02f    /* regulate estimate to 98% of DCLO  */
#define DCLO_PI_KP                  0.3f     /* Nm of cap per A over target       */
#define DCLO_PI_KI                  4.0f     /* Nm per (A·s)                      */
#define DCLO_EST_FILT_TAU_S         0.01f    /* I_dc estimate LPF time constant   */

/* eta^ calibration (runs once per fresh 0x110 sample) */
#define DCLO_ETA_INIT               0.95f    /* seed: inverter efficiency         */
#define DCLO_ETA_MIN                0.5f     /* wide clamp: eta^ is a calibration */
#define DCLO_ETA_MAX                1.2f     /*   scalar, not a physical claim    */
#define DCLO_ETA_ADAPT_GAIN         0.1f     /* correction fraction per sample    */
#define DCLO_ETA_ADAPT_MIN_A        15.0f    /* adapt only above this current     */

/* BMS staleness timeout (0x110 / 0x12A arrive every ~308 ms when healthy) */
#define DCLO_BMS_TIMEOUT_MS         2000U

/* =========================================================================
 * SOC-AWARE DCLO RESERVE — ported from TnV/PEGV4_Production
 * ========================================================================= */

#define SOC_BREAK                   0.10f    /* Abs guard starts ramping below this SOC */
#define SOC_GUARD_MAX_A             10.0f    /* Max absolute A reserved from DCLO at 0% SOC */

/* =========================================================================
 * DCLO BACKSTOP TRIM — tighten-only integrator on the RAW BMS-measured
 * battery current vs 100% of DCLO. Second line of defense behind the fast
 * loop above: it stays at 0 unless the estimator itself is broken, because
 * the fast loop regulates 2% below this line. Gains sized to the ~308 ms
 * 0x110 period — keep KI_ATTACK <= 2.0 at that rate or the trim hunts
 * against the Rx latency.
 * ========================================================================= */

#define DCLO_FB_KI_ATTACK           1.5f     /* (A of trim)/s per A over DCLO     */
#define DCLO_FB_KI_RELEASE          0.2f     /* (A of trim)/s per A of headroom   */
#define DCLO_FB_IBATT_SIGN          -1.0f    /* 0x110 PC: charge-positive BMS     */

/* =========================================================================
 * ENUMERATIONS
 * ========================================================================= */

typedef enum {
    TEMP_STATUS_NORMAL   = 0,    /* All temperatures within nominal range */
    TEMP_STATUS_DERATING = 1     /* At least one sensor in derating zone  */
} TempDerateStatus_t;

/* =========================================================================
 * STRUCTURES
 * ========================================================================= */

typedef struct {
    float              scale;          /* Combined scale [0.0, 1.0]   */
    float              motor_scale;    /* Individual motor scale       */
    float              ctrl_scale;     /* Individual controller scale  */
    TempDerateStatus_t status;         /* NORMAL / DERATING            */
} TempDerateResult_t;

/* =========================================================================
 * FUNCTION PROTOTYPES
 * ========================================================================= */

float get_torque_speed_limit(float speed_rad_s);

float apply_torque_scaling(float torque_ref, float speed_rad_s,
                            power_mode_t drive_mode, fnr_state_t direction,
                            float motor_temp, float ctrl_temp,
                            TempDerateResult_t *derate_result_out);

extern TempDerateResult_t derate_result;
#endif /* TORQUE_MANAGEMENT_H */