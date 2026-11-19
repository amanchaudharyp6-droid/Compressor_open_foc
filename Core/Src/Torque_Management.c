/**
 * @file    torque_management.c
 * @brief   Torque scaling based on speed and temperature derating
 *          for Pegasus 96V Motor Controller
 */

#include "Torque_Management.h"
#include <float.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

TempDerateResult_t derate_result;

/* =========================================================================
 * PRIVATE IMPLEMENTATION CONSTANTS
 * Public tuning lives in Torque_Management.h / vehicle_config.h / the
 * motor profile. Values below are internal math bounds and aren't
 * intended to be tuned at the application level.
 * ========================================================================= */

/* Omega floor for Pmax/omega — prevents divide-by-zero at standstill */
#define OMEGA_M_FLOOR_RADS          1.0f

/* dt sanity clamps for while-loop cadence (HAL_GetTick is ms resolution) */
#define DT_MIN_S                    1.0e-4f
#define DT_MAX_S                    0.05f

/* =========================================================================
 * TORQUE-SPEED CHARACTERISTIC TABLE
 * 14-point nominal envelope per vehicle variant (speed axes must match —
 * TORQUE_DERATED_AT_SPD below is shared). The duplicated 5100 RPM row is a
 * legacy step point, now zero-width; the interpolator skips it.
 * ========================================================================= */
#if HIRANGE_XR
/* HiRange XR: 40 Nm launch plateau (= SPORTS_MAX_TORQUE, street-safe tune),
 * mid-range at or below the 6.284 kW SPORTS power line — from ~3000 RPM
 * this table IS the power mapping, so retune with SPORTS_MAX_POWER. */
static const TorqueSpeedPoint_t TORQUE_SPEED_TABLE[TORQUE_SPEED_TABLE_SIZE] = {
    {    0.0f, 34.0f },
    {  500.0f, 34.0f },
    { 1000.0f, 32.0f },
    { 1500.0f, 30.0f },   /* below the 6.284 kW line (40.0 Nm) — softer mid   */
    { 2000.0f, 24.0f },   /* below the line (30.0 Nm)                         */
    { 2500.0f, 21.0f },   /* below the 6.284 kW line (24.0 Nm)                */
    { 3000.0f, 20.0f },   /* 6.284 kW @ 3000 RPM = 20.0 Nm — on the line      */
    { 4400.0f, 13.5f },   /* 6.284 kW @ 4400 RPM = 13.6 Nm                    */
    { 5100.0f, 11.5f },   /* taper into the voltage-limited region            */
    { 5100.0f, 11.5f },
    { 5300.0f, 10.9f },   /* >= 5300 RPM: back-EMF / voltage limited          */
    { 5500.0f, 10.4f },
    { 6000.0f, 9.0f },
    { 6500.0f, 8.0f }
};
#endif
#if HILOAD_XR
/* HiLoad XR (cargo): 80 Nm motor-peak launch, then the 9.215 kW SPORTS
 * power line from ~2000 RPM. NOT yet vehicle-tested — see the wheel-lift
 * caution on the envelope block in vehicle_config.h. */
static const TorqueSpeedPoint_t TORQUE_SPEED_TABLE[TORQUE_SPEED_TABLE_SIZE] = {
    {    0.0f, 70.0f },
    {  500.0f, 70.0f },
    { 1000.0f, 60.0f },
    { 1500.0f, 50.0f },   /* below the 9.215 kW line (58.7 Nm) — softer mid  */
    { 2000.0f, 44.0f },   /* 9.215 kW @ 2000 RPM = 44.0 Nm                   */
    { 2500.0f, 35.0f },   /* 9.215 kW @ 2500 RPM = 35.2 Nm                   */
    { 3000.0f, 29.0f },   /* 9.215 kW @ 3000 RPM = 29.3 Nm                   */
    { 4400.0f, 20.0f },   /* 9.215 kW @ 4400 RPM = 20.0 Nm                   */
    { 5100.0f, 14.0f },   /* taper into the voltage-limited region           */
    { 5100.0f, 14.0f },
    { 5300.0f, 11.5f },   /* >= 5300 RPM: back-EMF / voltage limited         */
    { 5500.0f, 10.0f },
    { 6000.0f, 8.0f },
    { 6500.0f, 7.0f }
};
#endif
#if HIRANGE_TR
/* HiRange TR (76.8 V): 72 Nm launch plateau on the 5.655 kW power line (same
 * peak power as XR, higher launch). Flat 72 Nm to the ~750 RPM knee (72 Nm x
 * 78.5 rad/s = 5.655 kW), then torque = 5655 W / omega. Tracks SPORTS_MAX_POWER
 * in vehicle_config.h so the table and the FOC power cap agree — no hidden
 * clipping. The >=5100 RPM tail is the power-line ideal; at 76.8 V the voltage
 * limit sits below it and binds first (FOC back-EMF limit) — re-map on logged
 * TR data. Speed breakpoints are shared with TORQUE_DERATED_AT_SPD, so the
 * 750 RPM knee is approximated by the 500->1000 RPM interpolation. */
static const TorqueSpeedPoint_t TORQUE_SPEED_TABLE[TORQUE_SPEED_TABLE_SIZE] = {
    {    0.0f, 72.0f },
    {  500.0f, 72.0f },   /* flat launch: 72 Nm @ 500 RPM = 3.77 kW (< 5.655) */
    { 1000.0f, 54.0f },   /* 5.655 kW @ 1000 RPM = 54.0 Nm                    */
    { 1500.0f, 36.0f },   /* 5.655 kW @ 1500 RPM = 36.0 Nm                    */
    { 2000.0f, 27.0f },   /* 5.655 kW @ 2000 RPM = 27.0 Nm                    */
    { 2500.0f, 21.6f },   /* 5.655 kW @ 2500 RPM = 21.6 Nm                    */
    { 3000.0f, 18.0f },   /* 5.655 kW @ 3000 RPM = 18.0 Nm                    */
    { 4400.0f, 12.3f },   /* 5.655 kW @ 4400 RPM = 12.3 Nm                    */
    { 5100.0f, 10.6f },   /* 5.655 kW @ 5100 RPM = 10.6 Nm                    */
    { 5100.0f, 10.6f },
    { 5300.0f, 10.2f },   /* >= 5300 RPM: 76.8 V voltage limit pulls below   */
    { 5500.0f, 9.8f },    /*   the power line — re-map on vehicle             */
    { 6000.0f, 9.0f },
    { 6500.0f, 8.3f }
};
#endif

/* =========================================================================
 * DERATED TORQUE ENVELOPE
 * Parallel to TORQUE_SPEED_TABLE (same speed breakpoints).
 * Tune per motor thermal characterisation.
 * ========================================================================= */
static const float TORQUE_DERATED_AT_SPD[TORQUE_SPEED_TABLE_SIZE] = {
    18.0f,  /*    0 RPM */
    15.0f,  /*  500 RPM */
    11.5f,  /* 1000 RPM */
    10.0f,  /* 1500 RPM */
    9.0f,  /* 2000 RPM */
    8.75f,  /* 2500 RPM */
    8.0f,  /* 3000 RPM */
    6.0f,  /* 4400 RPM */
    5.0f,  /* 5100 RPM (low side of step) */
    5.0f,  /* 5100 RPM (high side of step) */
    4.5f,  /* 5300 RPM */
    3.5f,  /* 5500 RPM */
    2.5f,   /* 6000 RPM */
    2.0f   /* 6500 RPM */
};

/* =========================================================================
 * MODE PARAMETERS
 * ========================================================================= */
typedef struct {
    float max_torque_nm;
    float nominal_max_power_w;
    float max_speed_rpm;
    float torque_scale;
    float slew_accel_nm_s;
} ModeParams_t;

static const ModeParams_t MODE_PARAMS[2] = {
    /* ECO    */ { ECO_MAX_TORQUE,    ECO_MAX_POWER,      SPEED_LIMIT_ECO_RPM,    TORQUE_SCALE_ECO,    TRQ_SLEW_ACCEL_ECO_NM_S    },
    /* SPORTS */ { SPORTS_MAX_TORQUE, SPORTS_MAX_POWER,  SPEED_LIMIT_SPORTS_RPM, TORQUE_SCALE_SPORTS, TRQ_SLEW_ACCEL_SPORTS_NM_S }
};

static ModeParams_t get_mode_params(power_mode_t drive_mode, fnr_state_t direction)
{
    /* REVERSE forces full SPORTS torque/power/scale; only the speed cap differs.
     * DCLO still applies downstream regardless of direction. */
    ModeParams_t mp = (direction == REVERSE || drive_mode == SPORTS)
                          ? MODE_PARAMS[1]
                          : MODE_PARAMS[0];
    if (direction == REVERSE) mp.max_speed_rpm = SPEED_LIMIT_REVERSE_RPM;
    return mp;
}

/* =========================================================================
 * PERSISTENT STATE (file-local)
 * ========================================================================= */
static float    s_ctrl_temp_filt  = 25.0f;
static float    s_motor_temp_filt = 25.0f;
static uint8_t  s_ctrl_derate     = 0;   /* ctrl temp latch (release hysteresis)  */
static uint8_t  s_motor_derate    = 0;   /* motor temp latch (release hysteresis) */
static float    s_prev_torque     = 0.0f;
static float    s_speed_integ     = 0.0f; /* PI speed-limit integrator (Nm)        */
static float    s_eff_speed_cap   = 0.0f; /* eased speed cap (rpm); ramps down on a mode downgrade */
static power_mode_t s_eff_mode    = SPORTS;/* mode actually IN EFFECT; a downgrade is deferred
                                            * above MODE_DOWNGRADE_DEFER_RPM. Seeded SPORTS so a
                                            * first call already above the threshold cannot adopt
                                            * ECO before the deferral has been evaluated once. */
static uint32_t s_last_cyc        = 0;    /* DWT->CYCCNT snapshot for dt measurement */
static uint8_t  s_dwt_ready       = 0;    /* lazy CYCCNT enable (works if EST_CYC_CNT=0) */
static float    s_boost_budget_J  = PWR_BOOST_E_MAX_J; /* overboost thermal budget (full at boot) */
static uint8_t  s_boost_active    = 1;    /* 1 = boost armed, 0 = held at continuous (hysteresis) */
static float    s_dclo_trim_A     = 0.0f; /* DCLO backstop trim (A), tighten-only   */
static float    s_eta_hat         = DCLO_ETA_INIT; /* DC-current estimator calib     */
static float    s_idc_filt        = 0.0f; /* filtered I_dc estimate (A)              */
static float    s_dclo_integ      = 0.0f; /* DCLO PI integrator (Nm)                 */
static uint32_t s_seen_0x110_ms   = 0;    /* last 0x110 Rx tick consumed for calib   */

/* =========================================================================
 * LOCAL HELPERS
 * ========================================================================= */
static inline float clampf_local(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* Derated envelope interpolation — walks the same speed axis as TORQUE_SPEED_TABLE. */
static float get_torque_derated_limit(float speed_rad_s)
{
    float spd = RADS_TO_RPM(fabsf(speed_rad_s));

    if (spd <= TORQUE_SPEED_TABLE[0U].speed_rpm)
        return TORQUE_DERATED_AT_SPD[0U];
    if (spd >= TORQUE_SPEED_TABLE[TORQUE_SPEED_TABLE_SIZE - 1U].speed_rpm)
        return TORQUE_DERATED_AT_SPD[TORQUE_SPEED_TABLE_SIZE - 1U];

    for (uint32_t i = 0U; i < (TORQUE_SPEED_TABLE_SIZE - 1U); i++) {
        float lo = TORQUE_SPEED_TABLE[i].speed_rpm;
        float hi = TORQUE_SPEED_TABLE[i + 1U].speed_rpm;
        if (spd >= lo && spd < hi) {
            float t = (spd - lo) / (hi - lo);
            return TORQUE_DERATED_AT_SPD[i] + t * (TORQUE_DERATED_AT_SPD[i + 1U] - TORQUE_DERATED_AT_SPD[i]);
        }
    }
    return TORQUE_DERATED_AT_SPD[TORQUE_SPEED_TABLE_SIZE - 1U];
}

/* =========================================================================
 * HIGH-SPEED REGEN FADE
 * Scales the FOC model's commanded regen cap to zero across the
 * REGEN_FADE_* band (see vehicle_config.h). max_Regen_Torque is a model
 * INPUT — same pattern as the FW/PID params — so this needs no change to
 * generated code. A single aligned float store is atomic on Cortex-M7, so
 * writing it from the superloop while FOC_Loop_step0 reads it in the 10 kHz
 * ISR cannot tear.
 *
 * Smoothstep (zero slope at both edges) rather than a hard cut: a step in
 * the regen cap would put a torque discontinuity at the boundary, i.e.
 * trade one lurch for another. Zero cap is safe in the regen state machine
 * — it multiplies through to a zero torque command, never divides by it.
 * ========================================================================= */
#if REGEN_FADE_ENABLE
static void update_regen_fade(float speed_rpm, power_mode_t drive_mode)
{
    float cap;

    if (drive_mode == SPORTS) {
        /* SPORTS: fade the commanded regen cap out at high speed. A larger regen
         * command re-engaging at speed triggers a current-angle excursion (the
         * decel jerk); REGEN_MAX_TORQUE_SPORTS is reduced to shrink it further. */
        if (speed_rpm >= REGEN_FADE_END_RPM) {
            cap = 0.0f;
        } else if (speed_rpm <= REGEN_FADE_START_RPM) {
            cap = REGEN_MAX_TORQUE_SPORTS;
        } else {
            float t = (speed_rpm - REGEN_FADE_START_RPM) /
                      (REGEN_FADE_END_RPM - REGEN_FADE_START_RPM);
            cap = REGEN_MAX_TORQUE_SPORTS * (1.0f - (t * t * (3.0f - (2.0f * t))));
        }
    } else {
        /* ECO: NO fade — regen active across the whole range at the ECO torque,
         * for consistent deceleration. ECO torque is low enough that the
         * high-speed excursion does not bite. */
        cap = REGEN_MAX_TORQUE_ECO;
    }

    FOC_Loop_U.Regen.max_Regen_Torque = cap;
}
#endif

/* Linear blend P_nom -> MIN_POWER_AT_FULL_DERATE_W across a temperature interval. */
static inline float calc_Pmax_from_temp(float temp, float t_start, float t_full, float P_nom)
{
    if (temp <= t_start) return P_nom;
    if (temp >= t_full)  return MIN_POWER_AT_FULL_DERATE_W;
    float r = (temp - t_start) / (t_full - t_start);
    return (1.0f - r) * P_nom + r * MIN_POWER_AT_FULL_DERATE_W;
}

/* =========================================================================
 * DCLO — closed-loop DC current limit (algorithm block in Torque_Management.h)
 * Local I_dc estimator + feedforward power cap + fast PI, with eta^
 * calibration and a backstop trim driven by the BMS on CAN 0x110.
 * ========================================================================= */

/* Absolute guard (A) subtracted from DCLO: 0 above SOC_BREAK, up to SOC_GUARD_MAX_A at 0%. */
static inline float soc_guard_A(float soc_pu)
{
    float soc = clampf_local(soc_pu, 0.0f, 1.0f);
    if (soc >= SOC_BREAK) return 0.0f;
    float k = (SOC_BREAK - soc) / SOC_BREAK;   /* 0..1 */
    return SOC_GUARD_MAX_A * k;
}

static float apply_dclo_torque_limit(float torque_in, float omega_m, float dt)
{
    uint32_t now_ms = HAL_GetTick();

    /* Effective DC limit. can_dclo boots at DCLO_DEFAULT and a BMS-published
     * 0 A limit is honoured (target -> 0 -> cap -> 0). If 0x12A goes stale,
     * fall back to min(last, DCLO_DEFAULT) — never loosen on silence. */
    float dclo = can_d.can_dclo;
    if (((now_ms - can_d.t_rx_0x12A_ms) > DCLO_BMS_TIMEOUT_MS) && (dclo > DCLO_DEFAULT))
        dclo = DCLO_DEFAULT;
    float dclo_eff = dclo - soc_guard_A(can_d.can_soc);
    if (dclo_eff < 0.0f) dclo_eff = 0.0f;

    /* Fast I_dc estimator from local FOC quantities (loop rate, no CAN).
     * Regen (negative P_ac) clamps to 0: DCLO limits discharge only. */
    float vbus = (float)d.Vdc;
    if (vbus < 10.0f) vbus = OP_VOLTAGE;   /* bus ADC not up yet */
    float p_ac = 1.5f * (FOC_Loop_Y.Vd * FOC_Loop_Y.Id +
                         FOC_Loop_Y.Vq * FOC_Loop_Y.Iq);
    if (p_ac < 0.0f) p_ac = 0.0f;
    float idc_raw = p_ac / (s_eta_hat * vbus);
    float a_filt  = dt / (DCLO_EST_FILT_TAU_S + dt);
    s_idc_filt   += a_filt * (idc_raw - s_idc_filt);

    /* Slow path — eta^ calibration once per fresh 0x110 frame, backstop trim
     * on the held measurement. Both freeze if the BMS goes quiet; the fast
     * loop keeps limiting on the estimator alone. */
    uint8_t fresh_0x110 = ((now_ms - can_d.t_rx_0x110_ms) <= DCLO_BMS_TIMEOUT_MS);
    float   ibatt       = DCLO_FB_IBATT_SIGN * can_d.can_battCurr;

    if (fresh_0x110 && (can_d.t_rx_0x110_ms != s_seen_0x110_ms)) {
        s_seen_0x110_ms = can_d.t_rx_0x110_ms;
        if ((ibatt > DCLO_ETA_ADAPT_MIN_A) && (s_idc_filt > DCLO_ETA_ADAPT_MIN_A)) {
            /* est/meas > 1 -> estimate reads high -> raise eta^ (and vice
             * versa) so the estimate converges onto the BMS measurement. */
            float ratio = clampf_local(s_idc_filt / ibatt, 0.5f, 2.0f);
            s_eta_hat *= (1.0f + DCLO_ETA_ADAPT_GAIN * (ratio - 1.0f));
            s_eta_hat  = clampf_local(s_eta_hat, DCLO_ETA_MIN, DCLO_ETA_MAX);
        }
    }
    if (fresh_0x110) {
        float over_A = ibatt - dclo_eff;   /* vs 100% of DCLO — backstop line */
        s_dclo_trim_A += ((over_A > 0.0f) ? DCLO_FB_KI_ATTACK
                                          : DCLO_FB_KI_RELEASE) * over_A * dt;
        s_dclo_trim_A = clampf_local(s_dclo_trim_A, 0.0f, dclo_eff);
    }
    dclo_eff -= s_dclo_trim_A;

    float target_A = dclo_eff * (1.0f - DCLO_PI_MARGIN);

    /* Feedforward power cap: the torque whose P_ac budget matches target_A.
     * Accurate once eta^ has calibrated, so it catches fast slew-driven
     * approaches to the limit before the PI has any error to act on.
     * (omega_m is pre-floored by the caller; at low speed T_ff is huge and
     * never binds — I_dc is small there anyway.) */
    float torque_ff = (s_eta_hat * target_A * vbus) / omega_m;

    /* Fast PI -> torque cap in [0, torque_in]; conditional-integration
     * anti-windup, same structure as apply_speed_limit_pi. */
    float err = target_A - s_idc_filt;                    /* + = headroom (A) */
    float u   = clampf_local(DCLO_PI_KP * err + s_dclo_integ, 0.0f, torque_in);
    uint8_t at_hi = (u >= torque_in);
    uint8_t at_lo = (u <= 0.0f);
    if (!((at_hi && err > 0.0f) || (at_lo && err < 0.0f))) {
        s_dclo_integ += DCLO_PI_KI * err * dt;
    }
    s_dclo_integ = clampf_local(s_dclo_integ, 0.0f, torque_in);

    return fminf(u, torque_ff);
}

/* =========================================================================
 * TEMP DERATE — per-sensor latches, worst case wins
 * Ctrl and motor blends are evaluated every cycle; the deeper derate takes
 * both the envelope and Pmax, so a latch on one sensor can never mask the
 * other overheating. Latches only add release hysteresis and drive the CAN
 * status flags (blend below derate_start is zero regardless of the latch).
 * Returns blended envelope torque; writes Pmax_out and derate_result_out.
 * ========================================================================= */
static float update_temp_derate(float torque_nominal, float torque_derated, float P_nom,
                                float *Pmax_out, TempDerateResult_t *result_out)
{
    /* Independent latches with release hysteresis */
    if (s_ctrl_derate) {
        if (s_ctrl_temp_filt < (CTRL_TEMP_DERATE_START - TEMP_HYSTERESIS_C)) s_ctrl_derate = 0U;
    } else {
        if (s_ctrl_temp_filt >= CTRL_TEMP_DERATE_START) s_ctrl_derate = 1U;
    }
    if (s_motor_derate) {
        if (s_motor_temp_filt < (MOTOR_TEMP_DERATE_START - TEMP_HYSTERESIS_C)) s_motor_derate = 0U;
    } else {
        if (s_motor_temp_filt >= MOTOR_TEMP_DERATE_START) s_motor_derate = 1U;
    }

    /* Mirror the latches onto the CAN status flags (Send_on_CAN_708) */
    d.ctlr_derate_flag  = s_ctrl_derate;
    d.motor_derate_flag = s_motor_derate;

    /* Per-sensor blend factors; both evaluated every cycle */
    float t_ctrl = clampf_local(
        (s_ctrl_temp_filt - CTRL_TEMP_DERATE_START) /
        (CTRL_TEMP_DERATE_END - CTRL_TEMP_DERATE_START), 0.0f, 1.0f);
    float t_motor = clampf_local(
        (s_motor_temp_filt - MOTOR_TEMP_DERATE_START) /
        (MOTOR_TEMP_DERATE_END - MOTOR_TEMP_DERATE_START), 0.0f, 1.0f);
    float t_blend = fmaxf(t_ctrl, t_motor);

    float torque_env = (1.0f - t_blend) * torque_nominal + t_blend * torque_derated;

    float Pmax_ctrl  = calc_Pmax_from_temp(s_ctrl_temp_filt,  CTRL_TEMP_DERATE_START,
                                           CTRL_TEMP_DERATE_END,  P_nom);
    float Pmax_motor = calc_Pmax_from_temp(s_motor_temp_filt, MOTOR_TEMP_DERATE_START,
                                           MOTOR_TEMP_DERATE_END, P_nom);
    float Pmax = fminf(Pmax_ctrl, Pmax_motor);

    if (result_out != NULL) {
        result_out->motor_scale = 1.0f - t_motor;
        result_out->ctrl_scale  = 1.0f - t_ctrl;
        result_out->scale       = (torque_nominal > 0.0f) ? (torque_env / torque_nominal) : 0.0f;
        result_out->status      = (s_ctrl_derate || s_motor_derate) ? TEMP_STATUS_DERATING
                                                                    : TEMP_STATUS_NORMAL;
    }

    *Pmax_out = Pmax;
    return torque_env;
}

/* =========================================================================
 * SPEED LIMITER — PI with conditional-integration anti-windup
 *
 * err = vmax - speed (RPM): + below the limit, - over it.
 * Output is a torque CAP in [0, torque_in]:
 *   - Far below vmax: err large +, u saturates at torque_in (full pass-through);
 *     the integrator is frozen (anti-windup), so the limiter is dormant.
 *   - Near/over vmax: u comes off the rail and the integrator ratchets to the
 *     steady holding torque T* that pins speed at vmax. No hard cut, no taper
 *     step, so no limit cycle.
 *   - Steep grade where even 0 Nm can't hold vmax: u floors at 0 and the
 *     integrator parks at 0 (frozen) instead of winding down — recovery is
 *     immediate once speed drops back, with no deep dip.
 * No negative output: downstream FOC owns regen (see header note).
 * ========================================================================= */
static float apply_speed_limit_pi(float torque_in, float speed_rpm, float vmax, float dt)
{
    float err      = vmax - speed_rpm;
    float u_unsat  = SPEED_PI_KP * err + s_speed_integ;
    float u        = clampf_local(u_unsat, 0.0f, torque_in);

    /* Conditional integration: freeze the integrator only when integrating
     * would push it further into the rail it is already sitting on. */
    uint8_t at_hi = (u >= torque_in);   /* full torque  -> below the limit  */
    uint8_t at_lo = (u <= 0.0f);        /* zero torque  -> deep over limit   */
    if (!((at_hi && err > 0.0f) || (at_lo && err < 0.0f))) {
        s_speed_integ += SPEED_PI_KI * err * dt;
    }
    /* Bound the integrator to the achievable (non-regen) cap range. */
    s_speed_integ = clampf_local(s_speed_integ, 0.0f, torque_in);

    return u;
}

/* =========================================================================
 * TORQUE-SPEED CHARACTERISTIC LIMIT
 * ========================================================================= */

/**
 * @brief   Get maximum allowable torque at a given speed from the
 *          torque-speed characteristic table using linear interpolation.
 *
 * @details Below the first table entry  → returns first entry max torque
 *          Above the last table entry   → returns last entry max torque
 *          Between two entries          → linear interpolation
 *
 * @param   speed_rad_s   Speed feedback (rad/s), sign handled internally.
 * @return  Maximum allowable torque (Nm) at given speed.
 */
float get_torque_speed_limit(float speed_rad_s)
{
    float speed_rpm = RADS_TO_RPM(fabsf(speed_rad_s));

    /* Below first breakpoint — return first entry max torque */
    if (speed_rpm <= TORQUE_SPEED_TABLE[0U].speed_rpm)
        return TORQUE_SPEED_TABLE[0U].max_torque_nm;

    /* Above last breakpoint — return last entry max torque */
    if (speed_rpm >= TORQUE_SPEED_TABLE[TORQUE_SPEED_TABLE_SIZE - 1U].speed_rpm)
        return TORQUE_SPEED_TABLE[TORQUE_SPEED_TABLE_SIZE - 1U].max_torque_nm;

    /* Find segment containing speed_rpm and interpolate */
    for (uint32_t i = 0U; i < (TORQUE_SPEED_TABLE_SIZE - 1U); i++)
    {
        float spd_lo = TORQUE_SPEED_TABLE[i].speed_rpm;
        float spd_hi = TORQUE_SPEED_TABLE[i + 1U].speed_rpm;

        if (speed_rpm >= spd_lo && speed_rpm < spd_hi)
        {
            float trq_lo = TORQUE_SPEED_TABLE[i].max_torque_nm;
            float trq_hi = TORQUE_SPEED_TABLE[i + 1U].max_torque_nm;

            float t = (speed_rpm - spd_lo) / (spd_hi - spd_lo);
            return trq_lo + t * (trq_hi - trq_lo);
        }
    }

    /* Should never reach here — return last entry as fallback */
    return TORQUE_SPEED_TABLE[TORQUE_SPEED_TABLE_SIZE - 1U].max_torque_nm;
}

/* =========================================================================
 * COMBINED TORQUE SCALING
 * ========================================================================= */

float apply_torque_scaling(float torque_ref, float speed_rad_s,
                            power_mode_t drive_mode, fnr_state_t direction,
                            float motor_temp, float ctrl_temp,
                            TempDerateResult_t *derate_result_out)
{
    /* --- Measure dt from the DWT cycle counter (CPU-clock resolution) ---
     * HAL_GetTick() is 1 ms resolution; this superloop iterates faster than
     * 1 ms, so most calls saw dt = 0 (clamped to DT_MIN_S) and the slew /
     * integral cadence was wrong and load-dependent. DWT->CYCCNT is free-
     * running at SystemCoreClock. Enabled lazily so this is correct even when
     * EST_CYC_CNT = 0 (main.c only enables CYCCNT under that flag). */
    if (!s_dwt_ready) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
        s_last_cyc        = DWT->CYCCNT;
        s_dwt_ready       = 1U;
    }
    uint32_t now_cyc = DWT->CYCCNT;
    uint32_t d_cyc   = now_cyc - s_last_cyc;   /* modular: correct across 32-bit wrap */
    s_last_cyc       = now_cyc;
    float cpu_hz = (float)SystemCoreClock;
    float dt = (cpu_hz > 1.0f) ? ((float)d_cyc / cpu_hz) : T_S;
    if (dt < DT_MIN_S) dt = DT_MIN_S;   /* also covers the very first call    */
    if (dt > DT_MAX_S) dt = DT_MAX_S;   /* clamps counter wrap / a long stall */

    /* --- IIR on temperatures (match TnV) --- */
    s_ctrl_temp_filt  += TEMP_IIR_ALPHA * (ctrl_temp  - s_ctrl_temp_filt);
    s_motor_temp_filt += TEMP_IIR_ALPHA * (motor_temp - s_motor_temp_filt);

    /* --- Mode-downgrade deferral (see MODE_DOWNGRADE_DEFER_RPM) -------------
     * Above the flat voltage-limited top speed both modes are saturated, so a
     * downgrade hands the current loop a new reference it has no voltage
     * authority to execute. Hold the mode in effect until the speed is back
     * inside the requested mode's own cap. Upgrades take effect immediately;
     * NEUTRAL re-arms below. Compared on the forward table entries so REVERSE's
     * cap override cannot make a downgrade look like an upgrade — and reverse
     * never reaches the threshold anyway. */
    float speed_rpm_now = RADS_TO_RPM(fabsf(speed_rad_s));
    if (MODE_PARAMS[drive_mode].max_speed_rpm <
            MODE_PARAMS[s_eff_mode].max_speed_rpm) {
        if ((speed_rpm_now > MODE_DOWNGRADE_DEFER_RPM) &&
            (speed_rpm_now > MODE_PARAMS[drive_mode].max_speed_rpm)) {
            /* still uncontrollable in the requested mode — keep the old one */
        } else {
            s_eff_mode = drive_mode;
        }
    } else {
        s_eff_mode = drive_mode;
    }

    /* Fade the FOC model's commanded regen cap out at high speed. Runs every
     * loop while driving; the model init value applies before FOC_START.
     * Uses the EFFECTIVE mode so a deferred downgrade does not step the regen
     * cap either (SPORTS keeps its high-speed fade to zero). */
#if REGEN_FADE_ENABLE
    update_regen_fade(speed_rpm_now, s_eff_mode);
#endif

    ModeParams_t mp = get_mode_params(s_eff_mode, direction);

    /* Time-limited power boost: gate the high-power modes' Pmax by the thermal
     * budget (see header). Hysteresis: once the budget empties, hold at
     * PWR_CONT_W until it recovers past PWR_BOOST_REARM_J. ECO stays untouched
     * (its nominal power is below PWR_CONT_W). */
#if PWR_BOOST_ENABLE
    if (mp.nominal_max_power_w > PWR_CONT_W) {
        if (s_boost_active) {
            if (s_boost_budget_J <= 0.0f)              s_boost_active = 0U;
        } else {
            if (s_boost_budget_J >= PWR_BOOST_REARM_J) s_boost_active = 1U;
        }
        mp.nominal_max_power_w = s_boost_active ? PWR_BOOST_W : PWR_CONT_W;
    }
#endif

    /* 1) Torque envelope: nominal + derated lookups */
    float torque_nom = get_torque_speed_limit(speed_rad_s);
    float torque_der = get_torque_derated_limit(speed_rad_s);

    /* 2) Temp derate (worst-of ctrl/motor): blends envelope AND Pmax together.
     *    P_nom fed in is mp.nominal_max_power_w -> ECO: ECO_MAX_POWER;
     *    SPORTS/REV: boost-gated (PWR_BOOST_W or PWR_CONT_W, set just above).
     *    Seed Pmax_eff to the nominal so state 0 is already correct before writeback. */
    float Pmax_eff = mp.nominal_max_power_w;
    float torque_env = update_temp_derate(torque_nom, torque_der,
                                          mp.nominal_max_power_w,
                                          &Pmax_eff, derate_result_out);

    /* 3) Mode torque scale + hard clamp */
    torque_env *= mp.torque_scale;
    if (torque_env > mp.max_torque_nm) torque_env = mp.max_torque_nm;

    /* 4) Power cap -> torque (Pmax / ω_mech) */
    float omega_m = fabsf(speed_rad_s);
    if (omega_m < OMEGA_M_FLOOR_RADS) omega_m = OMEGA_M_FLOOR_RADS;
    float torque_pwr = Pmax_eff / omega_m;

    float torque_cap = fminf(torque_env, torque_pwr);

    /* 5) Speed limit: PI controller with anti-windup (per-mode vmax; reverse
     *    already overridden in mp). In NEUTRAL hold zero and reset the
     *    integrator so it re-converges cleanly on the next drive engagement. */
    if (direction == NEUTRAL) {
        torque_cap    = 0.0f;
        s_speed_integ = 0.0f;
        s_eff_speed_cap = mp.max_speed_rpm;  /* re-arm the eased cap for next drive */
        s_eff_mode      = drive_mode;        /* at rest, adopt the requested mode    */
    } else {
        /* Ease the effective speed cap DOWN on a mode downgrade so the drive
         * torque bleeds off smoothly instead of the speed-PI stepping it (the
         * SPORTS->ECO-at-full-throttle lurch). Seed from the current speed so
         * the ramp is spent on the real overspeed, not on the gap above it; an
         * upgrade or no change takes effect immediately (self-heals from 0 at
         * boot, since any real cap >= 0). */
        if (mp.max_speed_rpm >= s_eff_speed_cap) {
            s_eff_speed_cap = mp.max_speed_rpm;
        } else {
            float from = fminf(s_eff_speed_cap, speed_rpm_now);
            s_eff_speed_cap = fmaxf(from - SPEED_CAP_SLEW_DOWN_RPM_S * dt,
                                    mp.max_speed_rpm);
        }
        torque_cap = apply_speed_limit_pi(torque_cap, speed_rpm_now,
                                          s_eff_speed_cap, dt);
    }

    /* 6) Clamp the driver request against the cap */
    float torque_cmd = fminf(torque_ref, torque_cap);
    if (torque_cmd < 0.0f) torque_cmd = 0.0f;

    /* Is the DRIVER the binding constraint on the way down? Only a driver-driven
     * release may be slowed by the high-speed rate below; a reduction coming
     * from the cap (speed limit, thermal) or from DCLO is PROTECTIVE and must
     * keep the full declared cut rate — that invariant is why
     * FOC_TRQ_RAMP_DOWN_NM_S >= TRQ_SLEW_DECEL_NM_S is asserted at all. A brake
     * command also arrives as torque_ref = 0, so it is excluded explicitly:
     * braking must never be slewed. */
    uint8_t driver_release = (torque_ref <= torque_cap) &&
                             !can_d.brake_command_active;

    /* 7) DCLO: closed-loop DC current limit — local I_dc estimator +
     *    feedforward power cap + fast PI, eta^ calibrated from CAN 0x110
     *    (see the algorithm block in Torque_Management.h). */
    float torque_pre_dclo = torque_cmd;
    torque_cmd = apply_dclo_torque_limit(torque_cmd, omega_m, dt);
    if (torque_cmd < torque_pre_dclo) driver_release = 0U;   /* DCLO is binding */

    /* 8) Torque slew with measured dt. Accel rate is per-mode; the DECEL rate is
     *    speed-dependent in SPORTS on a driver release — see
     *    TRQ_SLEW_DECEL_HS_NM_S in Torque_Management.h. */
    float decel_rate = TRQ_SLEW_DECEL_NM_S;
#if TRQ_DECEL_HS_ENABLE
    if ((s_eff_mode == SPORTS) && driver_release) {
        float t = (speed_rpm_now - TRQ_DECEL_HS_START_RPM) /
                  (TRQ_DECEL_HS_END_RPM - TRQ_DECEL_HS_START_RPM);
        t = clampf_local(t, 0.0f, 1.0f);
        t = t * t * (3.0f - (2.0f * t));    /* smoothstep: no kink at either edge */
        decel_rate += t * (TRQ_SLEW_DECEL_HS_NM_S - TRQ_SLEW_DECEL_NM_S);
    }
#else
    (void)driver_release;
#endif

    float delta    = torque_cmd - s_prev_torque;
    float max_up   = mp.slew_accel_nm_s * dt;
    float max_down = decel_rate * dt;
    if (delta >  max_up)   delta =  max_up;
    if (delta < -max_down) delta = -max_down;

    s_prev_torque += delta;

    /* Update the boost budget from delivered mechanical power (one-cycle lag):
     * deplete above the continuous rating, refill below it (cruise / rest). */
#if PWR_BOOST_ENABLE
    float p_delivered = s_prev_torque * omega_m;             /* W (mech) */
    s_boost_budget_J -= (p_delivered - PWR_CONT_W) * dt;
    if (s_boost_budget_J < 0.0f)              s_boost_budget_J = 0.0f;
    if (s_boost_budget_J > PWR_BOOST_E_MAX_J) s_boost_budget_J = PWR_BOOST_E_MAX_J;
#endif

    return s_prev_torque;
}