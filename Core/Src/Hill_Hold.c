#include "Hill_Hold.h"
#include "main.h"

/* The hold torque must be inside the FOC estimator's envelope or the
 * current-ref model clips it regardless of this cap. */
_Static_assert((int)(HH_MAX_TORQUE) <= (int)(MAX_TORQUE),
               "HH_MAX_TORQUE must be <= MAX_TORQUE");

/* isr_foc_start runs at 10 kHz (d.Op_ts = 0.0001 s); the PI updates at
 * 1 kHz (HH_TS), matching the speed-feedback rate in rt_OneStep. */
#define HH_DECIM 10U

typedef enum {
  HH_INACTIVE = 0,   /* armed — waiting for rollback */
  HH_ACTIVE,         /* PI holding the vehicle at 0 RPM */
  HH_RELEASING,      /* hold budget spent — ramping torque out */
  HH_COOLDOWN,       /* released; re-arm only on driver action */
  HH_STANDBY         /* driver drove away — grade estimate retained */
} hh_state_t;

static hh_state_t hh_state = HH_INACTIVE;
static float    hh_integ      = 0.0f;  /* integrator (Nm), [0, HH_MAX_TORQUE] */
static float    hh_out        = 0.0f;  /* hold torque (Nm), [0, HH_MAX_TORQUE]*/
static float    hh_ref_nm     = 0.0f;  /* last driver torque demand seen (Nm) */
static uint32_t hh_hold_ticks = 0U;    /* 1 kHz ticks spent holding */
static uint8_t  hh_div        = 0U;    /* 10 kHz -> 1 kHz decimator */

static void hh_reset(hh_state_t next)
{
  hh_state      = next;
  hh_integ      = 0.0f;
  hh_out        = 0.0f;
  hh_hold_ticks = 0U;
}

/* Move hh_out toward target at no more than rate_nm_s. Every state runs its
 * output through here so a state edge can never step the torque reference:
 * entering the hold used to apply Kp*HH_ENTRY_RPM in one 1 kHz tick, and
 * leaving it used to drop the whole hold torque at once. The driveline takes
 * either as a shock and rings on the lash. */
static void hh_slew(float target, float rate_nm_s)
{
  float step  = rate_nm_s * HH_TS;
  float delta = target - hh_out;

  if (delta > step)       hh_out += step;
  else if (delta < -step) hh_out -= step;
  else                    hh_out  = target;
}

static void hh_step_1kHz(void)
{
  /* Gate on the FNR chart state (Throttle_State), not the requested
   * Drive_State: the chart is what signs T_gen into T_ref_FNR, so torque we
   * inject is only guaranteed to point the right way while the chart is
   * latched in Forward or Reverse. This also keeps hill hold out of the
   * Neutral / Throttle_Error chart states, where T_ref is applied unsigned. */
  bool dir_engaged = (FOC_Loop_Y.Throttle_State == Forward) ||
                     (FOC_Loop_Y.Throttle_State == Reverse);

  if (!dir_engaged ||
      (FOC_Loop_U.Speed_1_Torque_0 != FOC_MODE_TORQUE) ||
      can_d.brake_command_active)
  {
    /* Brake / neutral always win and also re-arm a COOLDOWN. */
    hh_reset(HH_INACTIVE);
    return;
  }

  /* Speed in the "drive frame": positive = moving the way the engaged
   * direction wants to go, negative = rolling back. Throttle_State is in the
   * model frame, same as d.RPM, so MOTOR_DIRECTION flip needs no handling. */
  float drive_sign = (FOC_Loop_Y.Throttle_State == Forward) ? 1.0f : -1.0f;
  float v_drive    = drive_sign * d.RPM;

  /* The driver owns the torque whenever their demand sits above the hold's
   * output: the max() floor means the throttle, not the hold, is what is
   * moving the vehicle. Used both to release and to protect the grade
   * estimate below — hh_ref_nm is only a true driver demand because the
   * caller passes d.torque_final, a variable this module never writes. */
  bool driver_owns = (hh_ref_nm > hh_out);

  switch (hh_state)
  {
   case HH_INACTIVE:
    /* Armed: no torque until a rollback crosses the entry speed. */
    hh_slew(0.0f, HH_SLEW_NM_S);
    if (v_drive < -HH_ENTRY_RPM)
      hh_state = HH_ACTIVE;
    break;

   case HH_ACTIVE:
   {
    /* Release only to the DRIVER: their throttle demand must have taken
     * over the max() floor AND the vehicle must be moving the intended way.
     * Never release on speed alone — on a shallow slope the hold's own
     * integrator overshoot crosses the exit speed, torque cuts, the vehicle
     * rolls back and re-engages: a rollback/catch limit cycle that shows up
     * as oscillation with net uphill creep.
     *
     * Hand over to HH_STANDBY rather than resetting: hh_integ is the learned
     * grade torque, and the grade has not changed just because the driver
     * pulled away. Wiping it here is what made a blip-and-lift re-enter the
     * hold with nothing learned — rebuilding from zero straight into the
     * HH_MAX_TORQUE clamp, throwing the vehicle forward, dumping the
     * integrator again, and ringing. */
    if ((v_drive > HH_EXIT_RPM) && driver_owns)
    {
      hh_state      = HH_STANDBY;
      hh_hold_ticks = 0U;
      hh_slew(0.0f, HH_SLEW_NM_S);
      break;
    }

    float err = -v_drive;                /* > 0 while rolling back */

    /* Integrator: wind up while rolling back; FREEZE inside the standstill
     * deadband so it settles at the slope's holding torque instead of pumping
     * past it; unwind only for SLOW forward creep, to trim a slightly-too-high
     * estimate. Clamp to [0, HH_MAX_TORQUE] — push-only, clamp doubles as
     * anti-windup.
     *
     * The unwind is bounded to (HH_CREEP_RPM, HH_UNWIND_MAX_RPM). A forward
     * excursion FASTER than the upper bound is a transient — the driver
     * blipped the throttle and lifted, or the arrest overshot — and must NOT
     * dump the estimate. Dumping it was the observed fault: the next rollback
     * rebuilt hh_integ from zero, wound past the grade to arrest, overshot
     * forward, dumped again, and ring-cycled (hh_out toggling 0<->+ / CAN
     * 0x710 byte[4] flapping while byte[5] stayed ACTIVE). Freezing instead
     * means the P term bleeds hh_out down through the excursion but hh_integ
     * survives, so the hold recovers to the grade torque and settles.
     *
     * The lower bound is HH_CREEP_RPM, BELOW the wind deadband: the arrest
     * surplus that makes the vehicle creep slowly forward lives under the
     * deadband, where a symmetric freeze would leave it untrimmed forever, so
     * the trim reaches lower on the forward side than the wind-up does on the
     * backward side.
     *
     * Also gated on !driver_owns: forward motion the driver's own throttle is
     * causing says nothing about the hold being too strong. */
    if (v_drive < -HH_DEADBAND_RPM)
      hh_integ += HH_KI * err * HH_TS;
    else if ((v_drive > HH_CREEP_RPM) && (v_drive < HH_UNWIND_MAX_RPM) &&
             !driver_owns)
      hh_integ += HH_KI * HH_UNWIND_GAIN * err * HH_TS;

    if (hh_integ > HH_MAX_TORQUE)
      hh_integ = HH_MAX_TORQUE;
    else if (hh_integ < 0.0f)
      hh_integ = 0.0f;

    /* P term stays on the raw error: with a 0-speed setpoint, Kp*err is
     * -Kp*v — pure damping in both directions, including inside the
     * deadband where the integrator is frozen. */
    float target = HH_KP * err + hh_integ;
    if (target > HH_MAX_TORQUE)
      target = HH_MAX_TORQUE;
    else if (target < 0.0f)
      target = 0.0f;
    hh_slew(target, HH_SLEW_NM_S);

    /* Holding at 0 RPM parks DC current in one phase pair — cap how long
     * we cook the same windings. */
    #if HH_MAX_HOLD_MS > 0
    hh_hold_ticks++;
    if (hh_hold_ticks >= (uint32_t)HH_MAX_HOLD_MS)
      hh_state = HH_RELEASING;
    #endif
    break;
   }

   case HH_RELEASING:
    hh_slew(0.0f, HH_RELEASE_RATE_NM_S);
    if (hh_out <= 0.0f)
      hh_reset(HH_COOLDOWN);
    break;

   case HH_STANDBY:
    /* Driver drove away under their own torque. The output bleeds to zero so
     * it can never fight the throttle, but the grade estimate is kept and
     * only decays slowly, so a lift-and-roll-back on the same slope
     * re-engages at roughly the right holding torque instead of rebuilding
     * from zero. Brake / neutral / FNR / mode change still wipe it through
     * the reset at the top. */
    hh_slew(0.0f, HH_SLEW_NM_S);

    hh_integ -= HH_STANDBY_DECAY_NM_S * HH_TS;
    if (hh_integ < 0.0f)
      hh_integ = 0.0f;

    if (v_drive < -HH_ENTRY_RPM)
      hh_state = HH_ACTIVE;
    break;

   case HH_COOLDOWN:
   default:
    /* Don't re-engage while still rolling back — that would just re-burn
     * the hold budget in a loop. Re-arm once the rollback has stopped;
     * brake or FNR change re-arm via the reset at the top. */
    hh_slew(0.0f, HH_SLEW_NM_S);
    if (v_drive > -HH_ENTRY_RPM)
      hh_state = HH_INACTIVE;
    break;
  }
}

float HillHold_Apply(float ref_torque_nm)
{
  hh_ref_nm = ref_torque_nm;  /* driver demand, used for takeover release */

  hh_div++;
  if (hh_div >= HH_DECIM)
  {
    hh_div = 0U;
    hh_step_1kHz();
  }

  return (hh_out > ref_torque_nm) ? hh_out : ref_torque_nm;
}

bool HillHold_IsActive(void)
{
  /* Literally "producing torque". State alone no longer answers this: every
   * state slews its output, so a hold can still be bleeding out after the
   * driver has taken over and the machine has moved to HH_STANDBY. */
  return (hh_out > 0.0f);
}

float HillHold_GetTorque(void)
{
  return hh_out;
}

uint8_t HillHold_GetState(void)
{
  return (uint8_t)hh_state;   /* enum order matches the codes in Hill_Hold.h */
}
