#include "Speed_Sensing.h"
#include "main.h"

void Speed_Filter(float Speed)
{
	static float prev_speed = 0.0f;
	static float prev_filtered_speed = 0.0f;

    d.rad_s = ((SPEED_FILTER_CONSTANT_1 * prev_filtered_speed) + (SPEED_FILTER_CONSTANT_2 *(Speed+prev_speed)));
    d.RPM = d.rad_s / 0.1047f;

    prev_speed = Speed;
    prev_filtered_speed = d.rad_s;
}

/* Speed from the angle step over one T_S. The sin/cos pairs of this and the
 * previous angle give sin and cos of the step (cross and dot product), and
 * atan2 gives the step itself, wrap-free.
 *
 * This used to return the cross product alone — sin(dtheta)/T_S — which reads
 * low as the step grows: -1.2 % at 2500 rpm, -4 % at 4500 rpm, -7.5 % at
 * 6488 rpm true, where it read 6000 (tachometer-confirmed on the bench,
 * 25-09-2026; ported from Pegasus/feature/unified_image/dev a25800b3). Every
 * speed cap, table and km/h figure now means true speed; the caps kept their
 * numbers, so they bind lower on the motor than they did before this fix
 * (SPORTS 6000 rpm was really ~6488).
 *
 * SPEED_SENSE_CLAMP_RAD_S is the glitch clamp. The old 733.038 on the
 * sin-based value sat at asin(0.733)/T_S = 822.8 rad/s (7857 rpm) true.
 * Raised to 1150 rad/s (10982 rpm) for the 10000 rpm TW-7 bench motor: it must
 * sit above OVERSPEED_TRIP_RPM (10300) or the clamped estimate could never
 * reach the trip. atan2 of a 1 ms step is unambiguous to pi/T_S (30000 rpm). */
#define SPEED_SENSE_CLAMP_RAD_S   1150.0f

#if defined(OVERSPEED_TRIP_RPM)
_Static_assert((int)(OVERSPEED_TRIP_RPM * 0.10471976f) < (int)(SPEED_SENSE_CLAMP_RAD_S),
               "OVERSPEED_TRIP_RPM must be below SPEED_SENSE_CLAMP_RAD_S, or the trip can never fire");
#endif

void Speed_Sense(float MechAngle)
{
    static float sine_rotor_prev;
    static float cosine_rotor_prev;

    const float sine_rotor   = sinf(MechAngle);
    const float cosine_rotor = cosf(MechAngle);

    const float step_sin = (sine_rotor * cosine_rotor_prev) - (cosine_rotor * sine_rotor_prev);
    const float step_cos = (cosine_rotor * cosine_rotor_prev) + (sine_rotor * sine_rotor_prev);

    sine_rotor_prev   = sine_rotor;
    cosine_rotor_prev = cosine_rotor;

    d.raw_rad_s = atan2f(step_sin, step_cos) * DERIVATIVE_CONSTANT;

    if (d.raw_rad_s >= SPEED_SENSE_CLAMP_RAD_S) {
        d.raw_rad_s = SPEED_SENSE_CLAMP_RAD_S;
    } else if (d.raw_rad_s <= -SPEED_SENSE_CLAMP_RAD_S) {
        d.raw_rad_s = -SPEED_SENSE_CLAMP_RAD_S;
    }

    Speed_Filter(d.raw_rad_s);
}
