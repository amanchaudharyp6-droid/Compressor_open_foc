#!/usr/bin/env python3
"""Reproduce every figure in HANDOFF_DECEL_JERK.md.

Answers two questions for the HiRange XR / 96 V / MOTOR_PROFILE_DEFAULT build:

  1. At what speed does each drive mode start commanding a voltage vector
     outside the achievable SVM circle?  Per-axis clamps are +/-Vdc/sqrt(3)
     EACH, so a pair can pass them individually and still sit outside the
     circle; the per-phase clamp then truncates it and the anti-windup
     back-calculation cannot see that -> integrators latch.

  2. Coasting SPORTS->ECO from above the ECO speed limit, at what speed does
     apply_speed_limit_pi() come off its zero rail?  Its integrator is clamped
     to [0, torque_in] unconditionally while conditional integration freezes
     it, so over the limit it parks at torque_in (its max) rather than winding
     down, and the cap re-engages at full authority.

MODELS THE CODE AS OF 0f871b3 -- i.e. WITH the non-salient torque inverse.
Branch HEAD carries the salient-inverse fix, which changes every current here;
see HANDOFF_HILLHOLD_AND_FW.md -> Issue 3.

Pure stdlib. Constants mirror Core/Inc/motor_profiles/motor_default.h,
Core/Inc/vehicle_config.h and Core/Inc/Torque_Management.h -- update together.
"""

import math

# --- motor_default.h ------------------------------------------------------
LAMBDA, LD, LQ, RS, POLEPAIRS = 0.0263, 0.000148, 0.000386, 0.01047, 3.0
OP_VOLTAGE = 96.0
SVM_VOLTAGE_LIMIT = OP_VOLTAGE / math.sqrt(3.0)          # 55.4 V

# --- vehicle_config.h (HIRANGE_XR) ---------------------------------------
RPM_TO_KMPH = 0.01044
REGEN_FADE_START_RPM, REGEN_FADE_END_RPM = 2500.0, 3500.0
REGEN_MAX_TORQUE_ECO, REGEN_MAX_TORQUE_SPORTS = 5.0, 3.0
SPEED_PI_KP = 0.05

# --- Torque_Management.c TORQUE_SPEED_TABLE (HIRANGE_XR) ------------------
TORQUE_SPEED_TABLE = [
    (0.0, 40.0), (500.0, 40.0), (1000.0, 38.0), (1500.0, 30.0),
    (2000.0, 24.0), (2500.0, 21.0), (3000.0, 20.0), (4400.0, 13.5),
    (5100.0, 11.5), (5300.0, 10.9), (5500.0, 10.4), (6000.0, 9.0),
    (6500.0, 8.0),
]

MODES = {
    "ECO":    dict(power=2514.0, torque_cap=16.0, speed_limit=4500.0,
                   regen=REGEN_MAX_TORQUE_ECO, fade=False),
    "SPORTS": dict(power=6284.0, torque_cap=40.0, speed_limit=6000.0,
                   regen=REGEN_MAX_TORQUE_SPORTS, fade=True),
}


def torque_speed_limit(rpm):
    """get_torque_speed_limit() -- linear interpolation over the table."""
    if rpm <= TORQUE_SPEED_TABLE[0][0]:
        return TORQUE_SPEED_TABLE[0][1]
    if rpm >= TORQUE_SPEED_TABLE[-1][0]:
        return TORQUE_SPEED_TABLE[-1][1]
    for i in range(len(TORQUE_SPEED_TABLE) - 1):
        lo, t_lo = TORQUE_SPEED_TABLE[i]
        hi, t_hi = TORQUE_SPEED_TABLE[i + 1]
        if lo <= rpm < hi:
            return t_lo + (rpm - lo) / (hi - lo) * (t_hi - t_lo)
    return TORQUE_SPEED_TABLE[-1][1]


def commanded_torque(rpm, mode):
    """min(table envelope, power cap, mode torque cap) -- steps 3-5 of
    Torque_Management_Update(), before the speed-PI cap."""
    omega = rpm * 2.0 * math.pi / 60.0
    return min(torque_speed_limit(rpm), mode["power"] / omega,
               mode["torque_cap"])


def current_refs(t_cmd):
    """0f871b3 chain: the NON-salient inverse reused as a total current
    magnitude, then the MTPA split ':5521'/':5522'."""
    i_mag = abs(2.0 * t_cmd) / (3.0 * POLEPAIRS * LAMBDA)
    a = LAMBDA / ((LQ - LD) * 4.0)
    i_d = a - math.sqrt(a * a + 0.5 * i_mag * i_mag)
    i_q = math.sqrt(max(i_mag * i_mag - i_d * i_d, 0.0))
    return i_d, (i_q if t_cmd >= 0.0 else -i_q)


def delivered_torque(i_d, i_q):
    """'Torque_calculation' ':5575' -- the SALIENT equation."""
    return (i_d * i_q * (LD - LQ) + i_q * LAMBDA) * POLEPAIRS * 1.5


def steady_state_volts(i_d, i_q, rpm):
    omega_e = rpm * 2.0 * math.pi / 60.0 * POLEPAIRS
    v_d = RS * i_d - omega_e * LQ * i_q
    v_q = RS * i_q + omega_e * (LD * i_d + LAMBDA)
    return v_d, v_q, math.hypot(v_d, v_q)


def regen_cap(rpm, mode):
    """update_regen_fade(). NOTE the ECO branch is a bare assignment -- no
    fade and no ramp, so a SPORTS->ECO switch above REGEN_FADE_END_RPM steps
    the cap 0 -> REGEN_MAX_TORQUE_ECO instantly."""
    if not mode["fade"]:
        return mode["regen"]
    if rpm >= REGEN_FADE_END_RPM:
        return 0.0
    if rpm <= REGEN_FADE_START_RPM:
        return mode["regen"]
    t = ((rpm - REGEN_FADE_START_RPM) /
         (REGEN_FADE_END_RPM - REGEN_FADE_START_RPM))
    return mode["regen"] * (1.0 - t * t * (3.0 - 2.0 * t))


def speed_pi_release_rpm(mode_name="ECO", start_rpm=5300.0, step=0.25):
    """Coasting down, the first rpm at which apply_speed_limit_pi() leaves the
    zero rail, given an integrator parked at its [0, torque_in] clamp."""
    mode = MODES[mode_name]
    rpm = start_rpm
    while rpm > 1000.0:
        t_in = commanded_torque(rpm, mode)
        if SPEED_PI_KP * (mode["speed_limit"] - rpm) + t_in > 0.0:
            return rpm, t_in
        rpm -= step
    return None, None


def main():
    print(f"SVM circle = Vdc/sqrt(3) = {SVM_VOLTAGE_LIMIT:.1f} V "
          f"(BusVoltage_V pinned to {OP_VOLTAGE:.0f} V nominal, deliberately)")
    print("mod % > 100 => commanded vector outside the achievable circle\n")

    speeds = (26, 31, 37, 42, 45, 47, 50, 55, 56, 58)
    for name, mode in MODES.items():
        print(f"=== {name}: {mode['power']:.0f} W, {mode['torque_cap']:.0f} Nm cap, "
              f"speed limit {mode['speed_limit']:.0f} rpm "
              f"({mode['speed_limit'] * RPM_TO_KMPH:.1f} km/h) ===")
        print(f"{'km/h':>5} {'rpm':>6} {'T_cmd':>6} {'deliv':>6} {'Iarms':>6} "
              f"{'Vd':>7} {'Vq':>7} {'|V|':>6} {'mod%':>5} {'regen':>6}")
        onset = None
        for kmph in speeds:
            rpm = kmph / RPM_TO_KMPH
            t_cmd = commanded_torque(rpm, mode)
            i_d, i_q = current_refs(t_cmd)
            v_d, v_q, v = steady_state_volts(i_d, i_q, rpm)
            mod = 100.0 * v / SVM_VOLTAGE_LIMIT
            if mod > 100.0 and onset is None:
                onset = kmph
            print(f"{kmph:5d} {rpm:6.0f} {t_cmd:6.1f} "
                  f"{delivered_torque(i_d, i_q):6.1f} "
                  f"{math.hypot(i_d, i_q) / math.sqrt(2.0):6.0f} "
                  f"{v_d:7.1f} {v_q:7.1f} {v:6.1f} {mod:5.0f}"
                  f"{'**' if mod > 100.0 else '  '} "
                  f"{regen_cap(rpm, mode):5.1f}")
        if onset:
            print(f"  -> over-modulated from ~{onset} km/h upward")
        else:
            print("  -> never over-modulated: the loop keeps headroom throughout")
        print(f"  -> commanded regen is ZERO above "
              f"{REGEN_FADE_END_RPM * RPM_TO_KMPH:.1f} km/h\n"
              if mode["fade"] else
              "  -> regen commanded at every speed, no fade, no onset ramp\n")

    rpm, t_in = speed_pi_release_rpm()
    print("SPORTS -> ECO coast-down (integrator parked at its [0, torque_in] clamp):")
    print(f"  speed-PI leaves the zero rail at {rpm:.0f} rpm = "
          f"{rpm * RPM_TO_KMPH:.1f} km/h, cap stepping 0 -> {t_in:.2f} Nm")
    for probe in (5268.0, 4800.0, 4700.0, 4650.0, 4604.0, 4550.0, 4500.0):
        t = commanded_torque(probe, MODES["ECO"])
        u = SPEED_PI_KP * (MODES["ECO"]["speed_limit"] - probe) + t
        state = "rail 0" if u <= 0.0 else f"cap {min(u, t):.2f} Nm"
        print(f"    {probe:5.0f} rpm ({probe * RPM_TO_KMPH:4.1f} km/h): "
              f"u_unsat = {u:+6.2f}  {state}")


if __name__ == "__main__":
    main()
