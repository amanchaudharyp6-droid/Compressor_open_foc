# Session Handoff — Deceleration grab / jerk on throttle-release and mode switch

**Date:** 2026-08-05 · **Branch:** `OTA_Reintegrated` · **Vehicle:** HiRange XR, 96 V, `MOTOR_PROFILE_DEFAULT`
To resume on another machine: pull this branch, then tell Claude Code to read this file.

> ### ⚠ READ THIS FIRST — which code the observations came from
>
> The three symptoms below were observed on **`0f871b3`** (checked out detached for testing),
> which does **NOT** contain the torque-inverse fix. Branch HEAD (`1de0f51` and later) **does**.
>
> That matters for every current and torque figure in this document. On `0f871b3` a 40 Nm SPORTS
> command really delivers ~89 Nm and draws 236 Arms; with the fix it delivers 40 Nm at 143 Arms.
> **Re-measuring on branch HEAD will give different numbers for the same throttle position.** See
> `HANDOFF_HILLHOLD_AND_FW.md` → *Issue 3* for that defect. Every table here is `0f871b3`.

---

## The three symptoms as reported

1. **SPORTS, decel from 55–56 km/h and above** — very hard, strong grab, then violent jerking.
2. **ECO, decel from 46–47 km/h** — a grab, then mild jerking, then smooth.
3. **SPORTS → ECO mid-drive** — vehicle decels, then at ~47–48 km/h a grab and violent jerking.

1 and 2 happen **only on a hard, immediate throttle-to-zero.** A gradual release does not trigger
either. That single fact is the strongest diagnostic in the set — see *Why gradual release is fine*.

---

## Config facts that drive everything below

| Constant | Value | Where |
|---|---|---|
| `RPM_TO_KMPH` | 0.01044 | rpm = km/h × 95.785 |
| `SVM_VOLTAGE_LIMIT` | 96/√3 = **55.4 V** | `motor_default.h` |
| `REGEN_FADE_START_RPM` / `END` | 2500 / **3500** (26 / **36.5 km/h**) | `vehicle_config.h:325` |
| `REGEN_MAX_TORQUE_SPORTS` | 3.0 Nm, **faded to 0 above 3500 rpm** | `vehicle_config.h:249` |
| `REGEN_MAX_TORQUE_ECO` | 5.0 Nm, **no fade at any speed** | `vehicle_config.h:248` |
| `SPEED_LIMIT_ECO_RPM` | `ECO_MAX_SPEED` = 4500 rpm = **47.0 km/h** | `Torque_Management.h:60` |
| `SPEED_LIMIT_SPORTS_RPM` | `SPORTS_MAX_SPEED` = 6000 rpm = 62.6 km/h (never reached) | `Torque_Management.h:61` |
| `TRQ_SLEW_DECEL_NM_S` | 100 Nm/s | `Torque_Management.h:215` |
| `TRQ_REQ_SLEW_DOWN_NM_S` | 100 Nm/s (cascaded with the above) | `vehicle_config.h:373` |
| `SPEED_PI_KP` / `KI` | 0.05 / 0.25 | vehicle-validated, do not "restore" 0.15 |
| `ID_KI` / `IQ_KI` | 6.0 / 7.0 | `motor_default.h` |

Regenerate every number in this document with:

```
python tools/decel_envelope.py
```

---

## Issue 1 — SPORTS above 55 km/h: nothing is commanding that braking

**`REGEN_FADE_END_RPM` is 3500 rpm = 36.5 km/h. Above that SPORTS commands ZERO regen.**

So at 55 km/h the torque chain is not asking for any braking torque — it is ramping an 11 Nm
command to zero over ~110 ms. Whatever decelerates the vehicle, the torque chain did not request it.
**This is the fact that rules out the entire torque-command path** (throttle map, caps, slew rates,
regen config) as the cause of issue 1.

What it is instead: the current loop is commanding a voltage vector outside the achievable circle.

| km/h | rpm | T_cmd | Iarms | Vd | Vq | \|V\| | mod % | regen cap |
|---|---|---|---|---|---|---|---|---|
| 26 | 2490 | 21.1 | 126 | −45.3 | 10.4 | 46.4 | 84 | 3.0 |
| 31 | 2969 | 20.1 | 120 | −51.5 | 12.8 | 53.0 | 96 | 1.6 |
| 37 | 3544 | 16.9 | 101 | −52.6 | 17.8 | 55.5 | **100** | 0.0 |
| 42 | 4023 | 14.9 | 89 | −53.2 | 22.1 | 57.6 | **104** | 0.0 |
| 45 | 4310 | 13.9 | 83 | −53.5 | 24.7 | 58.9 | **106** | 0.0 |
| 47 | 4502 | 13.2 | 79 | −53.3 | 26.5 | 59.5 | **107** | 0.0 |
| 50 | 4789 | 12.4 | 74 | −53.5 | 29.1 | 60.9 | **110** | 0.0 |
| 55 | 5268 | 11.0 | 66 | −52.9 | 33.7 | 62.7 | **113** | 0.0 |
| 56 | 5364 | 10.7 | 64 | −52.7 | 34.6 | 63.0 | **114** | 0.0 |
| 58 | 5556 | 10.2 | 61 | −52.3 | 36.5 | 63.8 | **115** | 0.0 |

**SPORTS is over-modulated from 37 km/h upward.** At 55 km/h the loop wants 62.7 V of a 55.4 V
circle.

The mechanism, per axis in `FOC_Loop.c`: PI → clamp(±L) → add feedforward → clamp(±L) → per-phase
clamp in `apply_SVM_voltages()`, with `L = ±BusVoltage_V/√3` at both clamps.

- The per-axis clamps are ±55.4 **each**, so Vd = −52.9 and Vq = +33.7 both pass individually.
- The **pair** is 13% outside the circle. The per-phase clamp then truncates the vector.
- The anti-windup back-calculation is `(Vd_PID − rtb_Sum)` — the PI's **own** saturation only. It
  cannot see the clamp on `Vd_PID + Vff`, nor the per-phase clamp. Voltage the regulator believes
  it applied is discarded, so the current error persists and both integrators wind to their rails
  holding an error that can never be satisfied.

When the reference collapses, the integrators must unwind at roughly `Ki · err`:

| Axis | Ki | initial err | unwind rate | time from a 55 V rail |
|---|---|---|---|---|
| Iq | 7.0 | 0 − 81 = −81 A | ≈ 567 V/s | **~100 ms** |
| Id | 6.0 | 0 − (−43) = +43 A | ≈ 258 V/s | **~215 ms** |

That is a 100–200 ms window in which the applied voltage bears no relation to the demand — the
current goes where the back-EMF and the stale negative Vd take it, which is braking — followed by
an over-shoot as the integrators cross. A 5–10 Hz limit cycle is what "violent jerking" is.

### Why gradual release is fine

A slow release keeps `err` small, so the integrators track continuously and the loop never leaves
its linear region. Only a step demands a correction the saturated loop cannot make. This asymmetry
is the signature of integrator latching and is why the symptom is reproducible on command.

---

## Issue 2 — ECO at 46–47 km/h: real, commanded, and benign

**ECO never over-modulates**: 75% at 47 km/h, 90% even at 58 km/h. The loop has headroom the whole
way, which is exactly why this symptom is mild and clears itself.

| km/h | rpm | T_cmd | Iarms | \|V\| | mod % | regen cap |
|---|---|---|---|---|---|---|
| 37 | 3544 | 6.8 | 40 | 35.0 | 63 | 5.0 |
| 45 | 4310 | 5.6 | 33 | 40.4 | 73 | 5.0 |
| 47 | 4502 | 5.3 | 32 | 41.8 | **75** | 5.0 |
| 55 | 5268 | 4.6 | 27 | 47.5 | 86 | 5.0 |
| 58 | 5556 | 4.3 | 26 | 49.7 | 90 | 5.0 |

ECO has **no regen fade** — 5 Nm is commanded at every speed. Hard throttle-to-zero at 47 km/h is a
step from **+5.3 Nm drive to −5.0 Nm regen**, a 10.3 Nm reversal that flips Vd from −23.4 V to
+21.8 V (~45 V on the d axis). The feedforward `VdFF = −Lq·Iq_ref·ω_e` carries most of that swing
because it flips with `Iq_ref`, which is why it settles into smoothness.

**Conclusion: issue 2 is not a fault.** It is an un-ramped 5 Nm regen onset, and it is a different
problem from issue 1. Fixing it is a comfort change (ramp the onset), not a defect fix.

---

## Issue 3 — SPORTS → ECO: two events, one of them a genuine bug

### Event A — at the switch (55 km/h)

The vehicle is above ECO's 4500 rpm speed limit, so the speed-PI caps at 0 and the torque command
collapses. Same 113% over-modulated state as issue 1, same latch, same violence. Note this happens
**regardless of pedal position** — the mode switch alone produces the step.

### Event B — at 48.1 km/h, the speed-PI comes off the zero rail

`apply_speed_limit_pi` (`Torque_Management.c:411`) clamps its integrator to `[0, torque_in]`
**every cycle**, while conditional integration freezes it whenever `at_lo && err < 0`. Over the
limit, therefore, `s_speed_integ` does not wind down — it **parks at `torque_in`, its maximum**.
The cap then re-engages with the integrator already at full authority instead of rising from zero.

| rpm | km/h | `u_unsat` | state |
|---|---|---|---|
| 5268 | 55.0 | −33.84 | rail 0 |
| 4800 | 50.1 | −10.00 | rail 0 |
| 4700 | 49.1 | −4.89 | rail 0 |
| **4604** | **48.1** | **+0.01** | **releases** |
| 4550 | 47.5 | +2.78 | cap 2.78 Nm |
| 4500 | 47.0 | +5.33 | cap 5.33 Nm |

**4604 rpm = 48.1 km/h**, against the reported 47–48 km/h.

The existing code comment says the integrator "parks at 0 (frozen) instead of winding down". That
is true for the steep-grade case it was written for (where `torque_in` is large and the integrator
was already low), but **not** here: on a SPORTS→ECO switch `torque_in` collapses to the ECO power
cap (~4.6 Nm at 5270 rpm) and the unconditional clamp drags the integrator up to that value.

### Event C — the actual bug: regen cap steps 0 → 5 Nm on the mode switch

`update_regen_fade` (`Torque_Management.c:217`) applies the smoothstep **only in SPORTS**. The ECO
branch is a bare assignment:

```c
} else {
    /* ECO: NO fade — regen active across the whole range at the ECO torque */
    cap = REGEN_MAX_TORQUE_ECO;
}
```

Switching SPORTS→ECO at 5270 rpm therefore steps `max_Regen_Torque` from **0 → 5 Nm instantly**,
with no ramp. The comment three lines above it states the fade exists because *"a larger regen
command re-engaging at speed triggers a current-angle excursion (the decel jerk)"*.

**The mode switch does exactly the thing the fade was written to prevent.** This is the clearest
single defect found in this session.

---

## Ruled out, with numbers

| Suspect | Why not |
|---|---|
| **Speed limiter injecting braking** | `torque_cmd = fminf(torque_ref, torque_cap)` — a ceiling only, and `apply_speed_limit_pi` clamps its output to `[0, torque_in]`. It can never command negative torque. |
| **Commutation angle advance** (`16f438b`, `d1bbff0`) | At 55 km/h with `torque_final` ≈ 11 Nm, `trq_gate = (11−7)/8 = 0.5`, so advance = 0.05 rad mech = **8.6° electrical**, removed over ~40 ms as torque falls 11→7 at 100 Nm/s. Rotating the frame 8.6° shifts apparent Id by ~+12 A on a 92 A vector. Real, but an order of magnitude too small. It is also 0.3% of the electrical rotation rate at 265 Hz, so not a frame-slew problem. |
| **Torque command / slew rates** | SPORTS commands **zero** regen above 36.5 km/h, and the decel ramp is 110 ms for 11 Nm. Nothing in the command explains a strong grab. |
| **Field weakening module** | `Core/Src/Field_Weakening.c` does not exist on this commit — it was reverted. Not a factor. |
| **The torque inverse** | It makes every transient ~2.2× larger than labelled, so it *amplifies* all of the above, but it is not the trigger: the grab happens with a zero torque command. |

---

## ⚠ Where the tidy story breaks down — do not over-trust it

**The symptom survived the circular-limiter + anti-windup build.** On 2026-07-30 the report was:
*"if i was immediately making the throttle 0, the vehicle started to stop but it was very
aggressive, and the vehicle goes into oscillations and jerking, causing cutoffs. If i was manually
controlling the throttle, like a very gradual decrease then this didn't happen."*

A circular limiter makes modulation ≤ 100% by construction. If over-modulation and integrator
latching were the whole story, that build should have cleared issue 1. It did not. Two readings:

- the d-priority allocation in that build starved Vq (`vq_room = sqrt(vmax² − vd²)` → 0 when `|Vd|`
  reaches vmax), replacing one failure with another that looks identical from the seat; or
- the high-speed decel transient is marginal for a reason not yet isolated.

**Resolve this before investing in a circular-limit rewrite.** It is the open question of this
session.

---

## ⚠ You currently cannot measure any of this

`0x715` bytes `[0]` and `[1]` carried commanded Id and the modulation index ×100. They did **not**
survive the revert to `0f871b3` — `Send_on_CAN_715` (`fdcan_AL.c:682`) leaves both at zero:

```c
data[2] = MTPA_FWC_condition;  data[3] = pack_voltage;
data[4] = bus_voltage;         data[5] = lv_12v;   /* [0],[1] unused */
```

Decode when restored:

| Byte | Signal | Decode |
|---|---|---|
| `[0]` | commanded Id | `Id = data[0] − 180` A (always ≤ 0) |
| `[1]` | modulation index ×100 | `\|V\| / Vmax_SVM`. **100 = exactly at the SVM circle; >100 = asking for more voltage than the inverter can make.** |

Byte `[1]` must be sampled unconditionally so baseline and experimental runs compare directly.

---

## Plan, in order of value

1. **Restore the two telemetry bytes on `0x715`.** Zero behaviour change, and it converts this
   whole document from argument into measurement. If byte `[1]` reads > 100 during the grab, issue 1
   is confirmed; if it reads < 100, the over-modulation story is wrong and the open question above
   is the real one.
2. **Ramp the regen cap across a mode switch** — fixes Event C. Small, contained, no generated code.
   Simplest form: slew `max_Regen_Torque` toward its target instead of assigning it, so the ECO
   branch cannot step.
3. **Make `TRQ_SLEW_DECEL_NM_S` speed-dependent.** 100 Nm/s is a deliberate "responsive cut", but
   cutting 11 Nm in 110 ms at 113% modulation is asking a lot of a saturated loop. Slowing it above
   ~4000 rpm is low-risk and tests whether transient *rate* is what breaks it — independent of the
   root cause, and it needs no theory to be true.
4. **Wind the speed-PI integrator down** instead of letting the unconditional clamp park it at
   `torque_in` — fixes Event B's abrupt re-engagement at 48.1 km/h.
5. **Only then** revisit circular voltage limiting, with **Vq priority** rather than Vd, and only
   after step 1 has told you whether over-modulation is actually present.

Steps 1 and 2 are cheap and safe, and 1 is what makes everything else decidable.

---

## Facts worth not re-deriving

**Steady-state voltage model** (used for every mod % in this document):

```
Vd = Rs·Id − ωe·Lq·Iq
Vq = Rs·Iq + ωe·(Ld·Id + λ)      ωe = rpm · 2π/60 · P
mod % = 100 · √(Vd² + Vq²) / (Vdc/√3)
```

**HiRange XR constants**: λ = 0.0263 Wb, Ld = 0.148 mH, Lq = 0.386 mH, Rs = 10.47 mΩ, 3 pole pairs,
`MOTOR_PEAK_AC` = 353.55 A, `RPM_TO_KMPH` = 0.01044, `SVM_VOLTAGE_LIMIT` = 55.4 V.

**`0f871b3` current chain** (the broken inverse — see `HANDOFF_HILLHOLD_AND_FW.md` Issue 3):
`I_mag = 2·T_cmd/(3·P·λ)`, then MTPA `Id = A − √(A² + I_mag²/2)` with `A = λ/(4(Lq−Ld))` = 27.63 A,
`Iq = √(I_mag² − Id²)`. Delivered torque `= 1.5·P·(λ·Iq + (Ld−Lq)·Id·Iq)`.

**`BusVoltage_V` is pinned to 96 V nominal, deliberately** — not measured Vdc. Real Vdc makes the
vehicle slow as SOC dips. Do not "fix" this; it means the 55.4 V limit is a constant, not a
function of charge state.

**Mode envelope at a glance** (HiRange XR): ECO 16 Nm / 2514 W / 4500 rpm; SPORTS 40 Nm / 6284 W /
6000 rpm. The torque-speed table tracks the SPORTS power line from ~3000 rpm, so in SPORTS above
that speed the table *is* the power mapping.

**Observed top speeds**: ECO 46–47 km/h (its speed limit), SPORTS 55–56 km/h (voltage limited, well
below its 62.6 km/h speed limit). SPORTS never reaches its speed limiter — it runs out of volts
first, which is why the speed-PI plays no part in issue 1.
