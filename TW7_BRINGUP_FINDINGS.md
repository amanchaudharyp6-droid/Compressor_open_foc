# TW-7 bench bring-up: findings, 29 Oct to 1 Nov

What the 29 Oct and 1 Nov bench logs show about holding 10500 rpm on the TW-7, how sure we are of each point, and what to do next. Companion to [TW7_BRINGUP_CHANGES.md](TW7_BRINGUP_CHANGES.md), which lists the firmware changes. Terms follow [CONTEXT.md](CONTEXT.md).

- **Bench:** free rotor, supply set to 50.9 V with a current limit of about 60 A. It cannot sink current.
- **Firmware:** commit `10c821f8` (`MOTOR_PROFILE_HIRANGE_TR`) for every log here, 29 Oct and 1 Nov.
  - τ was set over 0x115 after each boot: 0 / 150 / 170 / 210 / 250 µs in the earlier logs, then 130 µs from `10500` on.
  - Speed PI 4/8 (×0.8 gain set) unless noted.
- **Logs** (`HMC_Logs/Bringup_gen4/29_10`):
  - Earlier: `sweep`, `wow`, `10000_rpm`, `9500RPM`, `10000_pi_changed`.
  - τ = 130 batch: `10500`, `10200`, `10200_2` (idle only), `10100_repeat`, `10100_repeat_tf_happened`, `10500_unstable`, `10500_unstable_2`, `10500_unstable_3`.
  - The board was power-cycled before every log.
- **Logs, 1 Nov** (`HMC_Logs/Bringup_gen4/1_11`): `tuned_9900rpm_trial`, `trial_b`, `massive`, `retry_of_massive`, `retry_of_massive_2`, `retry_of_massive_3`, `105000_working`, `power_cycled_fixes_not_applied`, plus `pegasus_20261101_203355` from the GUI logger.
- **Vetting:** independent reviews recomputed the numbers from the parsed CSVs and read the code. A model of the 10 kHz current loop was checked against the measurements. The sections below §2 are the record as it was written; where they conflict with the Summary or the Theory check, those two win.

**Confidence labels**

| | Meaning |
|---|---|
| **Proven** | Several independent checks in the data agree, and the code explains it |
| **Measured** | Read directly from the logs |
| **Strong lead** | Fits the data and a model reproduces it, but no bench test has isolated it yet |
| **Inferred** | The most likely explanation of measured data; not tested |

---

## 1. Summary (status at 1 Nov, after the deep review)

This section is the current position. Sections further down are kept as the record; where they disagree with this one, this one wins. The plan is in the Deep review.

1. **10500 rpm holds on the bench, using a workaround** (Measured).
   - There are two logged passes: 33 s and 26 s, speed σ about 1.2 rpm, no wobble, no drift, and no trip down to where each log ends (5813 and 1406 rpm).
   - The settings were λ 0.0085 and speed PI 3.0/6.1 sent over CAN after boot, with τ 130. Neither log reaches 0 rpm.
2. **The 3.3 Hz wobble comes from the Vq feed-forward being clamped on its own** (Proven on the bench; mechanism reproduced in simulation).
   - **Bench:** an A/B on the same calibration put Vq_FF on its clamp 100 % of the time with λ 0.01121 (wobble 0.5 → 5.6 A in 6 s) and 0 % with λ 0.0085 (wobble 0.08 A).
   - **Simulation:** a nonlinear 10 kHz model of the current, FW and speed loops reproduces a 3.3 Hz limit cycle only with the separate clamp and the logged measurement error.
     - Removing that clamp makes it stable at any λ.
     - A complex-vector PI also makes it stable.
     - Changing the FW gain doesn't.
   - The clamp engages because the measured |Id| reads low, which pushes Vq_FF up.
3. **The current measurement is wrong in both phase and amplitude at speed, and part of that is a firmware bug** (Proven in code; sizes from fits).
   - **Phase V is one PWM period stale** (`adc.c:152`, `main.c:490-491`). This causes the 2fe ripple in Id/Iq (16 A at 10500 rpm, matching prediction) and about 46 µs of extra average lag.
   - **Sense filter:** the RC filter is about 132 µs (schematic), plus a residual delay of about 26 µs (fit).
   - **Lag at 10500 rpm:** about 59° in total, of which τ 130 corrects 41°.
   - **The true operating point at 10500 rpm** is about id −85…−95 A, iq 5–8 A, 60–67 Arms and about 1 kW. The logs show −72 A and 27 A, and power computed from them is 1.6 kW.
   - **Ld can't be pinned from these logs:** 63.5–77 µH, depending on how the lag is split.
4. **Boot-to-boot angle scatter (σ 2.9°) is a decode bug** (Proven: fixed and bench-measured on `unified_image`).
   - The firmware decodes the MT6816 PWM as a 4096-clock frame. The real frame is 4119 clocks: the vehicle-branch fix `d56e7e09` quotes 16 high, 4095 data and 8 low, and was bench-measured against Z.
   - A frame of 4119 gives a position-dependent error spanning 23 counts. That predicts σ 2.9° and −19 counts for the one checkable pair; the logs show σ 2.9° and −21.
5. **The angle between the encoder and the rotor has shifted in lasting steps** (Measured; the cause is unknown).
   - Offset-free levels are −34°, −56° and −97° electrical. Every autotune landed lower, 1.148 → 0.916 rad (−13° mechanical).
   - The steps are far larger than the decode error, and the encoder signals look healthy.
   - Only inspection can say whether the magnet, the sensor board or the rotor moved. Z can't detect it, because Z comes from the same magnet.
6. **Any angle error becomes a runaway, because speed mode can't command negative torque** (Measured; margins from simulation).
   - Today's measurement error happens to give about 20° of tolerance.
   - **Fixing the ADC on its own would cut that to about 6°.** A negative Iq floor (−15 A) restores 15°.
   - Order matters: angle accuracy and the negative floor first, then the ADC.
7. **Other firmware items** (details in the Deep review):
   - About 450 kHz of encoder and DMA interrupts that serve no purpose.
   - Slow anti-windup, and a speed integrator range that doesn't match its output.
   - Duty scaled by a slow bus value capped at 51 V.
   - A 30 ms speed filter inside the speed loop.
   - Slow over-current protection, and an all-switches-open safe state.
   - A divide by Lq − Ld with no guard.
8. **The bench supply** started most of the earlier trips at wobble peaks (§5). None of the holds without the wobble sagged it. No supply-side current has ever been logged.
9. **Other branches already fix several of these** (Measured in code; see "Other branches").
   - The vehicle lines fixed the PWM decode, which confirms the 4119-clock frame.
   - `unified_image` and TURBO fixed the stale phase-V sample.
   - They also fixed the interrupt load and fast over-current.
   - They also exposed a **safety defect still present here:** `__HAL_TIM_MOE_DISABLE` does nothing while PWM runs, so only the PD15 inhibit stops the bridge on a fault.
   - Phase 1 now ports those fixes instead of writing new ones.

## Deep review: model and base firmware (1 Nov, late)

**Method.** Two read-only code reviews (control logic, and base firmware) were followed by my own checks against the code, the logs and a nonlinear simulation. A third review, meant to attack the theory, stopped at a usage limit; I did its main checks myself, listed below. Scripts are in `scratchpad/n3/`: `stale*.py`, `fit3.py`, `fit4.py`, `sim*.py`.

### Confirmed defects

1. **Phase V is one PWM period old** (code fact, matches the logs).
   - What happens: ADC2 converts U and then V, 7.8 µs each (`adc.c:152,174`). The interrupt fires after the first conversion (`ADC_EOC_SINGLE_CONV`). The callback reads both results at `main.c:490-491`, while V's conversion is still running. The HAL then clears the second flag at the end of the interrupt. So V is about 92 µs older than U.
   - Effects in the dq frame:
     - A ripple at twice the electrical frequency, (2/√3)·sin(ωe·92 µs/2) of the current.
     - About 46 µs of extra average lag.
     - Amplitude reads ×1.11 high at 10500 rpm.
   - Check against the logs: slow-frame Id/Iq paired with the encoder count from the next frame fits that ripple with R² 0.73 at 10500 rpm and 0.88 at about 8800 rpm.

     | Speed | Measured ripple amplitude | Predicted amplitude |
     |---|---|---|
     | 10500 rpm | 16.5 A | 19.9 A |
     | about 8800 rpm | 16.1 A | 15.5 A |

   - **This is the "2fe ripple".** It isn't sensor mismatch.
   - Through Kp it puts about 2 V of ripple at 1.75 kHz on Vd/Vq. That's why peak modulation reaches 1.00 while the mean is 0.85. It may also be the "hum".
2. **The boot PWM decode assumes a 4096-clock frame; the reviewer reads the MT6816 frame as 4119** (`main.c:678`, `Helper_Functions.c:125`). I couldn't open the datasheet to confirm the 4119, but the fit below supports it.
   - The resulting error spans 23 counts, about 10° electrical, depending on rest position. A uniform spread of 23 counts gives σ 6.6 counts, or 2.9°: **exactly the measured boot scatter.**
   - The one checkable rest-to-boot pair (`10100_repeat` → `tf_happened`) predicts −19 counts; −21 was measured.
   - TIM5 resolution is 1 µs, and the decode uses a single frame.
3. **The boot angle is loaded 2 s late** (`isr_handlers.c:82`). TIM2 counts during current calibration, then gets overwritten with the stale boot value. That's harmless on a stopped bench rotor, but wrong on a rolling vehicle.
4. **TIM2 encoder interrupts serve no purpose** (`main.c:211` `HAL_TIM_Encoder_Start_IT`, priority 1, no TIM2 callback). At 10500 rpm they fire at about 358 kHz. The ADC1 DMA half/full interrupts add about 91 kHz (continuous circular scan). The ISR-time telemetry can't show the CPU cost, because the ADC ISR has the highest priority. The cost lands on TIM17 and the main loop; an estimate of 35–40 % is not measured.
5. **The feed-forward is clamped on its own, ahead of the PI** (`FOC_Loop.c:506-513`, `574-588`). See Simulation below for why this is the wobble.
6. **Anti-windup is slow, and the speed integrator range doesn't match its output.**
   - Back-calculation time is 0.2–0.26 s for the current PIs, against Ti ≈ 50 ms; it's 0.7 s for speed.
   - The speed integrator is clamped to ±283 A, but its output is 0…283 A, so it can wind down to −283 A (`FOC_Loop.c:1672-1679`).
7. **Speed mode can't command negative torque** (`SPEED_PID_OUT_LOWL` 0). That's what turns any frame error into a runaway.
8. **Duty is normalised by a 10 ms-filtered bus value capped at 51 V,** not the bus voltage at that instant. During a sag or swing the inverter's gain is wrong by the ratio of the two.
9. **Smaller items:**
   - The MTPA path divides by Lq − Ld without a guard (`Current_Ref_Estimator.c:416`).
   - The feed-forward has no Rs·I term.
   - The gain-set switch keys on the measured torque.
   - The Ld/Lq derate keys on |ω·T_meas|.
   - States aren't reset on entering FOC_START.
   - Several encoder fault checks can never fire, or are wired wrong.
   - The phase over-current check runs at 1 kHz on the filtered current, and TIM1 break is off.
   - The safe state is all switches open, so the bus reaches 89–102 V after a high-speed trip.

### Simulation

The model is nonlinear and runs at 10 kHz: the dq plant (exact discretisation), the firmware's PI with back-calculation, the per-axis and circle limits, the closed-loop FW integrator, the 1 kHz speed PI with its 30 ms filter and 0 A floor, a free rotor with 1 kW of loss, and the logged measurement error (−18° residual, ×0.9).

| Case at 10500 rpm | Result |
|---|---|
| Vq_FF clamped, as now | **3.3 Hz limit cycle**, Id_ref 27 A p-p, speed σ 43 rpm (the bench showed 3.3 Hz) |
| Same, but with a perfect current measurement | Stable. The clamp only engages because measured \|Id\| reads low, which pushes Vq_FF up |
| Vq_FF not clamped, λ 0.01121 | Stable, σ 0.2 rpm |
| Vq_FF not clamped, λ 0.0085 | Stable (same as the bench workaround) |
| Clamped + complex-vector PI, g = 0.5 | Stable |
| Clamped + FW gain scheduled, or FW KI 100 | **Not fixed:** the runs became unstable or ran away |
| At 9000 / 7500 rpm, clamped | 3.6 Hz lightly damped / 4 Hz decaying (the bench was quiet at 9000 and 7500) |

**Runaway margin against encoder frame error**, starting from a settled state:

| Measurement | Holds at | Runs away at |
|---|---|---|
| As now: −18° residual (τ 130, stale V) | −20° | not reached |
| Stale V fixed, τ 130: −3° residual | −6° | −10° |
| Stale V fixed, Iq floor −15 A | −15° | — |

**Fixing the ADC on its own would make the drive less tolerant of angle error.** Today's measurement error is what protects it. Fix the angle accuracy and give the speed loop a negative Iq floor first.

### Corrections to earlier claims

- **"Datasheet Ld fits"** (Theory check): **not identifiable.** Delays on the current side and on the voltage side, RC, and Ld trade off against each other.
  - With the stale V (92 µs, from the code) and RC 132 µs (from the schematic), the best fit is Td 26 µs, output-delay error −5 µs, drop 0.5 V and **Ld ≈ 74 µH**, at 0.07 V rms.
  - Ld lands anywhere in 63.5–77 µH depending on assumptions. That's plausible given cross-saturation at iq ≈ 0. Only an independent inductance test would settle it.
- **"Td ≈ 73 µs":** mostly the stale V (46 µs average). Td is about 26 µs.
- **True current at 10500 rpm** is 85–95 A peak (60–67 Arms), and true iq is 5–8 A. Power is 0.93–1.0 kW. All of these hold under either model, and the τ-sweep collapse holds too (47.7 / 47.8 / 45.9 A).
- **The FF + PI clamp idea is back,** in its clean form: no separate feed-forward clamp. The simulation supports it.
- **"Put λ 0.0085 in the profile" is withdrawn.** It's a patch on a symptom.
- **Hardware against fitted voltage delay:** the code's timeline gives 116 µs from the angle read to the pulse centre. The fit and the original low-speed calibration give about 80–85 µs. That's unresolved; a scope would settle it.

### Plan, cleanest first

| Phase | Change | Where | Risk | Why |
|---|---|---|---|---|
| 0 | Inspect and mark the magnet and sensor board; supply ammeter; clamp probe; scope a GPIO for ISR timing | bench | none | Settles alignment, power, amplitude and the delay |
| 1a | PWM decode for the 4119 frame, average about 32 frames, finer TIM5 tick; load TIM2 at the reading; re-run the autotune | base | low | Boot scatter goes from ±5° to well under 1° |
| 1b | Use `HAL_TIM_Encoder_Start` without interrupts; drop the ADC1 DMA half/full interrupts or slow the scan | base | low | CPU back |
| 1c | Flash the 0x422 / 0x40A telemetry | base | none | Checks 1a and phase 2 directly |
| 2a | **Remove the separate Vd/Vq feed-forward clamps.** Keep the per-axis and circle limits on the total, and the H4 anti-windup. λ back to about 0.0106, speed PI back to 4/8 | model | low–medium | Simulation: no wobble at any λ |
| 2b | Negative Iq floor in speed mode (−15…−30 A), with a DC-link limiter that shrinks it toward 0 as Vdc rises above about 52 V | model | medium | Restores runaway margin; must come before 3a |
| 2c | Anti-windup Kb ≈ 1/Ti; speed integrator clamped to its output range | model | low | Faster recovery |
| 2d | Derate off on the bench (later: schedule it on \|I\|); MTPA guard; Rs·I in the feed-forward | model | low | Correctness, mainly for vehicles |
| 2e | Optional: complex-vector term with a CAN gain that boots at 0 | model | medium | Robust if the feed-forward is wrong |
| 3a | ADC: sample U and V at the same instant (dual-ADC injected), shorter sampling time; then re-fit τ | base | medium | Removes the 2fe ripple and frees voltage headroom |
| 3b | Normalise duty with the bus voltage measured each ISR; keep the filtered, capped value for limits and the FW target | base | low | Correct gain in sags |
| 3c | Board rev: cut the 132 µs RC (C12 100 nF → about 10 nF); Z RC to under 20 ns | hardware | — | Removes most of the lag and amplitude error |
| 4 | Speed: a PLL at 10 kHz instead of a 1 ms difference and a 30 ms filter; retune with the datasheet J | base | medium | Speed-loop phase margin, the 6/rev alias |
| 5 | Back-EMF angle check at 1–3 krpm (blocks the ramp); fast per-ISR OC trip; active-short safe state above about 8000 rpm; fix the dead encoder checks | base | medium | Safety; the angle check catches alignment shifts that Z can't |
| 6 | -O2, DTCM, D-cache; then 16–20 kHz | base | medium | More margin everywhere |

**Z correction.** Once the PWM decode is fixed, it adds little at boot. It can't see a magnet slip, because Z comes from the same magnet. With EXTI priority 3 and a 1 µs RC, it isn't reliable at speed. Clean use: snap TIM2 at the first Z below about 300 rpm using a count_at_z stored by the autotune, then only monitor the Z-to-Z count above that.

**Leave alone:**
- The FW structure: closed-loop voltage FW is standard. Its gain didn't help in the simulation.
- Park, SVM and the MTPA maths.
- H2/H3, until 3a.

## Other branches: what they found and fixed (2 Nov)

**Method.** Three read-only reviews read every commit not in `tw-7_bringup`:
- the vehicle lines: HiRange XR/TR 2.0, HiLoad XR 2.0, LivGuard and TnV;
- the engineering branches: `unified_image`, TURBO, `dyno_test_xr` and `igbt_open_foc`.

I checked the key commits and the HAL macro myself. Dumps are in `scratchpad/branches/`.

**Lineage.** `tw-7_bringup` forked from the HiRange TR line at `0f871b30`, so `Pegasus/HiRange_TR_2.0/dev` is the natural source for cherry-picks. `unified_image` replaced the Simulink FOC with hand-written C, so its fixes need porting rather than picking.

### Defects on `tw-7_bringup` that other branches already fixed

| Defect here | Fixed on | Evidence there | Status |
|---|---|---|---|
| **`__HAL_TIM_MOE_DISABLE` does nothing while PWM is enabled.** The HAL macro (`stm32h7xx_hal_tim.h:1261`) clears MOE only when every CCxE/CCxNE is 0. So on a fault, only the PD15 inhibit stops the bridge (`Helper_Functions.c:291`, `main.c:690`, `tim17_handlers.c:54`) | TR dev `fc1bede8` (unconditional MOE), `59f06c95` (PVD and CPU lockup into the TIM1 break, BOR 2.7 V, latch after 3 resets), `14ba2914` | Code review. The option-byte write isn't verified on hardware | **Safety. Port first.** |
| Stale phase V (Deep review #1) | `unified_image` `fa93eb66` → `804e208c` → `bf4aa9ff`; TURBO port `ef588767` (6 files) | Same diagnosis: V about 85 µs old, W = −U−V inherits it. Simulated 2fe torque ripple 1.49 → 0.15 Nm. Their fitted sensor lag drops by about 0.4 Ts, matching our 46 µs. **Not run on a unit** | Fix: `ADC_EOC_SEQ_CONV` in the CubeMX USER block, 32.5-cycle sampling, the callback returns unless JEOS, NO_OF_SAMPLES halved, and a missed-deadline counter `g_foc_ccr_late` |
| Boot PWM decode 4096 vs 4119; one capture; loaded late (Deep review #2–3) | `unified_image` `d56e7e09` → TR `0a4bea82`, `ffff09e6`, `719e9772` (XR `a8e65f5f`, `c97ec4bb`, `dcaf539a`); standstill gate `7169ffdc` | **Bench-measured on the same MT6816:** 13 counts of boot scatter before, Z read 231–232 over 7 boots after. The commits quote the frame as 16 clocks high, 4095 data, 8 low (4119 total) | Fix: n = 4119·duty − 16, TIM5_PSC 0, circular median of 5 taken at the TIM2 write, frame and 12 V checks. Re-run the autotune after |
| Averaged offset computed from the ×1000 staging registers, so Neutral uses a value 1000× too large until reboot (`tim17_handlers.c:37`) | `12c0c339` / `e26f50246` | Vehicle, 12 Aug: Id went positive in Neutral | One line. Port |
| TIM2 encoder interrupts; ADC1 DMA half/full interrupts | `unified_image` `ea54585c`, `bf4aa9ff` | Measured: TIM2 about 200 k/s at 6000 rpm; DMA 30–46 k/s, 10–12 % CPU | Port |
| Over-current at 1 kHz; Id/Iq trip equal to its clamp | XR `0b105c14`, TR `accaf382`, HiLoad `51ed78a7`; `unified_image` instant line | Vehicle ID_IQ_OC at 460 A against a 435 A clamp | Port, **after** the stale-V fix (the 2fe ripple inflates peaks). Keep the clamp and the trip as separate constants |
| Anti-windup 4–5× slow | `unified_image` Kaw = Ki/Kp (`foc_config.h:95`) | Design | Matches Deep review phase 2c (about 20.6 d / 26.7 q here) |

### Where other branches agree or disagree with our findings

- **Separate feed-forward clamp (our wobble cause):**
  - Every branch has it, and others name it as a problem. `unified_image` says the model clamped "twice, before and after the feedforward"; `dyno_test_xr` `eb7db255` replaced the FF clamp but never ran it.
  - **Nobody else reproduced the 3.3 Hz cycle.** Our evidence is the bench A/B plus our own simulation.
  - Vehicle experience with voltage-limit allocation is mixed. Unbounded d-priority starved Vq and gave violent decel; proportional scaling brought noise back (HANDOFF_DECEL_JERK.md). Removing the FF clamp while keeping our bounded circle limit (0.9 r for Vd) is not vehicle-tested.
- **Z re-sync:**
  - Both vehicle lines tried re-anchoring to Z and **back away from it**: late Z reads gave false corrections of 7 counts (`unified_image`) and about 32 counts rms above 5000 rpm (TnV).
  - Z is used for monitoring only (ENC_IDX_LOCK_ENABLE 0). **Don't re-anchor to Z.**
- **Field weakening:**
  - `unified_image` warns that an Id-domain voltage loop "reverses sign beyond the ellipse centre and runs away". Our `FW_CL_ID_FLOOR_K` 1.1 lets Id go 10 % past −λ/Ld. On TW-7 that's about −185 A, against a true −95 A in use, so there's margin, but set the factor to ≤ 1.0.
  - The HiLoad and TR experience: their FW gain had to be low (TR FW_KFW 0.002 → 0.0002).
  - Our simulation says changing the FW gain doesn't fix the wobble.
- **Bus normalisation:**
  - The vehicles deliberately pin the model bus at or above OP_VOLTAGE (raw Vdc "visibly slows the vehicle" at low SOC). Any change must keep the FW target on the pinned value.
  - The dyno notes say normalising by a different bus changes the loop gain.
- **Speed mode floor 0:** the same everywhere. Vehicles run in torque mode with an app-level speed limiter that uses conditional integration (`Torque_Management.c:442-460`), which is a pattern worth copying. `unified_image` sims also show a bare-rotor runaway.
- **Angle advance on vehicles:** XR uses a fixed 0.1 rad mechanical advance, gated by torque. It cured the 26–45 km/h noise after the FW feed-forward and anti-windup patches had "zero effect". It's empirical; it's off on LivGuard and TW-7.
- **Output delay:** `unified_image` derives 1.1 Ts (107 µs from angle read to valley). Our code timeline gives 116 µs with a roughly 9 µs angle read. The 85 µs constant is a fitted lump.
- **Stale-V size:** about 85 µs (`unified_image`, assumes the rev V ADC clock) against our 92 µs (rev Y clock). Check REV_ID.

### Other items found

- **Clock configuration (check):** VOS3 with APB2/3/4 at HCLK/1 = 100 MHz (`main.c:410,447-450`). The reviewer says the VOS3 APB limit is 50 MHz. If so, it's twice the limit. Verify against the STM32H743 datasheet's maximum-clock table before anything else depends on it.
- **400 MHz** on `unified_image`: VOS1 cut the ISR from 56 to 42 µs (−27 %) and costs 40–90 mA. Not cleared for vehicles. Its next steps were DTCM placement and trimming `HAL_ADC_IRQHandler` (about 6 µs).
- **CAN:** `TxBuffersNbr` 5 + FIFO 32 = 37 (`fdcan.c:64-65`), a depth the `unified_image` review calls undefined.

### Vehicle-side items to pass on (outside the bench's scope)

- XR/TR lack the salient torque inverse (`1de0f514`), so every Nm figure there sits on an inflated torque axis.
- **HiLoad staging `978a3078`** raises the Id/Iq clamp and the OC trip together (110 → 125 %). The clamp, about 495 A, now sits above the 480 A phase trip. Its comment is stale, and this is a platform that has already lost three power stages.
- Staging and Production on HiLoad lack the 6 Oct MOE, reset-guard and boot-seed fixes.
- OV detection is still 1 kHz on the vehicles; the 121 V peak came from latency.
- No branch has a runtime encoder plausibility check. The A/B/Z fault checks can't fire on any of them.

### Revised phase 1 (port, don't write)

| Step | Port | Notes |
|---|---|---|
| 1-safety | TR `fc1bede8`, `59f06c95`, `14ba2914` | Bench-check PD15 and MOE behaviour; check the BOR option write |
| 1a | `0a4bea82`, `ffff09e6`, `719e9772`, plus `12c0c339` | Re-run the autotune after |
| 1b | `ea54585c` / the DMA part of `bf4aa9ff` | Interrupt load |
| 1c | our 0x422 / 0x40A telemetry | Already built |
| 3a, later | TURBO `ef588767` | Keep tw-7's CAN-set τ instead of its fixed 1.45 Ts. Needs phase 2b first (runaway margin) |

## Theory check: do the textbook equations hold?

**Superseded in part (Deep review).** This section didn't model the stale phase-V sample, a code fact. With it included:
- The delays, RC and Ld can't be separated, so Ld is 63.5–77 µH, not "datasheet fits".
- "Td ≈ 73 µs" is mostly the stale sample (46 µs average); about 26 µs remains.
- True current at 10500 rpm is 85–95 A peak (60–67 Arms), not exactly 95 A.
- **Still holding:** the textbook equations, the τ-sweep collapse, true iq of 5–8 A, power of about 1 kW, the τ-window picture, and the complex-vector PI result.

**Yes.** The standard steady-state equations are:

- `Vd = Rs·id − ωe·Lq·iq`
- `Vq = Rs·iq + ωe·(Ld·id + λ)`

They assume id and iq are the true currents. On this board they aren't, and that's the missing piece. There is no extra motor term. The `k` factor proposed earlier for the feed-forward isn't motor theory: it's the sense filter's amplitude loss, placed in the wrong spot. **That proposal is withdrawn.**

### The sense chain

Each phase current passes through a first-order filter and a plain delay before the Park transform, which then rotates it by ωe·τ. At steady state the true current is:

`I_true = I_logged · e^(−jωe·τ) · (1 + jωe·RC) · e^(jωe·Td)`

`RC` is 132 µs from the schematic. `Td` is about 73 µs from the fit, and likely covers the second RC stage, the sensor and the time between the ADC sample and the angle read. A filter's phase isn't a constant delay, which is why a single τ can't be right at every speed.

| Speed (rpm) | Total lag (µs) | Total lag (°) | Left over with τ 130 (°) | Amplitude read |
|---|---|---|---|---|
| 3000 | 203 | 18.3 | 6.6 | 0.98 |
| 6000 | 198 | 35.7 | 12.3 | 0.92 |
| 9000 | 191 | 51.6 | 16.5 | 0.85 |
| 10000 | 189 | 56.6 | 17.6 | 0.82 |
| 10500 | 187 | 59.0 | 18.0 | 0.81 |

### Fit of the textbook equations

Both axes, 1000–10500 rpm, five logs (`tuned_9900rpm_trial`, `trial_b`, `105000_working`, `power_cycled_fixes_not_applied`, `pegasus_20261101_203355`). Each log has its own λ and frame offset. Rs is scaled for temperature.

| Model | rms error (V) | Max error (V) | Result |
|---|---|---|---|
| Datasheet Ld/Lq, τ taken as the whole lag | 1.89 | 4.04 | Fails |
| Ld/Lq free, τ taken as the whole lag | 0.32 | 1.84 | Lq 21 µH, which isn't physical |
| **Datasheet Ld/Lq, RC 132 µs, Td and inverter drop fitted** | **0.16** | **0.66** | Td 73 µs, output-delay error −2 ± 2 µs, drop 0.89 V |
| Same, RC also fitted | 0.12 | 0.41 | RC 143 µs, Td 67 µs |
| Same, RC and Ld also fitted | 0.09 | 0.52 | RC 121 µs, Td 81 µs, Ld 68.5 µH |

So **datasheet inductances work**, and a free fit of Ld lands within 8 % of the datasheet. The earlier "Ld about 73–77 µH" came from correcting amplitude but not phase, and is withdrawn. Lq can't be fitted on its own, because the true iq is so small.

The fitted λ falls as the motor heats: 10.4 mWb at about 50 °C to 10.0 mWb at about 71 °C. That's up to 4 % below the datasheet back-EMF figure, and it trades off against the inverter drop term, so treat it as 10.0–10.8 mWb.

### Independent checks (none of these went into the fit)

- **The 29 Oct τ sweep collapses to one current.** Three τ settings at 6000 rpm, a different day and boot, map to the same true current:

  | τ (µs) | Logged Id / Iq (A) | True id / iq (A) | \|I\| (A) |
  |---|---|---|---|
  | 0 | −33.9 / 33.0 | −50.6 / 7.6 | 51.2 |
  | 150 | −43.4 / 13.6 | −48.7 / 7.4 | 49.3 |
  | 210 | −47.1 / 4.6 | −50.8 / 6.8 | 51.2 |

- **The earlier measured lag matches.** The sweep implied 205–208 µs at 6000 rpm; this model gives 198 µs. It also explains why the implied lag fell with speed.
- **The true torque current agrees across logs.** At 9900 rpm the logged Iq reads 26.8, 27.1, 22.8, 22.5 and 13.3 A across the five logs. After correction, in the rotor frame, those become 8.2, 7.9, 7.9, 7.9 and 8.7 A. A free rotor's drag shouldn't depend on τ or the boot, and it doesn't.
- **The inverter drop matches the dead time.** 100 ticks at 100 MHz is 1 µs, which gives (4/π)·Vdc·td·fsw ≈ 0.6 V. The fit gives 0.6–0.9 V, which leaves room for device drop.
- **The 85 µs output-delay compensation holds up.** Fitted error is −2 to −16 µs.
- **Textbook Vq at 10500 rpm:** 5498 × (63.5 µH × −94.9 A + 10.27 mWb) + 0.03 V = 23.4 V. Measured: 22.95 V.

### The real operating point

From `pegasus_20261101_203355`, in the controller's frame (the fitted frame offset for this boot is small). Power is 1.5·(Vd·id + Vq·iq) with the commanded voltages, so it's the power the controller asked for, including the inverter drop term.

| Speed (rpm) | Logged Id / Iq (A) | True id / iq (A) | Logged power (W) | True power (W) |
|---|---|---|---|---|
| 3000 | 0 / 4.4 | −0.6 / 4.5 | 114 | 116 |
| 5000 | −19.1 / 8.3 | −21.5 / 4.9 | 375 | 264 |
| 7000 | −51.6 / 17.5 | −60.3 / 5.1 | 911 | 522 |
| 9000 | −67.1 / 24.7 | −84.1 / 5.4 | 1363 | 816 |
| 10500 | −71.8 / 27.6 | −94.9 / 5.0 | 1587 | 1014 |

- **Real current at 10500 rpm is about 95 A peak, or 67 Arms,** which is 86 % of the rated 77.8 Arms. The over-current trips see the reading, not the real value.
- **The logged "Iq 27 A" isn't torque current.** About 20 A of it is field-weakening current rotated into the q-axis by the residual lag.
- **The bus-droop data can't separate the two power columns.** Both fit a straight line (21 mΩ or 36 mΩ). No supply-side current was ever logged, so earlier "34 A" figures were derived from logged power, not measured.

### Why τ 130 works better than the "correct" value

Two separate effects (the second is model-only):

1. **Runaway margin.** The under-correction makes Iq read high, which keeps Iq_ref away from its 0 A floor. At τ 130 the margin is about 21° of frame shift. At the correct τ it would be about 4–5°. That's consistent with τ 250 running away.
2. **A narrow stable window.** In the closed-loop model, the 1 Hz slow mode loses damping at low τ, and a mode near 1.1 kHz goes unstable at high τ:

   | τ at 10500 rpm (µs) | 110 | 130 | 150 | 170 | 190 |
   |---|---|---|---|---|---|
   | Slow-mode damping | −0.17 | −0.05 | +0.06 | +0.17 | +0.27 |
   | Fast mode (\|z\| over 1 is unstable) | stable | stable | stable | 1.012 | 1.025 |

   The model is slightly pessimistic: it calls τ 130 marginal, and the bench held 33 s. The shape is the useful part. A 1.1 kHz mode wouldn't show in 200 Hz telemetry, so the logs can't confirm or rule it out.

## Options and recommendation (first pass)

**Superseded in part (Deep review).** The simulation adds a cleaner first fix than any option here: drop the separate feed-forward clamp. The plan table in the Deep review replaces the Recommendation below. The rollback table still applies, except that the 0x422/0x40A telemetry is now kept.

### What the literature says

- **The equations** are the standard dq model. Measurement-chain errors are a separate, known topic: low-pass filters on current feedback delay the phase at high speed ([Park & Kim 2016](https://koreascience.or.kr/article/JAKO201612455011297.pdf)), and measurement error compensation has its own literature ([Kim, Sul & Lee 2014](https://doi.org/10.1109/TIA.2014.2301873)). I didn't find a paper treating the combined amplitude-and-phase correction at this operating point, so that part rests on the first-order maths and the checks above.
- **Cross-coupling at low sample ratio** is the classic high-speed problem, and the standard answer is the complex-vector regulator ([Briz, Degner & Lorenz 2000](https://minds.wisc.edu/items/1ee8b0b2-5c2e-4e39-933c-b75cb8009b07)), with follow-up work for low ratios ([Yim, Sul et al. 2009](https://doi.org/10.1109/TIA.2009.2013600), [Wang et al.](https://unpaywall.org/10.1631%2FFITEE.1601390)). The full texts of these weren't available to me; the control form below is my own derivation of the plant-pole cancellation, and the model checks it.
- **Delay:** 1.5 sample periods is the usual estimate ([Nottingham](https://nottingham-repository.worktribe.com/OutputFile/5367842)), and the usual fix is a voltage-angle advance, as H2 does. Here the fit supports 85 µs.
- **Feed-forward from flux maps** replaces inductances with lookup tables in current ([Gemassmer et al., KIT](https://publikationen.bibliothek.kit.edu/1000047994/3585795)), which also recommends predicting flux from measured currents, not references. That matters for high-torque saturation. This bench isn't there: datasheet L already fits.
- **Iron-loss resistance models** ([TH Nürnberg note](https://www.th-nuernberg.de/fileadmin/institute/elsys/ELSYS_Notes/EN-0013.pdf)) add a parallel branch. They'd improve loss estimates, not control, at this stage.

### Options

The model is the closed-loop dq model with the sense chain, at 10500 rpm. Tracking is Id against Id_ref at 3.3 Hz. Coupling is how many amps of d-axis error a 1 V q-axis disturbance causes at 3.3 Hz. These are model numbers, except where marked.

| Option | Code | Tracking | Coupling (A/V) | Stable? | Bench status |
|---|---|---|---|---|---|
| Original: Vq_FF clamped | none | 0.14 (bench 0.25) | 2.65 | Slow mode marginal | Wobbles, trips |
| **λ 0.0085 + PI 3/6.1** (today) | none, CAN only | 0.81 | 2.6 | Same | **Passed twice** |
| `k` in the feed-forward | new hand-edit | 1.21 (overshoots) | 2.6 | Same | **Withdrawn** |
| Correct the measurement, textbook FF | new hand-edit | 0.99 | 2.8 | **Unstable from about 9000 rpm at 1.1 kHz** | Not tried |
| **Complex-vector PI, g = 0.5** | 2 terms | 1.00 | 0.06 | Yes, \|z\| 0.93 | Not tried |
| Complex-vector PI + corrected measurement | 2 terms + hand-edit | 1.00 | under 0.05 | Yes, \|z\| 0.92 | Not tried |

**How the complex-vector PI works.** The plant's cross-coupling puts a lightly damped pole at ±jωe. Adding a speed-scaled cross term to each integrator cancels it, so the loop no longer depends on the feed-forward being right. In the model it tracks at 1.00 even with Vq_FF sitting on its clamp. It needs no fitted constants, only ωe and the existing Kp values:

```c
integrator_d += (... + Id_error*Ki_d - g*we*Kp_q*Iq_error) * Ts;
integrator_q += (... + Iq_error*Ki_q + g*we*Kp_d*Id_error) * Ts;
```

Model sensitivity at 7000 and 10500 rpm, feedback uncorrected, τ 130:

| Cross gain g | 0 | 0.25 | 0.5 | 0.75 | 1.0 | 1.25 | −0.25 |
|---|---|---|---|---|---|---|---|
| Worst \|z\| | 1.000 | 0.967 | 0.934 | 0.906 | 0.990 | 1.037 | 1.040 |

So g 0.25–0.75 is well inside the stable range at nominal values, while above about 1.1 or with the wrong sign it's unstable. Over a wide plant spread (Ld 50–75 µH, Lq 68–105 µH, RC 100–150 µs, Td 40–100 µs, up to 11500 rpm) g = 0.5 and 0.75 stay stable (worst |z| 0.995–0.996). g = 1 just crosses 1.0 (1.006), and so does g = 0.25 at the extreme corner (1.027). Plain PI reaches 1.05 on the same spread. That makes g = 0.5 the sensible target.

This is a linear model. It doesn't include the voltage limit, the field-weakening loop or the speed loop.

**Why correcting the measurement isn't first.** It's the right theory, but it does three things at once: it removes the runaway margin, the model says it destabilises the plain PI, and it changes what the over-current trips see. It belongs after the complex-vector PI and a negative Iq floor.

### Recommendation (superseded by the Deep review plan)

1. **Now: put the tested settings in the profile.** λ 0.0085, speed PI 3.0/6.1, τ 130 as the boot default. Four constants, no new control code, and exactly what passed twice. The cost is a wrong λ in the estimator's torque estimate, which doesn't matter on a free rotor.
2. **Next: try the complex-vector PI,** with a CAN gain that boots at 0. Start at g = 0.25 below 3000 rpm to prove the sign, then raise it. If it works, λ goes to its real value and speed PI back to 4/8.
3. **Later: correct the measurement** and move τ, only if true torque and current matter (for example a vehicle build), and only with step 2 and a negative Iq floor.

### Can the model be improved?

| Item | Change? | Why |
|---|---|---|
| Ld, Lq | **Keep datasheet values** | They fit. No need for fitted values |
| λ | Use about 10.6 mWb once step 2 works | Datasheet back-EMF and the fit agree; 11.21 is high |
| Ld/Lq derate | **Set to 0 on the bench** | Driven by a false torque estimate, up to the 15 % cap in old logs. The datasheet's 25 % / 23 % applies at peak current |
| Output delay 85 µs | Keep | Fit supports it |
| τ | **Leave at 130** until the Iq floor and step 2 are done | See above |
| Iq floor (`SPEED_PID_OUT_LOWL` 0 A) | Try about −10 A | The model has about 1 kW of losses to spend, so a small negative floor shouldn't regenerate. Check the regen path first |
| λ against temperature | Not needed | ±0.3 mWb is about ±1.5 V, which the PI covers |
| Dead-time compensation | Not needed | About 0.6–0.9 V, and the PI covers it |
| Flux maps, iron-loss model | Not needed on the bench | Only matter at high torque or for loss estimates |
| Measured-current decoupling | No | Model: marginally unstable at 10500 rpm with today's uncorrected feedback (\|z\| 1.001) |
| Current PI gains | Revisit with step 2 | Ki/Kp is 20–27 rad/s against R/L of 70–97 rad/s, which is what the magnitude-optimum rule sets it to |

### What can be rolled back

| Item | State | Recommendation |
|---|---|---|
| 0x422 feed-forward telemetry: three blocks in `FOC_Loop.c`, plus `fdcan.h` and `fdcan_AL.c` | Uncommitted, never flashed | **Drop.** It's the only new code in generated files, and the values can be computed offline from logged Id_ref, speed and bus voltage |
| 0x40A encoder diagnostics | Uncommitted, never flashed | **Drop.** Z comes from the same magnet, so it can't see a slip. Boot scatter is only about 3° |
| Parser2 and GUI decoding of those two frames | Uncommitted | Drop with them; the columns just stay empty otherwise |
| GUI trace logger, "Parse last log", Parser2 pairing and missed-frame fixes, `test_trace_log.py` | Uncommitted | **Keep.** Tools only, proven on hardware |
| H8 (V_s unit delay), H10 (`Vd_next`), `BENCH_FORCE_PWM_C`, `Min/Max_DC_V` | Committed | **Remove**, as already listed in the change sheet |
| FW Iq ceiling | Committed | Candidate. Check whether it ever engaged |
| H4/H5 circle limit | Committed | Keep for now. It only acts in trips, but peak modulation is already 1.00 |
| H2 output delay, H3 τ, H1 closed-loop FW | Committed | Keep. Each is needed and supported |
| Proposals: `k` feed-forward, TIM2 filter change | Never coded | **Drop.** Each was aimed at a cause now ruled out |
| Proposals: remove the separate FF clamp; Z re-sync | Never coded | **Revived** in the Deep review: the clamp removal is phase 2a. Z is optional, as a low-speed snap plus monitoring |

## 2. Where we are

| Speed | Best result | Log |
|---|---|---|
| 9000–9500 rpm | σ 1.0–1.4 rpm, repeatable | several |
| 10000 rpm | λ default: 30–37 s holds, σ 1.3–6 rpm, wobble grows slowly | `10000_rpm`, `10100_repeat`, `10500_unstable_3` |
| 10200 rpm | λ default: 28 s hold, wobble growing. λ 0.0085: 20 s, σ 1.5 rpm | `10200`, `massive` |
| 10500 rpm | λ default: 3.5–6.5 s before a trip. **λ 0.0085: 33 s and 26 s, σ 1.2 rpm, no wobble** | `10500`, `105000_working`, `pegasus_20261101_203355` |

### 1 Nov: run A of the feed-forward test (`1_11/tuned_9900rpm_trial`)

- Settings are the firmware defaults, confirmed from the echoes: λ 0.01121, speed PI 4/8, τ 130. There was a new angle autotune before this run (EEPROM offset 1.055).
- **Boot was normal:**
  - Vd −1.6 V at 3300 rpm during the ramp (back-EMF angle −2.6°);
  - back-EMF angle 8.3° at 9900 rpm, inside the 8–11° cluster of earlier normal boots.
- **Holds:**
  - **9000 rpm:** σ 1.3 rpm.
  - **9900 rpm, 29 s:** σ 4.6 rpm. The 3.3 Hz Id_ref ripple stayed steady at 1.5 A rms.
  - **9950 rpm, 10 s:** σ 3.9 rpm. The 3.3 Hz envelope grew 2.3 → 4.2 A, about +6 % per second, doubling in roughly 11 s. The 10500 trip came at about 8–9 A.
  - The bus stayed at 47.7–48.8 V and power peaked at 1.8 kW, so the supply played no part this time.
  - The log ends at 7840 rpm partway through the ramp-down, with no trip up to that point.
- **Tracking:** Id followed Id_ref at a gain of 0.25 and −92° at 2.5–4.5 Hz. Every hold from 9000 to 10500 rpm, across five boots and two calibrations, gives 0.22–0.34 at −84 to −94°. This is consistent with Finding 2 and doesn't depend on the boot.
- **New: the 10 Hz "ringing" at 9900 rpm is a 6-per-revolution speed ripple aliased by the 1 kHz speed loop.**
  - A narrow line in measured speed sits at |6·f_mech − 1000 Hz|.
  - It follows that formula through every ramp: 29/31 and 20/20 one-second windows in two 29 Oct logs, and 6/8 today.
  - It sits at 20 Hz in both 10200 rpm holds and at 50 Hz at 9500 and 10500 rpm.
  - At 9900 and 10100 rpm it lands on the ~10 Hz speed-loop mode and is amplified about 3–4×: 3.8 rpm rms and 1.3 A rms of Iq_ref.
  - At 9950 rpm it moves to 4.8 Hz, where the speed loop rejects it.
  - It's harmless at 10500 rpm (50 Hz).
  - Avoid 9900 and 10100 rpm as hold points. Near 9967 and 10033 rpm it would sit at 3.3 Hz, on top of the wobble.
  - Where the 6/rev comes from (encoder pattern or mechanical) is not known.

### 1 Nov: "trial B" (`1_11/trial_b`) was not the intended test

- **λ was never changed.** The 0x404 echo reads 0.01121 for the whole log, and no 0x103 frame was sent.
- **What did change from run A:**
  - speed PI 3.0/6.1, sent at 0 rpm; with λ unchanged, this just lowers the speed-loop gain by 24 %;
  - τ 150 instead of 130;
  - a new boot, worse than A: Vd −3.2 V at 3300 rpm (back-EMF angle −7.3°, against −2.6° in A).
- **Result:**
  - 9000 rpm: σ 1.0 rpm.
  - 9950 rpm, 2.6 s: σ 5.3 rpm, 3.3 Hz envelope 1.4 → 2.6 A.
  - 10200 rpm: the envelope was already 8–9 A on arrival, σ 14 rpm.
  - Then the usual sequence: speed jump, Id_ref down to −97 A, 2.5 kW, bus 48 → 32 V in about 40 ms, BUS_VOLTAGE_UV_ERROR. Tracking gain was unchanged at 0.23 / −88°.
- **Verdict:** not comparable with A, because three things changed and the intended one didn't. It is consistent with "lower speed-loop gain doesn't help".
- **Noisy Id/Iq plots:**
  - The 10 Hz slow frames carry single samples. At 9000 rpm they show a ripple at twice the electrical frequency of about 11–13 A (R² 0.5 against 2θe). It's about 4 A at 6000 rpm and absent at 2000 rpm.
  - The 200 Hz fast frames (5 ms means) are clean. Error to reference is 0.2–0.6 A rms on steady holds; the 1.8 A at 9900–9950 rpm is the 3.3 Hz wobble.
  - **(Superseded: it's the stale phase-V ADC sample; see Deep review.)** The ripple grows faster than the current does, so it's more likely a phase-to-phase mismatch in the sense chain than a current-loop gain problem. A clamp-on probe would show whether it's real current or measurement.
  - Through Kp it adds about 1.6 V of voltage ripple and uses about 0.1 of the modulation-index headroom (peak 0.97 against a mean of 0.85).

### 1 Nov: real run B (`1_11/massive`). The feed-forward fix works; a new blocker appears

- **Settings, confirmed from the echoes:** λ 0.00849, speed PI 3.0/6.1, τ 130. The EEPROM offset is 1.046, so the angle autotune was re-run.
- **Boot:** Vd −0.66 V at 3300 rpm (back-EMF angle 0.0°), the best-centred boot so far. The boot offset from run A is the same at 3300, 9000 and 9950 rpm (2.4–2.8°).
- **The 3.3 Hz wobble is gone.** Compare the "Before" column with the B results:

  | Hold | Speed σ | Id_ref swing 1–8 Hz | Before (same speeds, λ 0.01121) |
  |---|---|---|---|
  | 9950 rpm, 4 s | 0.9 rpm | 0.08 A | σ 3.9, 2.5 A and growing |
  | 10200 rpm, 20 s | 1.5 rpm | 0.07–0.14 A | σ 3.9, 2.5 A (29 Oct) |
  | 10500 rpm, about 15 s | 1.2–1.3 rpm | 0.10 A | σ 7.3, 5.0 A, then a trip |

  - The improvement is 15–40×, far larger than the boot-to-boot scatter. This is one run, so repeat it.
  - Tracking gain can no longer be measured, because Id_ref barely moves.
- **New blocker: the angle reference drifts at 10200 rpm and above.**
  - From 978 s onward the controller's frame rotated steadily: back-EMF angle +11.2° → −11.9° over 38 s, at −0.48°/s at 10200 rpm and −0.62°/s at 10500 rpm, speeding up at the end.
  - It was a rigid rotation: φV and φI both moved +6° per 16 s, while |V| (23.7 V), |I| (73 A), power (1.5 kW) and speed stayed constant.
  - Measured Iq fell 29 → 0 A while the speed held. The true torque was unchanged; the frame was rotating under it.
  - At 1016.07 s Iq_ref reached its 0 A floor (`SPEED_PID_OUT_LOWL`), so the speed loop had no braking left. The rotor ran 10507 → 10800 rpm in 0.2 s and tripped OVERSPEED_ERROR. The link then sat at 91–97 V from uncontrolled generation.
  - No earlier hold drifted (|slope| ≤ 0.02°/s): 29 Oct 10200 rpm for 26 s at 43–46 °C, 10500 rpm for 4.5 s at 42 °C, 10000 rpm for 33 s at 58–62 °C, today's 9900 and 9950 rpm.
  - Nothing in firmware writes TIM2 after boot, and FOC_ANGLE_ADVANCE is 0. A rigid rotation of V and I means TIM2 is drifting against the rotor, about 52 counts in 38 s.
  - **Lead (retracted later, see §6):** the encoder loses counts at ≥10200 rpm (A/B at 174–179 kHz, quadrature edges 1.4 µs apart). The TIM2 input filter (IC1F/IC2F = 10: fDTS/16 with N = 5 at a 100 MHz timer clock) needs each level stable for about 0.8 µs, which leaves only about 1.7× margin.
  - The motor was 70–75 °C, against 41–62 °C on runs that didn't drift, so heat in the encoder is the likely trigger. This is not yet proven. A `count_at_z` telemetry would prove or disprove it.
  - A magnet slipping on the shaft is unlikely at constant speed, since nothing loads it. The next boot check will rule it out: Vd should be back near −0.65 V.
- **Heat:** the motor reached 66 → 75 °C in this log. With a free rotor all input power is loss. (**Superseded:** this table uses the logged current, which overstates power above about 4500 rpm. See the Theory check: about 1 kW at 10500 rpm.)

  | Speed (rpm) | 4000 | 5000 | 6000 | 8000 | 9000 | 10000–10500 |
  |---|---|---|---|---|---|---|
  | Loss | 150 W | 340 W | 620 W | 1.1 kW | 1.3 kW | 1.5 kW |

  - Fundamental copper loss is only about 60 W, so the rest is iron, magnet and harmonic loss once field weakening starts.
  - That was estimated at about 3× the loss at the rated 4 kW output. With the corrected power it's closer to 2×, still with natural cooling only.

### 1 Nov: retries after `massive`. The angle reference is now off by about 40° on every boot

- **`retry_of_massive` and `retry_of_massive_2` are two power-ups with the same EEPROM offset (1.046) and λ 0.00849.**
  - At 1000–4000 rpm the back-EMF angle is **−38°** and **−46.6°**, constant with speed. In `massive` it was 0.0°, and in run A and trial B −2.6° and −7.3°.
  - Vd at 3300 rpm read −12.5 V and −14.6 V, against the −4 V power-cycle limit.
  - |E|/ωe at these points gives λ ≈ 10.9 mWb at 47–49 °C, matching the datasheet.
- **Both runs ran away as soon as field weakening started, at about 4100 rpm:**
  - Id_ref went negative, the hidden torque Id·sin(err) grew, and the speed loop cut Iq_ref to its 0 A floor.
  - **Retry 1:** speed 4480 → 5180 rpm, then BUS_VOLTAGE_UV_ERROR. The rotor kept accelerating to 6544 rpm after the trip.
  - **Retry 2:** 4400 → 10818 rpm in about 2 s. Id_ref drove to the −147 A floor (−1.1·λ/Ld with λ 0.0085), because voltage saturation made field weakening dig deeper, which added more forward torque. That positive feedback ended in OVERSPEED_ERROR, and the link reached 102 V.
- **Interpretation (lead):** the shift survives power cycles. At each boot the PWM absolute reading re-preloads TIM2, so lost counts would have been cleared, and the offset is unchanged.
  - That points to a mechanical shift between the MT6816 magnet and the rotor of about 8° mechanical. Either the magnet slipped on the shaft or the sensor board rotated.
  - It most likely happened progressively during the hot, high-speed `massive` run, where the in-run drift was already about −23°.
  - **To confirm:** inspect and mark the magnet and board. A fresh autotune would land about 0.14–0.16 rad from 1.046.
- **This also reopens the 29 Oct boot scatter.** Mechanical slip between runs could explain part of it, not only the single PWM reading. A `count_at_z` reading can't tell a slipped magnet apart, because Z comes from the same magnet. Only a rotor-referenced check, the back-EMF angle, can.
- **Encoder hardware, from the schematic:**
  - MT6816 rated to 25000 rpm, so it's not near its limit.
  - A/B: 10 Ω + 1 kΩ + 100 pF, τ = 100 ns. That's fine for 179 kHz.
  - Z: 10 kΩ + 100 pF, τ = 1 µs. At 10500 rpm a one-step Z pulse lasts about 1.4 µs, so it only reaches about 75 % of its swing. Fix this before relying on a Z-based re-sync at speed.

### 1 Nov evening: new autotune, 10500 rpm held, and an encoder check from the logs

- **Logs:**
  - `retry_of_massive_3` is standstill only: PCAN-View's trace keeps only about the last 150 s, so the earlier attempts were overwritten.
  - `105000_working` is the same power-up as `retry_of_massive_3`, since τ and λ were already set when it started.
  - `power_cycled_fixes_not_applied` is a fresh power-up with τ 130 only.
- **The autotune moved the EEPROM offset from 1.046 to 0.916 rad** (CW 1.06 → 0.93): −0.13 rad, or −37° electrical. That matches the −38° / −47° back-EMF measurement made before the autotune.
- **`105000_working` (λ 0.00849, PI 3/6.1, τ 130):**
  - 10500 rpm held for about 33 s, σ 1.2–1.45 rpm, Id_ref 1–8 Hz swing 0.08 A, 0 % alternation, motor 64–68 °C.
  - Ramp-down had no trip down to 5813 rpm, where the log ends.
  - This is the first run that meets the "cleared" limits in a log, apart from the unlogged ramp-down below 5800 rpm.
- **Clean λ A/B on the same calibration and τ:** `power_cycled_fixes_not_applied` (λ 0.01121, PI 4/8) had a matching frame: 3300 rpm back-EMF angle −4.6° against −5.0°, δ at 10500 rpm 4.9° against 4.6°.
  - Vq_FF sat 100 % on its clamp (unclamped 37 V against a 28 V limit); with λ 0.00849 it was 0 % clamped (22 V).
  - The 3.3 Hz wobble grew from 0.5 to 5.6 A rms in 6 s, at −94° and tracking gain 0.23.
  - Stop was commanded at 143.9 s. During the ramp-down Id_ref swung to −95 A, power hit 2.3 kW and Iq_ref dropped to 0. Bus went 47 → 80 V in about 150 ms, then BUS_VOLTAGE_OV_ERROR at 145.05 s.
  - This also answers the review's caveat: a hot (73 °C) default-λ hold wobbles, so λ is the cure, not temperature.
- **Encoder electronics look healthy (the TIM2 count-loss lead is retracted):**
  - No encoder fault flags, with `ENABLE_ENC_ERRORS` on.
  - **Count against the next boot's PWM reading:** three clean power-up pairs read +2 (`sweep` → `wow`), +1 (`9500RPM` → `10000_pi_changed`) and −21 counts (`10100_repeat` → `tf_happened`).
    - The firmware scales the PWM frame as 4096 steps. If the MT6816 frame is 4119 clocks, as the reviewer reads the datasheet (not confirmed by me), the error spans about 23 counts per revolution, which predicts about −19 for that third pair. That would also explain most of the 2.9° boot scatter.
    - No pair covers `massive`.
  - **Drift:** none at the same speed and heat that drifted in `massive`: −0.005°/s over 34 s at 10500 rpm and 64–68 °C, +0.007°/s over 6 s at 73–74 °C. Largest frame-to-frame speed step in the hold is 4 rpm.
  - **Repeatability:** back-EMF angle −5.0° and −4.6° across a power-cycle, at boot counts 2610 and 1688.
- **The alignment is what moved.**
  - Offset-free misalignment after the autotune is −98°, the same as the retries (−93 / −101°). The autotune re-zeroed the shift; it didn't undo it.
  - Every autotune so far has landed lower: 1.148 → 1.128 → 1.069 → 1.055 → 1.046 → 0.916 rad, a total of −13.3° mechanical, or −66° electrical. One-way movement fits a mechanical creep or slip, not measurement noise.
  - EEPROM, τ, boot position and count loss are ruled out. Nothing else found survives a power cycle.
  - The logs can't say which part moved: magnet on shaft, sensor board in housing, or rotor on shaft. A marked inspection is the only direct test.
- **Tried with no result:** rest positions against a cogging grid. The test has too few points to say anything.

### 1 Nov, 20:33: second logged 10500 rpm pass (`pegasus_20261101_203355`, recorded by the GUI logger)

- **Settings:** offset 0.916, τ 130, λ 0.00849 and speed PI 3.0/6.1 sent at 0 rpm, boot count 191, motor 58 → 66 °C. The firmware was not reflashed, so there is no 0x422/0x40A data.
- **10500 rpm held for about 26 s:** σ 1.15 rpm in steady windows (about 0.6 rpm of that is the 50 Hz 6/rev alias line), Id_ref 1–8 Hz 0.07 A, 3.3 Hz envelope 0.03 A (max 0.10 A), 0 % alternation, no clipping, no derate, no overrun. Bus 47.8–49.1 V.
- **Ramp-down at 600 rpm/s:** smooth. Id_ref unwinds from −71 A to 0 by about 4200 rpm, Iq_ref never hits its floor (minimum 0.6 A), power stays positive (no regeneration) and the bus only rises from 48 to 49.8 V as load falls. The log stops at 1406 rpm with no error, about 2.3 s short of zero.
- **"Cleared":** with `105000_working`, that's two logged passes on the bench workaround. The only gap is the last part of each ramp-down (5813 and 1406 rpm).
- **Frame:** back-EMF angle −1.9° at 3300 rpm (Vd −1.29 V). Boots since the autotune read −5.0, −4.6 and −1.9°, within the normal ±3° scatter. The difference from `105000_working` stays about 3–4° from 3300 to 10500 rpm, so it's a plain boot offset.
- **No drift:** −0.009°/s over 24 s at 10500 rpm, 61–64 °C.
- **Runaway margin:** the current vector is at 68.9° at the hold, so a further 21° electrical (4.2° mechanical) frame shift would take Iq_ref to its floor. In `massive` the trip came after a 20° shift. The τ residual (δ ≈ +8°) is part of that margin; raising τ now would cut it to about 13° (inferred).
- **Voltage headroom:** mean |V| is 0.847 of the circle, but peak modulation index reaches 1.00 with 0 % clipping, so the per-step ripple uses up nearly all the 15 % margin.
- **Feed-forward estimate (from logged Ld/Lq, λ 0.00849):**

  | Speed (rpm) | 5000 | 7000 | 9000 | 10000 | 10500 |
  |---|---|---|---|---|---|
  | Vq_PI (V) | +4.9 | +4.5 | +3.2 | +1.8 | +0.6 |
  | Vd_PI (V) | −0.8 | +1.9 | +5.2 | +6.6 | +7.1 |

  λ 0.00849 is right only near 10500 rpm. The Vd term looks like the τ residual inflating Iq_ref.
- **Why the original Vq feed-forward was wrong (new; strong lead).** Fitting Vq − Rs·Iq/g = ωe(λ + L·Id/g), with g = 1/√(1 + (ωe·RC)²) for the sense filter, on five logs across 1000–10500 rpm:

  | Log | Motor (°C) | No filter: rms / max error (V) | RC 132 µs: rms / max error (V) | λ (mWb) | L (µH) | Best RC (µs) |
  |---|---|---|---|---|---|---|
  | A | 48–55 | 0.37 / 1.61 | 0.23 / 0.40 | 10.80 | 73.6 | 110 |
  | new | 58–64 | 0.46 / 1.71 | 0.21 / 0.31 | 10.66 | 73.0 | 110 |
  | `105000_working` | 61–68 | 0.41 / 1.72 | 0.25 / 0.35 | 10.51 | 72.1 | 100 |
  | `massive` | 66–70 | 0.35 / 1.14 | 0.20 / 0.31 | 10.49 | 74.7 | 100 |
  | `power_cycled` | 70–73 | 0.35 / 1.15 | 0.24 / 0.34 | 10.30 | 72.3 | 100 |

  - The best-fit RC (100–110 µs) was not imposed, and it sits close to the 132 µs read from the schematic.
  - λ comes out near the datasheet back-EMF value and falls steadily as the motor heats, roughly −0.2 %/°C against the NTC.
  - L is about 73 µH, against 63.5 µH in the profile.
  - **Superseded.** This fit corrected amplitude only, not phase, so it pushed the missing phase into L. Even with phase included, Ld turned out not to be identifiable (63.5–77 µH, Deep review). The `k` feed-forward proposal, and the claim that Ld is about 15 % higher, are withdrawn.
- **GUI logger on real hardware:** 104,987 frames in 209.6 s, no malformed rows, all 26558 fast windows paired, 0 missed. Tx rows use the PC clock, so they can sit slightly out of time order; Parser2 handled that. The trace went to the repo root because that's the folder the GUI remembered.

### 1 Nov: independent review of every claim (`scratchpad/vet3/`)

| Claim | Verdict | Notes |
|---|---|---|
| λ 0.00849 removed the 3.3 Hz wobble | Confirmed | Id_ref 1–8 Hz is 0.07–0.09 A at 10200/10500 rpm, against 0.66–3.9 A in older logs.<br>The old logs had Vq_FF clamped in 86–100 % of frames above 7.5 krpm (unclamped 29.5–38 V against a 28 V limit); `massive` is never clamped (19–22.7 V).<br>A matched-boot log (K105, same back-EMF-angle curve) still wobbled, and trial_b with PI 3/6.1 wobbled, so neither the boot nor the PI is the cure.<br>Caveat: `massive` is the only λ 0.00849 log and the hottest. |
| In-run frame drift in `massive` | Confirmed, smaller | 20° by the exact Ld-free angle δ; the back-EMF angle says 24°.<br>Rotor-frame id/iq/vd/vq stayed constant to ≤1.5 %, and power stayed at 1497 → 1499 W.<br>Drift was −0.35°/s at 70–72 °C and −0.51°/s at 73–74 °C. |
| The drift caused the overspeed trip | Confirmed | Iq_ref hit 0 at 1016.13 s, with δ ≈ −16° and Id_ref −82 A, which gives about +22 A of hidden q current against about 17 A needed. |
| Persistent shift across power-ups | Confirmed; slip is the best fit, not proven | The offset-free misalignment X sits in three levels:<br>−33.6 ± 2.9° (29 Oct before the runaway boot);<br>−55.6 ± 2.9° (from the runaway boot through `massive`, seven boots over two days and three EEPROM offsets);<br>−93 / −101° (retries).<br>τ, EEPROM, count loss and boot-angle dependence are all ruled out. |
| Retry runaways from the frame error | Confirmed | Iq_ref reached 0 about 3–3.5 s after Id_ref went negative.<br>Retry 2 went 4455 → 10772 rpm in 2.1 s. |
| λ ≈ 10.9 mWb matches the datasheet | Weakened | It agrees within about ±1.5 %, but the temperature coefficient isn't resolved. |
| Boot scatter caused by the single PWM read | Weakened | Scatter is σ ≈ 2.9° once the offset is removed. **Later explained** as a deterministic decode error (Summary item 4). |
| Ld/Lq derating is small | Confirmed, plus more | In `massive` it was −1.6 % mean with −9 % transients.<br>The older logs derated harder: K105 sat at the −12 to −15 % cap, K102 at −7.5 %.<br>It is driven by the controller's own (wrong-frame) torque estimate. |
| Slow-frame Id/Iq scatter is a 2fe ripple; no PI retune needed | Weakened | The scatter is periodic and scales with speed, but the 2fe lines explain only part of it.<br>Fast frames are clean (0.42–0.57 A error) only in `massive`; with the clamped feed-forward, Id doesn't follow Id_ref at 1–8 Hz at all.<br>Re-check the PI after the feed-forward is fixed. |

- **Other findings:**
  - **6/rev alias:** confirmed, and it dominates run A's σ at 9900 rpm.
  - **τ residual:** the high-speed back-EMF baseline of about +9 to +11° is a τ residual. Extrapolation suggests τ ≈ 170 µs nulls it; re-check this after the angle is fixed.
  - **λ looks speed-dependent:** |E|/ωe is 8.4–8.8 mWb at high speed against 10.7 at low speed, so 0.00849 is an effective high-speed value. The Theory check explains this: it comes from the current reading, not from λ.
  - **UV trips:** the trial_b and U3 under-voltage trips look like supply foldback at only about 1.5 kW.
  - **Post-trip bus:** reaches 89–102 V against the 80 V over-voltage setting.

### Datasheet check (TW-7 datasheet, updated)

- **Matches the profile:**
  - 5 pole pairs;
  - Rs 6.159 mΩ;
  - Ld/Lq 0.0635/0.0879 mH at rated;
  - λ 11.21 mWb;
  - 200.6 Arms peak current and 25 Nm peak torque;
  - ABZ+PWM feedback, which matches the boot scheme.
- **The back-EMF figure gives a lower λ than the datasheet's 11.21 mWb.**
  - The datasheet gives 6.98 Vrms/krpm (line-to-line) at 40 °C, which is λ = 6.98·√2/√3 / (2π·5·1000/60) = 10.9 mWb (11.0 mWb at 30 °C).
  - The two points give a −0.13 %/°C coefficient, so λ is about 10.4 mWb at 75 °C.
  - 0.0085 is a test value that pulls the feed-forward out of its clamp, not the real λ.
- **Max speed is 10000 rpm.** 10500 is 5 % above it, so the encoder's own speed or frequency rating needs checking.
- **Ld/Lq at peak:** 0.04791/0.06803 mH, a 25 %/23 % derating against the profile's 15 %. This only matters at high torque.
- **Inertia:** the datasheet gives 1.93e-3 kg·m², against our estimate of 2.6–3.0e-3. Trust the datasheet.
- **Insulation:** class F (155 °C), but `MOTOR_TEMP_OT_LIMIT` is 150 °C, which leaves little margin unless the NTC sits at the hot spot.

---

## 3. Finding 1: the angle reference moves at every power-up (Proven)

**Revised 1 Nov.** The frame does differ between power-ups, but the single PWM reading only accounts for about 3° of scatter. The larger differences below were EEPROM re-tunes and persistent alignment shifts. See the Summary and the 1 Nov review. This section is kept as the record.

### What the logs show

To compare boots, use the **back-EMF angle**: the angle of the voltage left over after removing the modelled resistive and inductive drops from the measured Vd/Vq:

```
epsE = atan2(Vd − Rs·Id + ωe·Lq·Iq,  Vq − Rs·Iq − ωe·Ld·Id)
```

It does not depend on λ, so motor temperature doesn't move it. With a perfect frame and a perfect model it would be 0°. For a given speed and firmware, how it changes from one boot to the next is the boot's angle error.

| Log (τ = 130) | Calibration | Vd at ~3300 rpm | epsE at ~3300 rpm | epsE at 10000 rpm | Outcome |
|---|---|---|---|---|---|
| `10500` | A | — | — | 11.0° | 10200 clean. ID_IQ_OC trip 6.5 s into 10500 |
| `10200` | A | +1.5 V ¹ | +5.8° ¹ | 18.5° | 28 s at 10200, wobble growing |
| `10100_repeat` | A | 0.0 V | +3.7° | 15.3° | Over-voltage trip at 10100 |
| `10100_repeat_tf_happened` | A | **−8.1 V** | **−22°** | — | **Runaway at a 6000 rpm reference** |
| `10500_unstable` | B | −2.6 V ² | −5.6° | 5.5° | 3.5 s at 10500, manual ramp-down |
| `10500_unstable_2` | B | — | — | 3.1° | Over-voltage trip ramping to 10500 |
| `10500_unstable_3` | B | −1.4 V | −2.0° | 8.9° | Under-voltage trip on a ramp |

- **Calibrations A and B:** the angle autotune was re-run once between `tf_happened` and `10500_unstable`. The EEPROM offsets went from 1.128 / 1.12 / 1.12 to 1.069 / 1.08 / 1.05 (avg / CW / CCW), so compare boots only within the same calibration.
- **Within calibration A** (three boots at 10000 rpm), epsE spans 7.5°. Within B it spans 5.8°. Pairwise differences are 2.4–7.5°, median about 4°.
- **Measurement windows and accuracy:**
  - The ~3300 rpm column comes from the first ramp, where Id ≈ 0. `—` means the log has no clean window there.
  - ¹ This `10200` value is from the ramp-down (Id ≈ +1 A).
  - Values marked ² and the `10100_repeat` entry come from slow frames that never changed value (whole volts), so they are good to about ±1.6°. The other 3300 rpm values are good to about ±1°, and the 10000 rpm values to about ±0.5°.

### Why this is the controller's angle, not a change in the motor

1. **Within a boot the angle doesn't move.** epsE holds to about ±0.35° through 30–35 s holds, so the encoder isn't slipping counts.
2. **Voltage and current rotate together.** At 10000 rpm the angle between the voltage and current vectors is 55.0 ± 0.4° in every τ = 130 boot, while the voltage angle alone spans 12.6°.
3. **Power ignores the angle.** Electrical power at 10000 rpm (1.52–1.71 kW) follows motor temperature (r = −0.98) and barely follows the boot angle (1–3 W per degree). The motor was doing the same thing in every boot.
4. **Reported Iq is a coordinate artifact.** It shifts 1.1 A per degree of boot angle, close to the 1.3 A/° that rotating the frame against about 75 A of field-weakening current predicts.
5. **The alternatives don't fit:**
   - a current-sensor offset would show as ripple at the electrical frequency, not a mean rotation;
   - a temperature change would not rotate V and I together;
   - τ was 130 µs in every boot (0x406 echo).

### The runaway (`10100_repeat_tf_happened`)

That boot was about 22° off from 130 rpm upward. At 3300 rpm Vd read −8.1 V, where about −0.65 V is normal.

- **The hidden torque:** with the frame rotated that far, about 20 A of what the controller treated as field-weakening current (Id) was really torque current.
- **What happened:** by about 5150 rpm the speed loop's torque command was at 0, and that loop can't brake (`SPEED_PID_OUT_LOWL` = 0). The motor kept accelerating to 8330 rpm until the log ended.
- **Size check:** the acceleration plus normal drag needs a torque equivalent to about 15–29 A of Iq. That matches Id × sin 22° ≈ 18–26 A.

The τ = 250 µs runaway in `sweep` pushed in the same direction from a different cause: τ over-correcting the current reading.

### Cause in the firmware

- At boot, `set_Initial_angle()` ([Helper_Functions.c](Core/Src/Helper_Functions.c), called at [main.c:181/203](Core/Src/main.c#L181)) turns the **last single** TIM5 capture of the encoder's PWM duty into an absolute angle. TIM5 is then switched off.
- About 2 s later, at the end of the current-sensor calibration (4096 samples at about 2 kHz), that angle is written into the incremental counter TIM2 ([isr_handlers.c:82](Core/Src/isr_handlers.c#L82)). That overwrites whatever TIM2 counted in the meantime.
- The Z index pulse only stores `d.count_at_z` ([main.c:632-655](Core/Src/main.c#L632-L655)). It feeds the Z-fault check and is not sent on CAN. Nothing ever uses it to correct the angle.
- So any error in that one PWM reading stays for the whole power cycle, multiplied by the 5 pole pairs:
  - one encoder count is 0.44° electrical;
  - the 25° outlier is about 57 counts.
- Rotor creep during the 2 s window is ruled out for the outlier boot: the rotor was stopped and untouched. That leaves the PWM reading itself as the prime suspect. Sending `d.count_at_z` over CAN would measure each boot's error directly, because Z sits at a fixed point on the rotor.

### What it means

- **The angle autotune can't fix this.** It runs on a boot whose counter was loaded from the same single PWM reading, so it inherits that boot's error. (Revised: that inherited error is about ±5° from the decode bug. The autotune does re-zero a real alignment shift.)
- **Cross-boot comparisons are unreliable.** At 10000 rpm 1° is 3.33 µs of τ. That's why the earlier "130 beats 150 beats 170" calls are withdrawn.
- **Vehicles:** the same start-up path is compiled for every motor profile. A bad boot on a vehicle in field weakening could produce torque nobody commanded. **This needs raising with the vehicle firmware owners.**

---

## 4. Finding 2: above ~7500 rpm the current loop cannot follow at a few hertz (Strong lead)

### Measured

In the five boots with usable 200 Hz data, measured Id follows Id_ref with a gain of only **0.21–0.32** and a lag of **83–95°** in the 2.5–4.5 Hz band above 8500 rpm. That's the band where the 3.3 Hz wobble lives.

- **It starts lower down:** `9500RPM` shows the same at 7500 rpm (0.27 at −82°, coherence 0.76).
- **Below 7500 rpm** Id_ref moves too little (under 0.02 A rms) to measure.
- **It doesn't depend on the wobble's size:** it holds from 0.2 A to 14 A peak-to-peak.
- **Voltage headroom doesn't explain it:**
  - during holds the 5 ms voltage means peak at 24.3–25.3 V against a 28.1–28.3 V limit;
  - the 10 kHz demand peaks reach a modulation index of 0.97–0.99 at 10000 rpm (1.02 at 10500), so headroom is thin but not clipping;
  - and the tracking loss is the same at every wobble size.

### Why: the Vq feed-forward is pinned at its clamp

The current regulators add a feed-forward to each PI output ([FOC_Loop.c:491-569](Core/Src/FOC_Loop.c#L491-L569)):

```
Vd_FF = −ωe·Lq·Iq_ref
Vq_FF =  ωe·(Ld·Id_ref + λ)      clamped on its own to ±Vbus/√3, before the PI is added
```

- **The inputs:** Ld and λ come from the reference estimator; λ is 11.21 mWb and Ld is derated with load. The bus is the 10 ms filtered Vdc, capped at 51 V.
- **The result:** the Vq feed-forward comes out well above the real Vq, and from about 7000 rpm it sits at the clamp (`10500_unstable_3`):

| Speed (rpm) | Vq_FF unclamped | Clamp (Vbus/√3) | Actual Vq |
|---|---|---|---|
| 3000 | 17.5 V | 28.6 V | 16.9 V |
| 6000 | 27.8 V | 28.6 V | 24.0 V |
| 7000 | 29.3 V | 28.6 V | 23.7 V |
| 9000 | 33.5 V | 28.5 V | 23.8 V |
| 10000 | 36.2 V | 28.4 V | 23.0 V |

Once clamped, the feed-forward no longer changes when Id_ref or the speed changes. At high speed, Id can only be moved through Vq, and the d-axis PI drives Vd, so the d-axis loses its fast path. Why the model runs about 13 V high at 10000 rpm isn't settled; likely contributors:
- λ of 11.21 mWb against about 10.4–10.8 mWb measured;
- the sense filter reading Id about 15 % low at that speed;
- the Ld derating.

### Model

A discrete model of the 10 kHz current loop was built with the firmware's gains, feed-forward and anti-windup, and the logged Ld/Lq. It reproduces the measurement:

| 3.3 Hz, Id_ref → Id | Gain | Phase |
|---|---|---|
| Measured, 8500–10500 rpm | 0.21–0.32 | −83° to −95° |
| Model, 10000 rpm, Vq feed-forward clamped | 0.27 | −80° |
| Model, 7000 rpm, clamped | 0.46 | −68° |
| Model, 10000 rpm, feed-forward working | 0.99 ideal; 0.77–0.91 allowing for the sense filter's gain, residual angle errors and Ld mismatch | ≈ 0° |

The root weakness is the current loop's own slow mode. Its integral gains are small (Ki/Kp ≈ 20–27 rad/s), so at speed the cross-coupling leaves a slow, lightly damped pole pair. Figures computed with fully derated Ld/Lq of 54/75 µH:

| Speed | Slow current-loop mode (natural frequency, damping) |
|---|---|
| 3000 rpm | 2.9 Hz, ζ 0.80 |
| 7000 rpm | 1.8 Hz, ζ 0.50 |
| 10000 rpm | 1.3 Hz, ζ 0.37 |

A working feed-forward lets reference and speed changes bypass that mode. A clamped one exposes it.

**Raising the current-loop Ki isn't the fix.**
- ×3 moves the mode onto the wobble frequency (gain 1.58 at 3.3 Hz).
- ×5 moves it higher but leaves ζ at 0.37.

### How this would make the 3.3 Hz wobble (inferred)

The speed loop and the field-weakening integrator both act through this d-axis current loop:
- the speed loop has its own mode at about 9 Hz;
- the field-weakening loop gain FW_CL_KI·ωe·Ld is about 64 rad/s, or 10 Hz, at 10000 rpm.

With the feed-forward clamped, the current loop answers at about 1 Hz. An inner loop slower than the loops it serves is a standard way to get a low-frequency oscillation. The 3.3 Hz wobble fits that picture, and it appears only in deep field weakening, where the feed-forward is clamped. Its exact frequency hasn't been derived.

One run with the speed PI halved (`10000_pi_changed`) showed the wobble growing faster. That's suggestive only, because the 4/8 comparison came from a different boot.

---

## 5. Finding 3: how the trips happen

### The four trips in the τ = 130 batch (Measured)

| Log | Trip |
|---|---|
| `10500` | ID_IQ_OC, with the bus at 66–81 V in the trip frames |
| `10100_repeat` | Over-voltage (80 V) |
| `10500_unstable_2` | Over-voltage (80 V) |
| `10500_unstable_3` | Under-voltage (25 V) |

Earlier in the day, `wow` tripped on ID_IQ_OC and `10000_pi_changed` and `9500RPM` on PHASE_CURR. The bus checks trip on a single 1 ms sample (`BUS_DC_*_COUNT` = 1). A 10 Hz frame can still show 47–48 V at an under-voltage trip, because it was sampled just before the collapse.

### Common sequence

1. **Sag (Measured).** A wobble peak pulls about 2.5 kW, roughly 50 A, from the link, close to the supply's ~60 A limit. The bus sags to 41–44 V.
2. **Swing up while still motoring (Measured; cause inferred).** Within about 0.1 s the link rises to 57–72 V while computed power is still +0.5 to +1.6 kW and Id is still −65 to −80 A.
   - This was seen in `10500_unstable_2`, `10500` and `10500_unstable_3`; the last one rode one such swing (153.6 s) without tripping.
   - The likeliest source is the supply overshooting as it comes out of current limit.
3. **The firmware's bus model is wrong exactly then (code fact).** The controller's bus voltage is the 10 ms filtered Vdc, capped at 51 V ([Sensor_Disconnection_Checks.c:223](Core/Src/Sensor_Disconnection_Checks.c#L223)).
   - At 57–68 V real, the inverter applies 12–33 % more voltage than the controller computes.
   - During a fast sag, the filter lags behind.
4. **Then whichever limit comes first:**
   - **Under-voltage (`10500_unstable_3`, 154.2 s):** field weakening deepens (Id_ref −87 → −107 A) and the motor surges 216 rpm in 70 ms, drawing 2.9 kW. The link falls from 49 to 28 V in about 55 ms; computed current rises to 67–102 A as the link capacitors discharge.
   - **Over-voltage (`10500_unstable_2`, `10100_repeat`):** current control is lost (modulation index 1.15, 21 % of steps clipped). Regeneration (Vd > 0, Iq < 0) appears only once the link is already at 66–72 V, and it carries the link to 80 V.
   - **Id/Iq over-current (`10500`):** with the bus at 66–81 V.
5. **After any trip** the free rotor charges the link to 86–92 V through the diodes (uncontrolled generation). The supply has to tolerate that.

### The bench supply (Measured)

- **Idle:** the board reads 0.5–0.8 V below the setpoint. Either the Vdc reading is low or the supply delivers less than it shows; a multimeter at the board will tell.
- **Hold draw:** 31–35 A at 10000 rpm.
- **Effective resistance:** roughly 30–90 mΩ during holds and about 130 mΩ at 10500 rpm (estimated from computed power).

---

## 6. Retracted or corrected

| Earlier claim | Now |
|---|---|
| τ 130 beats 150, and 170 is worse | **Retracted.** Each comparison was across boots, and one crossed the recalibration. |
| Parser2's τ_total drift (≈205 → 187 µs) comes from motor heating | **Corrected:** partly boot angle. Those absolute values are model- and boot-dependent. |
| The under-/over-voltage trips might be a fault-reporting bug | **Retracted.** The trips are real. |
| The trips at 10000 rpm and above are all bus-voltage crossings | **Corrected:** one of the four was Id/Iq over-current. |
| Over-voltage happens because the motor dumps energy after control is lost | **Corrected:** the bus first swings up while the drive is still drawing power. Regeneration only adds the last part (§5). |
| At 10000 rpm the drive is out of voltage, so the current can't follow | **Retracted** for holds (§4). Headroom is thin, and it does run out in the final moments of the trips. |
| A lower boot angle means a worse 10000 rpm hold | **Not supported** (Spearman p 0.16–0.43, n = 6–9). Only the extreme case, the runaway, is shown. |
| Halving the speed PI made things worse | **Downgraded to suggestive:** the comparison is across boots. |
| A ~1 Hz mode in `10500_unstable_2` | **Retracted:** an artifact of unpaired fast frames. It's the same 3.3 Hz mode. |
| The τ shift between boot groups matches 20 µs × Δω | **Retracted:** coincidental, n = 4. |
| Rotor creep during boot caused the outlier | **Ruled out** for that boot (rotor stopped and untouched). |
| Re-running the angle autotune fixes the angle | **Corrected:** the autotune inherits about 3° of boot error, but it does re-zero a real alignment shift. That's how 10500 rpm came back on 1 Nov. |
| The Vq feed-forward needs a `k` factor; Ld is about 73–77 µH | **Withdrawn:** amplitude was corrected without phase. |
| Datasheet Ld fits once the sense chain is modelled | **Withdrawn:** with the stale phase-V sample included, Ld isn't identifiable (63.5–77 µH, about 74 µH on the code-based split). |
| The sense chain has about 73 µs of extra pure delay | **Corrected:** about 46 µs of it is the stale phase-V sample; about 26 µs remains. |
| The 2fe ripple comes from mismatched sense channels | **Corrected:** it's the stale phase-V ADC sample (firmware bug). |
| Put λ 0.0085 and speed PI 3.0/6.1 in the profile | **Withdrawn:** that patches a symptom. Remove the separate feed-forward clamp instead (simulation: stable at any λ). |
| Lowering or scheduling the FW gain would calm the wobble | **Not supported:** in simulation neither fixed it, and both made it worse. |
| Fix the ADC to remove the residual lag | **Not on its own:** in simulation it shrinks the runaway margin from about 20° to about 6°. It needs angle accuracy and a negative Iq floor first. |
| Clamp feed-forward + PI together | **Dropped:** not needed. The clamp is only a problem when the feed-forward is wrong. |
| Free-spin loss is 1.5 kW at 10000 rpm | **Corrected:** that used the rotated current. About 1 kW, pending a supply ammeter reading. |
| Motor current at 10500 rpm is 73 A peak, 52 Arms | **Corrected:** that's the reading. Real current is about 95 A peak, 67 Arms. |
| The encoder may be losing counts through the TIM2 input filter | **Retracted:** no drift at the same speed and heat, rest counts match the next boot's PWM reading, and the shift survives power cycles. |
| The 6–8° boot scatter comes from the single PWM reading | **Corrected:** about 3°. The rest was EEPROM re-tunes and level shifts. The 3° itself fits the PWM decode frame-length error. |
| Re-sync the angle on the Z pulse | **Dropped:** Z comes from the same magnet, so it can't correct a slip, and boot scatter is small. |

Still standing:
- **τ compensation itself.** The within-boot sweep at 6000 rpm validates it. 9000 rpm also went from σ 17 rpm to about 1 rpm; that comparison is across boots, but the change is far larger than any boot effect seen.
- **τ = 250 µs runs away**, shown within one boot.

---

## 7. What to do next

Updated 1 Nov after the deep review. The firmware order is the plan table in the Deep review, phases 0–6. Bench rules until then:

1. **Every power-up: boot check.** Hold 3300 rpm with fast telemetry on, and read Vd.
   - With offset 0.916 it should read about −1.3 to −2.3 V.
   - **Below −4 V, stop.** Don't go above 3000 rpm. Find out why it moved before re-running the autotune. The two retries that ran away read −12.5 and −14.6 V.
2. **Every high-speed run: watch Iq.** At 10500 rpm it sits at 23–28 A. If it sinks steadily below about 12 A, ramp down at once.
3. **Use the bench workaround until phase 2a:** λ 0.0085 and speed PI 3.0/6.1 after boot, τ 130. Don't raise τ.
4. **Bench checks** (phase 0):
   - **Paint marks** across the magnet and shaft, and across the sensor board and housing. Check them after each high-speed run.
   - **Supply ammeter** at the 10500 rpm hold: about 21 A means about 1 kW; about 33 A means about 1.6 kW.
   - **Clamp probe** on one phase at 10500 rpm: 60–67 Arms confirms the corrected current; about 54 Arms (the logged value) refutes it.
   - **Scope a GPIO** toggled in the ADC callback and at the CCR write, to confirm the sample-to-PWM timing.
5. **Then the firmware phases, one change per flash, each needing approval.**
   - Start with the ports in "Revised phase 1": the MOE and reset guard, the boot seed bundle and the offset one-liner, then the interrupt load. Flash the 0x422/0x40A telemetry with them.
   - Then 2a (no separate feed-forward clamp) and 2b (negative Iq floor with a DC-link limit).
6. **Before anything merges:** the change sheet's "remove or fix" list still applies. The vehicle firmware owners should hear about:
   - the stale-V ADC bug;
   - the PWM decode;
   - the alignment shifts;
   - the separate feed-forward clamp;
   - speed mode having no negative torque.
   All of these sit in shared code.

## 8. Open questions

- **Which part moved?** Magnet on the shaft, sensor board in the housing, or rotor on the shaft. Only inspection can say.
- ~~Is the MT6816 PWM frame 4119 clocks?~~ Yes: `unified_image` `d56e7e09` decodes it and measured it against Z.
- **Is VOS3 with 100 MHz APB clocks within spec?** Check the STM32H743 datasheet's maximum-clock table.
- **What is the ADC clock rev (REV_ID)?** It sets whether V is 85 or 92 µs stale.
- **How is the measurement lag split** between the RC, the residual delay and Ld? An inductance measurement, or a log taken after the ADC fix, would separate them.
- **What is the voltage-side delay?** The code timeline gives 116 µs from angle read to pulse centre; the fits and the old calibration give 80–85 µs.
- **Is the real power at 10500 rpm about 1 kW?** The supply ammeter answers this.
- **What does the closed autotune library target?** The reviewer read from its header that it aims for Vd of −1 to −2 V at about 2000 rpm, which would bias the offset by a few degrees. Not verified.
- **How much CPU do the useless interrupts take?** Measure TIM17 jitter, or count idle-loop iterations.
- **Does the supply overshoot when it leaves current limit?** A scope on the link during a sag would show it.
- **Smaller sense filter or 20 kHz loop:** neither has been simulated yet.

---

## Method notes

- **Data:** slow frames are 10 Hz and fast frames are 200 Hz (5 ms means). The firmware floors the slow-frame Vd/Vq and Id/Iq, so their means sit about 0.5 below the fast-frame means; slow means were corrected by +0.5.
- **epsE model values:** Rs 6.16 mΩ, Ld 63.5 µH, Lq 87.9 µH (nominal). The firmware's load-derated Ld/Lq shift the absolute epsE at 10000 rpm by under 1°, and not the boot-to-boot differences.
- **Tracking gain:** cross-spectrum Id/Id_ref from 200 Hz data on each hold (Welch, 2.56 s segments). Results with coherence under about 0.6 were not used.
- **Model:** the current loop as in `FOC_Loop.c`, using effective gains Kp 0.136 / 0.12 and Ki 2.8 / 3.2, the integrator scaled by the 100 µs step, a salient plant with the logged Ld/Lq, and a one-sample PWM delay (variants made no difference). Small-signal only.
- **Theory fit:** speed bins of 200 rpm from fast frames, up-ramps and holds only. Least squares on both axes, with a per-log λ and frame offset. Inverter drop modelled as a constant voltage in phase with the current.
- **Nonlinear simulation** (`n3/sim*.py`): 10 kHz, with the exact discrete dq plant, the firmware PI with back-calculation, the separate feed-forward clamp, the per-axis and circle limits, the FW integrator, the 1 kHz speed PI with its 30 ms filter and 0 A floor, a free rotor with 1 kW of loss, and the measurement error modelled as a −18° rotation and ×0.9. The thresholds depend on the starting state; runaway margins were taken from settled starts.
- **Stale-V check** (`n3/stale*.py`): slow-frame Id/Iq paired with the encoder count from the adjacent 0x402 frame, fitted against e^(−j2θ).
- **Closed-loop model:** 10 kHz discrete dq model with the sense filter as a complex pole in the rotating frame, 85 % of a period output delay, and the effective gains above. Linear, with no voltage limit and no outer loops.
- **Scripts:** these live in the analysis session's scratchpad and are not in the repo yet.
