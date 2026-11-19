# TW-7 bench bring-up: change sheet

Every firmware change behind the TW-7 (HMC) 48 V bench build, why it was made, and whether the evidence supports it. Vetted 2026-10-29 against the code and the bench logs (`HMC_Logs/Bringup_gen4`, `27_10`, `28_10`), with verdicts updated 2026-11-01 where the 1 Nov logs changed them. Terms follow [CONTEXT.md](CONTEXT.md). The reasoning is in [TW7_BRINGUP_FINDINGS.md](TW7_BRINGUP_FINDINGS.md).

- **Range:** `ee96437a` → `8a62c3d7` (tag `tw7-baseline-1028`), plus the uncommitted working tree.
- **Build:** `MOTOR_PROFILE_HIRANGE_TR` + `HIRANGE_TR`. Every flag below marked "TR" is on in this build.
- **Bench:** free rotor, no load, 48–50 V supply that cannot sink current.

**Verdicts**

| | Meaning |
|---|---|
| **Validated** | A log shows the effect, larger than run-to-run spread |
| **Justified** | The control or physics argument holds; no isolating test yet |
| **Inert** | Built, but never engaged in any log |
| **Unproven** | A value picked from a single run |
| **Questionable** | The reason given is refuted, superseded, or derived for another motor |
| **Risk** | A defect found while vetting |

Run-to-run spread for one unchanged config is large: speed σ was 5.9 vs 14.8 rpm at 8000 rpm. Only one change clears that bar, so far.

---

## 1. Timeline

| Commit | Date | What it did for this build |
|---|---|---|
| `ee96437a` | 07-23 | Created the HIRANGE_TR slot for a 76.8 V HiRange TR **vehicle** (XR motor). Its vehicle constants (72 Nm envelope, 2000/5655 W power band) are still live in the TW-7 build. |
| `022338e2` | 07-30 | Id/Iq clamp and current OC fault sized at 1.2 × MOTOR_PEAK_AC (±340 A on TW-7). |
| `b1937519` | 10-05 | XR-motor dyno at 66 V. Added the **closed-loop FW** hand-edit, bench CAN plumbing, speed-only mode, atan2 speed sensing, and the unmarked V_s unit delay. |
| `8de48efa` | 10-12 | XR-motor dyno at 96 V. Changed the divider table, `MAX_PHASE_CURRENT` = IQ_UP_LIMIT, and added `BENCH_FORCE_PWM_C`. |
| `2d63cb53` | 10-26 | **TW-7 repurposing:** 5 pole pairs, λ 11.21 mWb, Ld/Lq 63.5/87.9 µH, 48 V, OV/UV 80/25, new Id/Iq gains. Closed-loop FW was off in this commit. |
| `932afe48` | 10-27 | Closed-loop FW back on, FW floor K 1.1, **85 µs output-delay compensation**, overspeed trip, new divider selected, 10 Hz telemetry. |
| `8a62c3d7` | 10-28 | Vdc-LPF bus model capped at OP_VOLTAGE, **circular \|Vdq\| limit**, **current-sense lag compensation** (τ booting at 0), derate 15/15, VREF 85, OP_VOLTAGE 51, 10500 / 10800 rpm. |
| working tree | 10-29 | 200 Hz bench telemetry, CAN TX queue fixes, Parser2, GUI tuning panel. |

**The closed-loop FW regulator and every `FW_CL_*` constant were designed on the XR motor** (λ 26.3 mWb, Ld ≈ 130 µH) and reused unchanged on the TW-7. The same applies to the speed PI and to the gain-switch thresholds.

---

## 2. Hand-edits in Simulink-generated code

All of these are lost when the model is regenerated. Each one has to be ported into the model or re-applied.

| # | Where | What it does | Why | Gate | Verdict |
|---|---|---|---|---|---|
| H1 | `Current_Ref_Estimator.c` FW block | **Closed-loop FW:** dId += KI·(VREF·Vbus/√3 − \|V\|)·Ts, with dId ≤ 0 and Iq trimmed to the Is_max circle | The generated FW chart integrates a V_s built from the pre-FW references, so it could never act | `FW_CLOSED_LOOP` (TR) | **Validated.** Iq tracking error went from −49…−74 A to about 0, and peak modulation index from 1.05–1.08 to 0.92–0.94. |
| H2 | `FOC_Loop.c` inverse Park | **Output-delay compensation:** the voltage is applied at θ + ωe·85 µs | Below base speed, Vd ≈ −Vq·tan(ωe·delay) | `FOC_VOUT_DELAY_COMP` (TR) | **Validated** below base speed: the voltage-angle residual went from +6…+11° to −3…+2°. Its share of the high-speed gain isn't isolated, because it shipped with H1. **1 Nov:** a fit of the textbook equations on five logs puts the error at −2 to −16 µs, so 85 µs stands. |
| H3 | `FOC_Loop.c` Park | **Current-sense lag compensation:** the Park transform uses θ − ωe·τ, with τ set live on 0x115 | The measured dq current is rotated by ωe·~210 µs (57° at 8800 rpm: 64 A measured Iq vs ~5.5 A real) | `FOC_ISENSE_DELAY_COMP` (TR) | **Validated at 6000 rpm** (29_10 `sweep.trc`). τ = 0 / 150 / 210 µs gave Iq 33.5 / 13.7 / 5.1 A with identical Vd/Vq, and every hold implies a lag of 205–208 µs. **τ = 250 µs ran away** (5850 → 7400 rpm with Iq_ref at 0) and tripped, because overshooting τ couples FW into positive torque. With τ on, 9000 rpm went from failing to σ ≈ 1 rpm. Later runs used 130 µs up to 10500 rpm. τ survives a trip; only a reset or power cycle returns it to 0. **The best τ is not yet known:** the 130 / 150 / 170 µs comparisons were made across power-ups, and the angle reference moves at every power-up by an amount worth about ±10–15 µs of τ at 10000 rpm. See [TW7_BRINGUP_FINDINGS.md](TW7_BRINGUP_FINDINGS.md) §3. **1 Nov:** the real lag is an RC filter (132 µs), a phase-V ADC sample that is one PWM period stale (a firmware bug, about 46 µs on average) and about 26 µs of other delay. That's about 186–202 µs depending on speed, and τ 130 under-corrects by about 18° at 10500 rpm. **Leave it at 130 for now:** the under-correction keeps Iq_ref away from its 0 A floor, about 21° of margin against about 4–5° at the correct value. τ can't correct the 19 % amplitude loss. |
| H4 | `FOC_Loop.c` after the PI sums | **Circular \|Vdq\| limit** with d-axis priority (\|Vd\| ≤ 0.9r), and back-calculation anti-windup against the applied voltage minus the FF | The per-axis clamps let \|V\| reach √2·r, and the stock anti-windup misses the post-FF clamp | `FOC_VDQ_CIRCLE_LIMIT` (TR) | **Inert:** clip ratio 0–2 %, and 0 % in both 10500 rpm passes, though peak modulation reaches 1.00. Logic verified, host test passes. Keep it as a safety net, but don't credit it with any improvement. |
| H5 | `Current_Ref_Estimator.c` FW block | FW feedback reads the **demanded** \|Vdq\| rather than the applied one | Otherwise the limit would cap the FW error | `FOC_VDQ_CIRCLE_LIMIT` | **Inert**, because demanded ≈ applied while nothing clips. |
| H6 | `FOC_Loop.c` ~L335 | Gain-set switch: the high set needs ≥ 400 rpm **and** measured torque ≥ 10 Nm; otherwise every gain is ×0.8 | Restores a switch the generator emitted as `10 <= 0` | on | **Justified, but inert on TW-7.** The free rotor never reaches 10 Nm, so the **Id, Iq and speed PIs always run at 0.8×** the profile values. Compare gains using the effective values. |
| H7 | `Current_Ref_Estimator.c` init | Id/Iq clamp and measured-current OC = 1.2 × MOTOR_PEAK_AC | Per-motor sizing | on | **Justified.** The comment still describes 384 Arms. |
| H8 | `Current_Ref_Estimator.c` ~L517 | V_s computed from the **previous** step's Vd/Vq | Not stated (commit has no message) | **ungated, unmarked** | **Questionable.** Telemetry only on TR, but it silently changes XR/HiLoad. Remove it, or gate and mark it. |
| H9 | `FOC_Loop.c` integrators, init | Back-calc gain from `can_d.*.Back_Kaw`, sample time `d.Op_ts`, clamps ±Vmax_SVM / ±MOTOR_PEAK_AC; Iq Kd seeded from the **Id** struct | Present since the model import | on, **unmarked** | **Justified** as behaviour. It's a **process risk**: it looks generated, but regeneration drops it. Iq Kd from Id is harmless only because both are 1e-5. |
| H10 | `FOC_Loop.h` | `Vd_next` output added | — | ungated | **Inert:** never read. Remove. |
| — | earlier hand-edits | Anti-windup fix `b2aff907` and FW feedforward `5236a652` | — | reverted in `16f438be` | Not active. Angle advance and the salient torque inverse are compiled **off** for TR. |

---

## 3. Application code and configuration

### Motor model and current loops

| Change | Why | Verdict |
|---|---|---|
| POLEPAIRS 5 | Datasheet | **Validated:** autotune and post-trip back-EMF agree within ~3 %. |
| LAMBDA 0.01121 Wb | Datasheet | **Questionable:** measured 10.0–10.8 mWb, falling with temperature, and the datasheet back-EMF gives 10.9 at 40 °C. Set to about 0.0106 once the feed-forward no longer depends on it. **Bench workaround:** λ 0.0085 with speed PI 3.0/6.1 over CAN passed 10500 rpm twice. It isn't the real λ; it pulls Vq_FF off its clamp. |
| Ld/Lq 63.5/87.9 µH | Datasheet rated point | **Supported (1 Nov):** with the sense chain modelled, the datasheet values fit both axes to 0.16 V rms, and a free Ld lands at 66–68 µH. |
| Ld/Lq derate 25/23 → 15/15 | "Smoothed high speed", from one run | **Questionable / Risk.** The derate is scheduled on measured torque × speed, between the TR **vehicle's** 2000–5655 W band. On the free rotor, the rotated current frame produces ~5 Nm of fictitious torque, so the derate bottoms out and feeds the degraded L into the FF and the FW floor. **Use 0/0 on the bench.** |
| Rs 6.16 mΩ (12.32/2) | Datasheet | **Inert** in this build (telemetry only). It is possibly halved twice. |
| Id PI 0.17/3.5, Iq PI 0.15/4.0 (run at ×0.8) | Bench tuning | **Unproven, bracketed:** 0.05/0.12 was unstable and 0.20/0.20 sat within spread. |

### Field weakening

| Change | Why | Verdict |
|---|---|---|
| FW_CL_KI 200 | Designed for ~20 rad/s at 2500 rpm **on XR Ld** | **Questionable:** on TW-7 at 9000 rpm it gives ~50–60 rad/s. That's on top of the speed-loop crossover, and it may be what drives the 3.8 Hz alternation. A/B test 100 vs 200. |
| FW_CL_VREF_PCT 90 → 85 | "Removed alternation at 8000–8500" | **Questionable:** the logs refute it. There was no alternation at 90, and it came back at 85. |
| FW floor K 1.7 → 1.1 (−1.1·λ/Ld) | 1.7 pushed past zero flux | **Justified, inert:** holds use −36…−54 A against a floor of about −194 A. |
| FW Iq ceiling (Id error > 40 A) | XR dyno cliff | **Unproven:** not in telemetry. It probably engaged only during collapses. |

### Voltage limit and bus model

| Change | Why | Verdict |
|---|---|---|
| FOC bus = 10 ms LPF of the measured Vdc, capped at OP_VOLTAGE | ±8 % voltage error from normalising to a constant 48 V. Uncapped, a regen spike released FW (`first.trc`, 90 V). | **Justified.** The cap is backed by the `first.trc` failure. Its effect on the oscillations isn't shown, because it was tested together with a supply change. The baseline commit message overstates this. |
| OP_VOLTAGE 51 | Single run | **Questionable:** it sits above the 45–49 V supply, which breaks the cap's own "at or below supply" rule. Raise it together with the supply (54–58 V plan). |
| Divider: new table selected, scale 0.00374597 | Idle reads 49 V on the 48 V supply, and post-trip back-EMF agrees | **Validated for this board. Risk:** `2d63cb53`/`8de48efa` also rewrote the *old* entry (0.00206 → 0.002675) and the new one. Existing XR/HiLoad boards would read the link 30–40 % high. Restore both and add a third entry for this board. |

### Speed loop and references

| Change | Why | Verdict |
|---|---|---|
| Speed PI 4/8 (runs at 3.2/6.4), 30 ms speed LPF inside the loop | Seeded from XR | **Questionable:** with J ≈ 2.8e-3 kg·m², crossover is ~53 rad/s with ~22–25° margin. That matches the ~9 Hz speed mode seen in field weakening. A/B test 2/4, or a shorter filter. |
| SPORTS_MAX_SPEED 10500, overspeed trip 10800 | Bench target | **Validated** as a mechanism. **Risk:** both are above the TW-7 datasheet maximum of 10000 rpm. |
| Speed-only CAN mode, ECO forced to SPORTS | Bench | **Validated** by use. The ECO→SPORTS hack is ungated: remove it before any vehicle build. |
| Speed-sense glitch clamp → 1150 rad/s | Reach 10500 | **Justified** for TR. It's ungated, so XR/HiLoad lose glitch rejection. |
| atan2 speed step | sin(step) read −7.5 % at 6488 rpm | **Justified.** The maths is right. Ungated. |

### Protections

| Change | Why | Verdict |
|---|---|---|
| Overspeed trip (10 ms at ≥ 10800) | Bench safety | **Inert** (never fired). The logic is verified. Its reaction is PWM-off, so above ~5250 rpm it causes uncontrolled generation (~102 V at 10500). The power stage is rated for that; the supply is not protected. |
| OV/UV 80/25 V | 48 V bench | **Justified** for OV. UV at 25 V is effectively off. |
| `MAX_PHASE_CURRENT` = IQ_UP_LIMIT | Not stated | **Justified** for TR. **Risk:** ungated, so it lowers XR to 424 A and LivGuard to 492 A. |
| Encoder A/B and 5 V checks enabled for TR | Coverage | **Risk:** they need \|Id\| **and** \|Iq\| ≥ 250 A, but the clamp is 340 A, so they can never fire on TW-7. The Z and PWM checks still work. |
| `BENCH_FORCE_PWM_C` | Bootstrap test, "remove before committing" | Off by default. **Risk:** the comment doesn't match the code. Remove. |

### Telemetry, CAN and tools

| Change | Verdict |
|---|---|
| CAN_BUS_CYCLE 500 → 100 ms | **Validated:** it enabled every 10 Hz analysis. Ungated: 5× the telemetry load on vehicles. |
| Debug IDs 0x113/0x114, unique filter banks, 0x115 (τ) and 0x116 (fast telemetry), StdFiltersNbr 20 | **Justified.** `fdcan.c` is a CubeMX file: re-apply after regeneration. |
| 0x406 bytes 0-1 = τ; 0x715 byte1 = demanded modulation index, byte3 = clip ratio | **Justified**, and decoded by Parser and Parser2. |
| *Working tree:* 200 Hz telemetry (ext 0x420/0x421, 5 ms means) | **Inert** until a log exists. It's the only way to see the 9 Hz mode. |
| *Working tree:* CAN TX queue: push and drain inside critical sections; every push now tries to send | **Justified.** One TX stall in 18 logs. The vetting also found that the old "kickstart only if empty" check could strand the queue permanently. **Fixed** 10-29, not yet bench-tested. |
| Parser2 (scorecard, tuning state from logged commands, config from the git rev); GUI tuning panel | **Validated:** used on every 1 Nov log and on the real bus. |
| *Uncommitted, 1 Nov:* GUI trace logger and "Parse last log", Parser2 order-agnostic pairing and missed-frame count, `tools/host_tests/test_trace_log.py` | **Validated:** 104,987 frames logged on real hardware with none lost. Keep. |
| *Uncommitted, 1 Nov, never flashed:* ext 0x422 feed-forward telemetry (three blocks in `FOC_Loop.c`) and ext 0x40A encoder diagnostics, with their Parser2 and GUI decoding | **Candidates to drop.** The feed-forward values can be computed offline, and Z can't see a magnet slip. They'd take bus load from 35 % to 46 %. |
| CMake minimum 4.0, Build.bat `-j16`, incremental builds | **Questionable** for merge: it forces CMake 4 on everyone. |

---

## 4. Keep, re-test, remove

**Keep:**
- Closed-loop FW (H1).
- Output-delay compensation (H2), re-fitted after τ is set.
- Current-sense lag compensation (H3): it's the instrument for the τ sweep.
- Circle limit (H4) as a safety net.
- Bus model, with the cap.
- Offsets 1.128.
- Bench CAN plumbing.
- Overspeed logic.
- Fast telemetry and the queue fixes, once a log parses.

**Re-test.** One change per flash against `tw7-baseline-1028`, scored with `python tools/Parser2.py <log> --scorecard`:
1. τ sweep: 210, then 150 and 250 µs at a 6000 rpm hold. Change τ only at or below 3000 rpm. Then re-fit the 85 µs delay, λ and Ld.
2. Speed PI 2/4 vs 4/8, or a shorter speed filter. Targets the 9 Hz mode.
3. FW_CL_KI 100 vs 200. Targets the 3.8 Hz alternation.
4. VREF 90 vs 85.
5. Derate 0/0 vs 15/15.
6. Supply 54–58 V, with OP_VOLTAGE matched. Alternation starts near 180 rpm per bus volt, and 10500 rpm at 49 V is 214 rpm/V.
7. LAMBDA 0.0107.

**Added 1 Nov (late), defects found in review** (details and plan in the findings doc, "Deep review"):
- ADC2 reads phase V one period stale (`EOC_SINGLE_CONV` with both ranks read in the rank-1 callback). That's the 2fe ripple and about 46 µs of extra lag.
- The boot PWM decode uses a 4096-clock frame; the MT6816 frame is 4119. That's the 2.9° boot scatter.
- TIM2 encoder interrupts and ADC1 DMA half/full interrupts fire at about 450 kHz combined at 10500 rpm, for nothing.
- The separate feed-forward clamp causes the 3.3 Hz wobble in simulation. Removing it fixes the wobble at any λ.
- Speed mode has no negative torque. The anti-windup is slow, and the speed integrator range doesn't match its output.
- **Don't fix the ADC alone:** in simulation it shrinks the runaway margin from more than 20° to about 6°.
- **Safety (2 Nov):** `__HAL_TIM_MOE_DISABLE` is conditional in the HAL, so it does nothing while PWM runs (`Helper_Functions.c:291`, `main.c:690`, `tim17_handlers.c:54`). Only the PD15 inhibit stops the bridge on a fault. Port TR `fc1bede8`, `59f06c95` and `14ba2914`.
- **Bug (2 Nov):** `tim17_handlers.c:37` averages the ×1000 staging registers, so the Neutral offset is 1000× off after an autotune until reboot. Port `12c0c339`.
- **Fixes that already exist on other branches** (findings doc, "Other branches"): boot seed (`0a4bea82`, `ffff09e6`, `719e9772`), stale V (TURBO `ef588767`), interrupt load (`ea54585c`, `bf4aa9ff`), fast OC (`accaf382`). Port them rather than writing new code.

**Added 1 Nov, for a smaller diff:**
- ~~Put the tested settings in the profile~~ Withdrawn: remove the separate feed-forward clamp instead (findings doc, phase 2a).
- Set the Ld/Lq derate to 0/0.
- Keep the 0x422 and 0x40A telemetry. They verify the PWM decode fix and the feed-forward change.
- Don't add the `k` feed-forward or a TIM2 filter change; each was aimed at a cause now ruled out. Z re-sync is optional: a snap at the first Z below about 300 rpm, then monitoring only.
- Check whether the FW Iq ceiling ever engages. If not, it can go.
- First control change: remove the separate Vd/Vq feed-forward clamps (phase 2a), then a negative Iq floor with a DC-link limit (2b). The complex-vector PI is the backup.

**Remove or fix before anything merges toward master or a TR vehicle:**
- Give TW-7 its own motor profile, and restore the 76.8 V TR slot.
- Restore the divider table, and add a third entry for this board.
- Gate CAN_BUS_CYCLE, MAX_PHASE_CURRENT, the speed clamp and ECO→SPORTS to the bench build.
- Remove H8, H10, `BENCH_FORCE_PWM_C` and `Min/Max_DC_V`.
- Mark the unmarked hand-edits (H8, H9, and the H4 `#else` branch).
- Fix the stale comments in `motor_hirange_tr.h`. They still describe 76.8 V, 384 Arms, 550 A, VREF 90 and −259 A.

---

## 5. Open questions

1. **What causes the ~210 µs current-sense lag?** Mostly answered on 1 Nov:
   - an RC filter of about 132 µs (from the schematic);
   - a stale phase-V ADC sample (one period old, about 46 µs on average, and the source of the 2fe ripple);
   - about 26 µs of other delay.

   The split between these and Ld isn't identifiable from voltage data. A clamp probe and a log taken after the ADC fix would settle it.
2. **Which loop owns the 3.8 Hz alternation, FW or speed?** Their crossovers coincide at 9000 rpm, so only an A/B test separates them.
3. **Is 10500 rpm reachable at 49 V at all?** Yes: 33 s and 26 s holds on 1 Nov, with no clipping, though peak modulation reaches 1.00.
4. **ISR cycle budget:** H3 adds four sin/cos calls and fast telemetry adds sums. `d.cycles` hasn't been checked.
5. **CAN TX-complete interrupt:** it's enabled with buffer mask 0, so it may never fire. Transmission then relies on each push sending directly, which is why the queue fix matters.
6. **Startup angle convention:** is it right for the TW-7 sensor? There were two EEPROM-read failures that fell back to the default offset.
