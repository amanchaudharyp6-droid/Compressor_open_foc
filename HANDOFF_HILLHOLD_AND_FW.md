# Session Handoff — Pegasus firmware: SPORTS driveability, torque axis, Hill hold

**Date:** 2026-08-03 · **Branch:** `OTA_Reintegrated` · **HEAD:** `1de0f51` · **Vehicle:** HiRange XR,
96 V, `MOTOR_PROFILE_DEFAULT`
To resume on another machine: `git reset --hard origin/OTA_Reintegrated` (the branch history was
force-rewritten on 2026-07-31/08-02 — a plain pull can merge dead history back in), then tell
Claude Code to read this file.

---

## ⬅ START HERE — state of the tree, 2026-08-03

| Issue | Status | Where |
|---|---|---|
| **1** — Hill hold: throttle-blip oscillation, torque ratchet, unwind dumping | ✅ **DONE, vehicle-validated** | `5aa2fc9` |
| **2** — SPORTS 26–45 km/h noise + accel pull-back + top-speed shortfall | ✅ **SOLVED, vehicle-validated** | `16f438b` (+ `d1bbff0`, `aed5f83`, `3755469`, `0f871b3`) |
| **4** — SPORTS hard-decel jerk from top speed | ✅ **SOLVED, vehicle-validated** | `0efd163` |
| **3** — Commanded torque was never the delivered torque | ⚠ **FIXED in code, UNDRIVEN — the one live work item** | `1de0f51` (HEAD) |

**What to do next.** Only Issue 3 is open. The salient-torque fix (`1de0f51`) makes the torque axis
honest, but the caps were **deliberately not rescaled**, so today the vehicle is *correct but slower*
(a "40 Nm" SPORTS command now delivers a real 40 Nm instead of the ~90 Nm it used to). Before driving:

1. Rescale the torque caps from the table in **Issue 3** (the axis is now honest, so every cap,
   the power line, the speed-PI and the throttle expo are all reading the wrong scale).
2. Expect a large pedal-feel change (SPORTS effectively `x^2.83 → x^1.93`, ~+87 % torque at half
   pedal) — this is a driveability/safety change, not a bug.
3. **Do not drive HiRange TR at all** until its envelope is re-derived — its current caps command
   608 A pk / ~250 Nm, above the 550 A board phase-OC fault. See Issue 3.

**What was abandoned (do not resurrect).** The whole "the noise is over-modulation / blind
anti-windup / missing field-weakening" thesis was **empirically disproven**. Two commits built to
fix it — `b2aff90` (anti-windup hand patch) and `5236a65` (field-weakening feedforward module) —
were driven and had **zero effect**, and were reverted by `16f438b`. The earlier `FOC_VLIM_*`
voltage-clamp experiments (round 4–13 in memory) were the same dead end. The real cause was a rotor
angle-offset calibration error (Issue 2). The FW physics is kept below as a record because it is
correct physics, but it is **not a work item** — the angle advance already recovered full top speed.

> Note on the reverts: `b2aff90`/`5236a65` are still *ancestors* of HEAD (`16f438b` is a forward
> revert, not a history rewrite), so their effects are backed out but the commits remain reachable.
> `Field_Weakening.c/.h`, `FW_ENABLE`, the `FieldWeakening_Apply` seam, and the 0x715 modulation-index
> telemetry that those commits added are all **gone from the working tree** — grep finds them only in
> old commit blobs.

---

## Issue 1 — Hill hold: violent oscillation after a throttle blip

**Status: DONE, vehicle-validated** (`5aa2fc9`, on top of `68ffe2b`). No outstanding work.

What was wrong: (a) `HillHold_Apply()` was fed `FOC_Loop_U.RefTrq`, its own output, so `max()`
ratcheted the torque command and the takeover test compared the hold against itself — now fed
`d.torque_final` (`isr_handlers.c:297`, contract in `Hill_Hold.h:26-31`); (b) the learned grade
torque was dumped on any forward motion including the driver's own throttle — now gated on
`!driver_owns` with an `HH_STANDBY` state that retains it; (c) the unwind is a slow trim, bounded to
`(HH_CREEP_RPM 5, HH_UNWIND_MAX_RPM 60)`.

Validated gains (`vehicle_config.h`): `HH_KP` 0.05, `HH_KI` 1.1, `HH_MAX_TORQUE` 25, `HH_SLEW_NM_S`
200 (`:447-452`). ⚠ `HH_MAX_TORQUE` 25 is on the **old** torque axis — it has actually been
delivering ~43 Nm (see Issue 3); if the caps are rescaled, raise it to ~43 **together with**
`MAX_TORQUE` or the `HH_MAX_TORQUE <= MAX_TORQUE` static assert breaks.

**Regression signals** (CAN 0x710, `fdcan_AL.c:665-681`): byte[5] (`HillHold_GetState()`) should
read `1 → 4 → 1` (ACTIVE → STANDBY → ACTIVE) across a blip-and-lift on a gradient. byte[4]
(`HillHold_IsActive()`) flapping while byte[5] stays `1` means the unwind is dumping the estimate
again. byte[6..7] (hold torque ×10) should stay smooth, not step.

---

## Issue 2 — SPORTS: loud noise + torque sag + top-speed shortfall, 26–45 km/h

**Status: SOLVED, vehicle-validated.** Root cause was a **rotor commutation-angle-offset calibration
error**; the fix is a dynamic commutation-angle advance (`16f438b` and follow-ons).

### Symptom
- SPORTS only. Loud noise 26–45 km/h, growing from 26, decaying toward 45, gone above 45.
- Acceleration slows in that band; feels like the vehicle is pulled back.
- Top speed fell short (~55–58 km/h vs a 61–62 km/h target).
- On a flyover ramp **ECO sustained 33–35 km/h while SPORTS dropped to 25** despite the higher cap —
  a higher torque cap producing a *lower* sustained speed, which only happens if the larger command
  is destroying the delivered torque.

### Root cause (vehicle-proven)
A systematic ~0.1 rad-mech (~0.3 rad-elec ≈ 17° at 3 pole pairs) error in the EEPROM commutation
offset landed the current vector off-axis. Off-axis current needs **more voltage** for a given
torque, so the loop hit the voltage rail early under load — which is exactly what showed up as
"over-modulation" in the drive-cycle log and as the top-speed shortfall. Realigning the angle drops
the voltage demand back under the rail and every symptom clears at once.

The driver confirmed it directly: hand-subtracting `0.1f` from both EEPROM offset angles cleared the
noise and the pull-back and restored the full 61–62 km/h. A **fixed** −0.1 then jerked the vehicle on
hard decel (it over-advances during braking), which is why the shipped fix is *dynamic and
motoring-gated*.

### The fix — dynamic commutation advance (`isr_handlers.c`, `isr_foc_start`)

`isr_handlers.c:227-266`, inside `#if FOC_ANGLE_ADVANCE`, applied to `offset` before it is consumed
at `:273` (`d.mech_angle -= offset`):

```c
float rpm_frac = clamp((|d.RPM| - ADV_RPM_LO) / (ADV_RPM_HI - ADV_RPM_LO), 0, 1);
float trq_gate = clamp((d.torque_final - ADV_TRQ_LO) / (ADV_TRQ_HI - ADV_TRQ_LO), 0, 1);
offset -= copysignf(ADV_MECH_MAX * rpm_frac * trq_gate, d.RPM);   /* :264 */
```

Constants (`vehicle_config.h:424-428`): `ADV_MECH_MAX` 0.1f, `ADV_RPM_LO` 500, `ADV_RPM_HI` 2350,
`ADV_TRQ_LO` 7.0, `ADV_TRQ_HI` 15.0.

- **RPM ramp** — zero at launch, full advance above `ADV_RPM_HI` (covers the noise band and top end).
- **Torque gate** — ramps on the driver drive-torque, so the advance is applied only while
  *motoring* and backs off to the raw calibrated offset on release/braking (this is what kills the
  fixed-offset decel jerk). Gated on torque **sign**, not current magnitude, because noise-band accel
  (~130–170 A) and hard-decel regen (~135 A) overlap in magnitude — only direction separates them.
- **`copysignf(…, d.RPM)`** — the advance carries the rotor-direction sign (`d1bbff0`). The correction
  predicts where the rotor will be when the PWM lands (`θ_used = θ_sampled + ω_m·dt`, ω_m signed), and
  `mech_angle = count·K − offset`, so advancing the angle means *subtracting* from the offset with the
  same sign. `d.RPM` is the differentiated mech angle, so its sign is the rotor direction — keying off
  it (not a build flag) stays correct for either gear and either `MOTOR_DIRECTION`. No zero-crossing
  chatter: the advance is already zero below `ADV_RPM_LO`.

Follow-on refinements:
- `aed5f83` / `0f871b3` — advance **disabled on the LivGuard profile** (`FOC_ANGLE_ADVANCE` is
  `#if MOTOR_PROFILE == MOTOR_PROFILE_LIVGUARD → 0 else 1`, `vehicle_config.h:419-423`). Gated on the
  *motor profile*, not the vehicle variant: the advance corrects a motor/pack property, LivGuard
  shows none of these symptoms, and an uncalibrated 0.1 rad-mech advance would *inject* the very error
  it removes elsewhere. `0f871b3` also bound HiLoad XR ↔ LivGuard both ways (`#error` at
  `vehicle_config.h:130` and `:133`) — a HiLoad XR built on the 96 V default profile would set
  `BUS_DC_OV_LIMIT` 120 on an 86.4 V pack.
- `3755469` — removed the dead `d.offset_applied` field (written every tick, read nowhere since
  `30e56ed`; once the advance ran before it, it had also silently stopped meaning "the calibrated
  offset"). Repo-wide grep for `offset_applied` now returns nothing.

The Eeprom `-0.1f` hand shim was reverted; `Read_EEPROM_at_init` (`Eeprom.c:91-103`) uses the raw
`(float)…_16bit / 1000.0f` offset again — the 0.1 rad-mech correction lives only as `ADV_MECH_MAX`.

### Validated vs build-verified
- **Vehicle-validated:** forward, `DIR_NORMAL`, HiRange XR / `MOTOR_PROFILE_DEFAULT` — no noise, no
  pull-back, strong SPORTS accel, full 61–62 km/h (`16f438b` message).
- **Build-verified only:** the reverse / `DIR_FLIP` rotor-direction-sign path (`d1bbff0`) is reasoned
  but never driven — worth a reverse-at-speed check. The LivGuard disable and the pairing guards are
  compile-time constructs.

### The false trail — kept as record (NOT a work item)

The controller-side analysis (rectangular voltage limit, blind anti-windup, missing field weakening,
the `FOC_VLIM_*` clamp rounds) was chasing a downstream symptom. The evidence that *looked* like
over-modulation is real and worth keeping, because it constrains any future work:

Drive-cycle log, `20-07-2026 NIGHT- HiRange 2.0 XR` (21,522 forward-motoring samples,
`tools/pcan_fw_analyse.py`), commanded voltage vs the ~56 V rail:

| Band (rpm) | km/h | mean \|V\| | max \|V\| | % over the SVM circle |
|---|---|---|---|---|
| 2000–2500 | 21–26 | 45.9 | 79.2 | 13.9 % |
| 2500–3000 | **26–31** | 58.6 | 79.2 | 43.1 % ← noise starts |
| 3500–4000 | 37–42 | 76.1 | 79.2 | 94.9 % |
| 4000–4500 | **42–47** | 78.3 | 79.2 | 99.5 % ← noise ends |

`max |V|` is 79.2 V = √(56²+56²) in every band — both axes railed at once. The noise band is where
saturation is *intermittent* (43 % → 99.5 %); above 45 km/h it is permanent (100 %) and therefore
acoustically steady, which is why it *decays* toward 45 rather than stopping abruptly.

Why this was a symptom, not the cause: with the angle mis-aligned, the machine needed excess voltage
for the torque it made, so it hit the rail early. Two vehicle A/Bs settled it — `b2aff90` +
`5236a65` drove with **zero effect** on the noise, and the driver's angle shim cleared it completely.
The full round-by-round record is in the `fw-entry-limit-cycle` memory.

The FW physics remains correct and is preserved in git history if genuine headroom beyond the
advance is ever needed: steady-state, the operating point must sit inside the ellipse
`(Lq·Iq)² + (Ld·Id+λ)² ≤ (Vmax/ω_e)²`, centred at `(−λ/Ld, 0)`; `Ich = λ/Ld = 177.7 A < Imax`, so
the machine is capable of arbitrarily deep weakening. But the vehicle reached 61–62 km/h on the
advance alone, so there is nothing to build here today.

---

## Issue 3 — Commanded torque was never the delivered torque  ⚠ LIVE / UNDRIVEN

**Status: fixed in code (`1de0f51`, HEAD), caps deliberately NOT rescaled, no vehicle test yet.**

Found 2026-08-03 from a vehicle observation: on HiRange XR the estimator's torque output on 0x724
read **88 Nm** against a 40 Nm SPORTS cap, and 25–26 Nm against a 16 Nm ECO cap. The estimate was
closer to the truth than the command.

### The defect

`Current_Ref_Estimator.c` inverted one torque equation and evaluated a different one:

| Step | Block | Equation |
|---|---|---|
| torque → current | `Divide ':5504'` (`:226`) | **non-salient**: `I_qo = 2·T / (3·P·λ)` |
| current → Id/Iq | MTPA split `':5521'/':5522'` | takes `I_qo` as the **total magnitude** |
| Id/Iq → torque | `Torque_calculation ':5575'` (`:842`) | **salient**: `1.5·P·(λ·Iq + (Ld−Lq)·Id·Iq)` |

Two compounding errors — a q-axis current reused as a magnitude (Iq drops to `I_mag/√2`, −29 %), and
the reluctance torque the split just created is absent from the inverse:

```
T_delivered ≈ T_cmd/√2 + (Lq−Ld)·T_cmd² / (3·P·λ²)
```

The error scales as **(Lq−Ld)/λ²**, so the low-flux/high-saliency `default` profile is the worst
case. Below the crossover it *under*-delivers; above it, it runs away.

| Profile | (Lq−Ld) | λ | crossover |
|---|---|---|---|
| `default` (HiRange XR, TR) | 238 µH | 26.3 mWb | 7.7 Nm |
| `livguard` (HiLoad XR) | 110 µH | 33.76 mWb | 27.3 Nm |

### Confirmed three ways

HiRange XR SPORTS, 40 Nm cap: measured **236 Arms** on a gradient; the model predicts 239 Arms and
**~90.9 Nm delivered**; the 0x724 estimator read **88 Nm**. A correct inverse needs only **143 Arms**
for a genuine 40 Nm. ECO 16 Nm delivered 22.4 Nm.

| Variant / mode | Measured | Implies `T_cmd` | Really delivering |
|---|---|---|---|
| HiRange XR SPORTS | 236 Arms | 39.5 Nm (cap 40 — binding) | **89.0 Nm** |
| HiRange XR ECO | 78 Arms | 13.0 Nm (power cap, not the 16 Nm cap) | 16.8 Nm |
| HiLoad XR SPORTS | 383 Arms | 82.3 Nm | 135.6 Nm — **but that vehicle is on an older build** |

⚠ **Confirm which build the HiLoad vehicle runs before trusting its numbers** — 383 Arms is above
this tree's `IQ_UP_LIMIT` (492 A pk = 348 Arms), so the tested HiLoad build is not this tree.

### The fix (applied, `Current_Ref_Estimator.c`)

Solve the salient equation **on** the MTPA locus. With `q = −Id` and `c = λ/(Lq−Ld)`, normalising
`x = q/c` reduces it to a scalar root-find:

```
x(x+1)³ = k ,   k = ( T_ref·(Lq−Ld) / (1.5·P·λ²) )²
```

Dimensionless and well conditioned (`k` stays O(1…40) where the raw form reaches 1e9). `f(x)` is
increasing and convex on `x ≥ 0` and both `k` and `k^{1/4}` bound the root, so Newton from
`min(k, k^{1/4})` descends monotonically — no bracketing, no overshoot, fixed iteration count,
constant ISR time. Verified in float32 against the generated split and torque equation over 0–120 Nm
on both profiles and the whole degrade range: **5 iterations → 5e-5 Nm**.

- HAND-EDIT comment block `Current_Ref_Estimator.c:275-316`; solver `:317-374`.
- Placed **after** `Motor_Parameter_Degrade ':5261'` (`:261-273`) so it sees the same degraded `Ld`
  the split uses. `Divide ':5504'`/`Switch2 ':5517'` are left untouched and `I_qo` survives as a
  diagnostic.
- **Model fix:** replace `Divide ':5504'` with this solve, then delete the block (see Hand patches).

### Rescaling the caps — required, not optional

Every torque constant is a **command** calibrated against the inflated axis. Left alone, the fix
makes the vehicle dramatically slower (SPORTS launch 90 → 40 Nm actual). Setting each constant to its
"delivers" value reproduces today's delivered torque with an honest label.

**It does not reproduce today's pedal feel.** The defect's quadratic compounded with
`THR_EXPO_SPORTS` 0.95 (`vehicle_config.h:347`), so the SPORTS curve is effectively `x^2.83` today and
becomes `x^1.93` after the fix + rescale — WOT matches, but ~+87 % torque at half pedal. A
driveability and safety change; expect it on the first drive.

Two further stages also computed in commanded Nm become correct only after the rescale: the **power
cap** `T = P/ω` (so today it permits more than `SPORTS_MAX_POWER`/`ECO_MAX_POWER`), and the
**speed-limiter PI** (`SPEED_PI_KP`/`KI`, vehicle-validated against the inflated response).

#### HIRANGE_XR (`default`, clamp 424 A pk) — today's values verified in tree

| Constant | Today | → rescaled | file:line |
|---|---|---|---|
| `ECO_MAX_TORQUE`, `REV_MAX_TORQUE` | 16 | **22.4** | `:163`, `:172` |
| `SPORTS_MAX_TORQUE`, `MAX_TORQUE` | 40 | **90.9** | `:170`, `:171` |
| `HH_MAX_TORQUE` | 25 | **43.0** | `:449` (single global define) |
| `REGEN_MAX_TORQUE_ECO` | 5 | **5.3** | `:248` |
| `REGEN_MAX_TORQUE_SPORTS` | 3 | **3.1** | `:249` |

`TORQUE_SPEED_TABLE` is in **`Torque_Management.c:41-56`** (not the header). Today (rpm 0/500/1000/
1500/2000/2500/3000/4400/5100/5300/5500/6000/6500): `40 40 38 30 24 21 20 13.5 11.5 10.9 10.4 9 8` →
rescaled `90.9 90.9 83.5 57.0 40.4 33.0 30.8 17.7 14.3 13.4 12.6 10.5 9.1`.

#### HILOAD_XR (`livguard`, clamp 492 A pk)

**Leave SPORTS at 80 Nm.** It is clamp-limited in this tree and the vehicle is already at 132 % of
rating; a true 80 Nm needs only 266 Arms under the fix, so accept the (safer) change in feel rather
than rescaling to 117.6. `ECO_MAX_TORQUE`/`REV` 32 → **37.4**. `REGEN_*` 5 → 5.0.

#### ⚠ HIRANGE_TR (`tr`, clamp 652 A pk) — DO NOT DRIVE UNTIL RE-DERIVED

TR shares the motor electricals but raised `IQ_UP_LIMIT` to 652 A, so nothing clamps the runaway.
`SPORTS_MAX_TORQUE` 72 commands **608 A pk / ~250 Nm** — above the 550 A board phase-OC fault
(`MAX_PHASE_CURRENT`, `board_config.h:77`), so full SPORTS most likely trips overcurrent rather than
delivering torque. Do **not** mechanically rescale; re-derive TR's envelope from the 80 Nm motor peak
and the 5.655 kW power line in honest Nm. TR has never been vehicle-tested.

### Not yet done
- Caps not rescaled — deliberate. The build is *safe* but slow until they are.
- No vehicle test of the fix yet.

---

## Issue 4 — SPORTS hard-decel jerk from top speed (SOLVED)

**Status: SOLVED, vehicle-validated** (`0efd163`) — mode-selective regen.

After the angle advance cleared the noise, a residual remained: releasing full throttle from 58–62
km/h jerked the vehicle at ~35–43 km/h (regen re-engaging at the fade end). ECO had no jerk but its
regen was too weak. The mechanism (from the earlier `sports-decel-regen` work): a regen command in
the high-speed region triggers a large current-**angle** excursion, so a small command produces
disproportionate braking — keeping regen small/faded at speed avoids that regime.

Fix — split regen by drive mode in `update_regen_fade(float speed_rpm, power_mode_t drive_mode)`
(`Torque_Management.c:217`, call site `:513`):
- **ECO — no fade** (`:234-239`): flat cap `REGEN_MAX_TORQUE_ECO` across the whole range for
  consistent deceleration; ECO torque is low enough the excursion does not bite.
- **SPORTS — fade** (`:221-233`): `(1 − smoothstep)` from `REGEN_MAX_TORQUE_SPORTS` at
  `REGEN_FADE_START_RPM` down to 0 at `REGEN_FADE_END_RPM`, plus a reduced SPORTS cap.

Config (`vehicle_config.h`): HiRange XR `REGEN_MAX_TORQUE_ECO` 5.0 / `_SPORTS` 3.0 (`:248-249`);
`#define REGEN_MAX_TORQUE REGEN_MAX_TORQUE_SPORTS` is the FOC model **init default only** (`:259`),
overwritten every loop once driving. Fade band `REGEN_FADE_START/END_RPM` = 2500/3500 for
HiRange XR / HiLoad (`:324-327`), 2240/2800 for HiRange TR (`:328-331`, a seed). `REGEN_FADE_ENABLE`
1 (`:323`). `REGEN_MIN_RPM_EN` 600 (`:260`) is the separate regen-arming floor, not part of the fade.

> Config-hygiene nit: the TR comment "seeded = HiRange XR" at `:256` is wrong (TR is 8, XR is 5).

---

## Facts worth not re-deriving

**CAN decode (VH_CAN_ID build)** — full table in the header of `tools/pcan_fw_analyse.py`. All
frames (0x705/706/708/709/710/715/716/717/724/726) drain off a 500 ms queue → **~2 Hz**
(`CAN_BUS_CYCLE` 500, `board_config.h:64`; gate `tim17_handlers.c:127`). Fine for steady-state binning
over 20k+ samples; useless for current ripple. Traps:

- Id/Iq carry a **+512** offset (`fdcan_AL.c:541-542`); Vd/Vq a **+62** offset (`:732-733`, below
  −62 V clips to raw 0).
- **The sign of `Iq` for forward motoring is NOT fixed across vehicles** — the gearbox orientation
  flips `MOTOR_DIRECTION`/`REGEN_POLARITY` and `Iq` flips with it. The asymmetry that saves the
  analysis: `Id_ref_MTPA = K − √(K² + I_mag²/2)` is **always ≤ 0** and orientation-invariant, while
  `Iq_ref` carries `sign(T_ref)` (`Current_Ref_Estimator.c:393-394`, sign block `:418-427`;
  `K = λ/(4(Lq−Ld)) = 27.63 A`). So Id never flips, only Iq. The tool detects the sign per log; it was
  **−1** on the 20-07-2026 log.
- **`torque_ref` on 0x724 cannot determine direction** — it arrives as a positive magnitude.

**0x715 payload today** (`fdcan_AL.c:684-695`): bytes [0]/[1] **spare/zero**, [2] `MTPA_FWC_condition`,
[3] pack voltage, [4] bus voltage, [5] 12 V, [6..7] RPM u16 LE. *(The Id / modulation-index bytes an
earlier handoff described were added by `5236a65` and removed with it — they do not exist.)*

**Rate limiters** are **per second** — the model multiplies by `d.Op_ts`, so `50.0f * 10000.0f`
means 50 A/tick = 500,000 A/s (`FOC_Loop.c:1680-1683`). The Id/Iq ramps never bind.
`FOC_TRQ_RAMP_*` = 150 Nm/s (`motor_default.h:129-130`) is a deliberate non-binding backstop above
the app slews, enforced by `_Static_assert` in `Torque_Management.h:235-239`.

**Bus-voltage model.** `Sensor_Disconnection_Checks.c:146`:
`model_busv = fmaxf(OP_VOLTAGE, can_pack_voltage)`, then clamped to `BUS_DC_OV_LIMIT` at `:147`, fed to
`d.Vmax_SVM = model_busv/√3` (`:149`). `1754b40` made the `OP_VOLTAGE` pin a *floor*, using BMS pack
voltage above nominal — suspect, because `can_pack_voltage` is measured at the pack (frame rate), not
the inverter DC link, so the two diverge under SPORTS current exactly when it matters. Byte-identical
to the old fixed pin at or below 96 V, so a low-SOC run is a free controlled experiment.

**HiRange XR constants** (`motor_profiles/motor_default.h`): λ = 0.0263 Wb (`:11`), Ld = 0.148 mH
(`:10`), Lq = 0.386 mH (`:9`), Rs = 10.47 mΩ (`:8`), 3 pole pairs (`:7`), `MOTOR_PEAK_AC` = 353.55 A
(250 Arms × √2, `:18-19`), `CURR_TORQUE_RATIO` = 80/(250·√2) ≈ 0.2263 Nm/A (`:20`),
`RPM_TO_KMPH` = 0.01044 (`vehicle_config.h:79`). Clamps: `IQ_UP_LIMIT = ID_UP_LIMIT =
1.20·MOTOR_PEAK_ARMS·√2` → 424 A pk default, 492 A pk LivGuard, 652 A pk TR.

**Stale README values**: the FW table at `README.md:677-685` still says `Kfw` 0.001 / `M2F` 0.7 /
`F2M` 0.5 / `RST` 0.7 — actual is 0.002 / 0.85 / 0.55 / 0.45 (`motor_default.h:102-106`). (FW is inert
and unused; noted only so the README isn't trusted.)

---

## Hand patches in Simulink-generated code

`FOC_Loop.c`, `Current_Ref_Estimator.c`, `MCU_Protections*.c`, `Open_FOC0.c`, `rtGetNaN.c`,
`rt_nonfinite.c` are **generated** — every edit is lost on model regeneration.

**Grep before and after any regeneration** (every marker uses `HAND-EDIT`; the literal `HAND PATCH`
with a space matches nothing):
```
grep -n "HAND-EDIT\|NOT IN THE SIMULINK MODEL" \
     Core/Src/FOC_Loop.c Core/Src/Current_Ref_Estimator.c
```

### Algorithm changes — live, unconditional, reverted silently by regen

| Edit | Where | Commit | Model fix needed |
|---|---|---|---|
| Regen exit guards test polarity FIRST, so a throttle re-press routes to the driver's real drive state instead of always firing Reverse | `FOC_Loop_Regen_mode1()`, `FOC_Loop.c:85` | `ea25970` | Yes — `FNR_switching` chart exit guards |
| Low/high-speed PID switch: `T_limit <= 0.0F` → `T_limit <= Actual_torque_from_meas` | `FOC_Loop.c:298` (Switch `':5057'`, RelOp2 `':5056'`, compare `:319`) | `6776a32` | Yes — the `0.0` constant on `':5056'` |
| Salient torque inverse — solve on the MTPA locus after `Motor_Parameter_Degrade ':5261'` so commanded torque = delivered torque (Issue 3) | `Current_Ref_Estimator.c:275` | **`1de0f51`** | Yes — replace `Divide ':5504'` with the solve, then delete the block |

The PID-switch edit was **live and unmarked for a month**: with `LOW_SPEED_I_T_SWITCH` = 1 and
`LOW_SPEED_T_LIMIT` = 10.0 the generated form `10.0 <= 0.0` is constant false, so the high-speed gain
set was unreachable. Regenerating without re-applying it silently puts the machine back on low-speed
gains at all speeds, with no build error.

> The anti-windup back-calculation hand patch that an earlier handoff listed here was `b2aff90`, which
> `16f438b` reverted — the stock `(Vd_PID − rtb_Sum)` back-calculation is back at `FOC_Loop.c:1362`
> (d) and `:1380` (q). There is nothing to mirror.

### Init-value parameterisation — low risk (regen reverts to hard-coded values, a tuning regression)

| Change | File | Commit |
|---|---|---|
| Id/Iq clamp 560 A → `IQ/ID_UP_LIMIT` | `Current_Ref_Estimator.c:954-957` | `022338e` |
| `Torque_gen_ramp` `0.005*10000` → `FOC_TRQ_RAMP_*_NM_S` | both (`FOC_Loop.c:1678`, `Current_Ref_Estimator.c:965`) | `2eab8ed` |
| `Regen_Active` → `d.regen_active` | `FOC_Loop.c:1635` | `ea25970` |
| `Dead_Time_Compensator.Enable` → `DEAD_TIME_COMP_ENABLE` | `FOC_Loop.c:1668` | `1e68ec4` |

---

## Build

```
docker compose run --rm builder
```
Local `cmake` is broken (the checked-in cache points at the container's `/work`), and `Build.bat`
nukes `Build/`. Only pre-existing warnings should appear, in `Core/Src/Open_FOC0.c:79`
(`-Wdouble-promotion`).

---

## State of the tree

HEAD `1de0f51`, branch `OTA_Reintegrated`, working tree clean. Linear history above the hill-hold
base `68ffe2b`:

```
68ffe2b  Hill Hold: vehicle-validated gain retune + CAN 0x710 telemetry
5aa2fc9  Hill Hold: fix throttle-blip oscillation, torque ratchet and unwind dumping   (Issue 1 — DONE)
639f0bc  Mark every hand edit in the Simulink-generated sources
d085121  Docs + tooling: this file, tools/pcan_fw_analyse.py, .gitignore
b2aff90  FOC: fix blind anti-windup …                 ← reverted by 16f438b (inert)
5236a65  Field weakening: application-owned module     ← reverted by 16f438b (inert)
42e7686  Docs: START HERE note …                       ← the now-removed stale banner
16f438b  Fix SPORTS noise + top speed: dynamic commutation-angle advance   (Issue 2 — reverts b2aff90+5236a65)
0efd163  Added regen according to modes …             (Issue 4)
d1bbff0  Angle advance: carry the rotor-direction sign
3755469  Remove the dead offset_applied field
aed5f83  Angle advance: disable on the LivGuard motor profile
0f871b3  Bind HiLoad XR and the LivGuard profile both ways
1de0f51  FOC: invert the salient torque equation, not the non-salient one   (Issue 3 — HEAD, UNDRIVEN)
```

History was force-rewritten on 2026-07-31 and 2026-08-02 (squashing hill-hold work, dropping the
reverted `FOC_VLIM_*` voltage-limit experiments). Any other clone must
`git reset --hard origin/OTA_Reintegrated` rather than pulling, or it will merge dead history back in.

---

## What is actually left

1. **Rescale the torque caps (Issue 3)** — deliberately not done. The build is honest but slow until
   they are. After rescaling, re-validate pedal feel, hill-hold firmness, the power cap and the
   speed-PI, all of which read the now-honest axis.
2. **Do not drive HiRange TR** until its envelope is re-derived (250 Nm / 608 A above the OC fault).
3. **Mirror the three live hand edits into the Simulink model** so they survive regeneration — the
   two Tier-1 edits (regen exit `ea25970`, PID switch `6776a32`) and the salient inverse `1de0f51`
   (replace `Divide ':5504'` with the solve, delete the block).
4. **Reverse / `DIR_FLIP` angle-advance sign** (`d1bbff0`) is build-verified only — worth a
   reverse-at-speed check.
5. **Config hygiene** — fix the TR regen comment "seeded = HiRange XR" (`vehicle_config.h:256`) and
   re-center the TR regen-fade band once there is a logged TR knee.
