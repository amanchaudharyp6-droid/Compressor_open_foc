# TW-7 bench bring-up, explained

For the engineer running the bench. It tells the whole story in plain words: the goal, the ideas you need, what we changed, what the logs proved, and what to do next. Terms follow `CONTEXT.md`. The change-by-change detail is in `TW7_BRINGUP_CHANGES.md`. This document is the readable version.

Status as of 29 Oct 2026, branch `tw-7_bringup`, baseline tag `tw7-baseline-1028`.

**Update, 1 Nov.** Parts of this page are out of date. [TW7_BRINGUP_FINDINGS.md](TW7_BRINGUP_FINDINGS.md) has the current picture. In short:
- 10500 rpm has held for 33 s and 26 s, with λ 0.0085 and speed PI 3.0/6.1 sent after boot, and τ 130.
- The textbook motor equations are fine. The current reading is what's off: at 10500 rpm it lags about 59° in total, and τ 130 corrects 41° of that. Part of the lag is a firmware bug: phase V is read one PWM period late, which is also the 2fe ripple. The real current there is about −85 to −95 A / 5–8 A, not the −72 A / 27 A in the logs.
- The wobble came from the Vq feed-forward being clamped on its own. A simulation reproduces it, and removing that clamp fixes it at any λ.
- The boot-to-boot angle scatter (about 3°) fits a frame-length error in the encoder PWM decode.
- Speed mode can't command negative torque, so an angle error turns into a runaway. Fixing the ADC without the angle fixes and a negative floor first would make this worse.
- The alignment between the encoder and the rotor has shifted in lasting steps. The check at 3300 rpm now reads about −1.3 to −2.3 V, and below −4 V means stop.
- Don't raise τ yet: 130 leaves about 20° of margin against a runaway.
- The plan is the phase table in the findings doc's Deep review.

---

## 1. The goal and the bench

**The motor.** TW-7 (HMC), an interior-magnet motor with 5 pole pairs. It runs on the `HIRANGE_TR` profile slot, which was made for a different, vehicle motor. That history matters later.

**The bench.**
- The rotor is free. There is no load. Friction and windage need only about 5 A of torque current.
- The supply is 48–50 V and cannot absorb power. Any energy the motor pushes back goes into the DC link, and the bus voltage rises.

**The goal.** Get the motor **cleared** at 10500 rpm. Cleared has a strict meaning. You ramp up at the bench rate and hold for 10–20 s. There must be no trip, no reference alternation, and speed σ under about 10 rpm. Then you ramp down to zero with no trip. You do all of that twice in a row.

Why it is hard: at 49 V the motor is already past the point where its own back-EMF uses up the voltage the inverter can give. Everything below is about working in that region, and about not being fooled by the measurements while you do.

---

## 2. The concepts

### d/q currents: field and torque

The controller does not steer the three phase currents directly. It turns them into two numbers that sit still while the motor spins, like looking at a spinning wheel from a camera that spins with it.

- **Id** points along the rotor magnet. It adds to or subtracts from the magnet's field. It makes almost no torque.
- **Iq** points across the magnet. It makes the torque.

Pushing Id negative weakens the magnet's field. That trick is the next idea.

### Base speed, back-EMF and the voltage circle

A spinning magnet makes a voltage in the windings, the back-EMF. It grows with speed. The inverter can only make a voltage up to a limit, the **voltage circle**: the DC link voltage divided by √3 (about 28 V on a 49 V bus).

The **modulation index** tells you how much of the circle is used. 1.00 means the edge. The controller keeps a voltage pair that it wants (the *demanded* modulation index). The circle limit then trims it if needed (the *applied* modulation index). The **clip ratio** counts how often it had to trim. **D-axis priority** means Vd is kept and Vq gets what is left.

Base speed is where the back-EMF alone fills the circle. On this bench that is close to where **uncontrolled generation** begins, about 5250 rpm (see below). Above base speed, the controller needs a trick to spin faster.

### Field weakening and the closed-loop FW regulator

The trick is **field weakening (FW)**: a negative Id cancels part of the magnet's field. The back-EMF drops, so the motor spins faster inside the same voltage. The price is current, and so heat. At 6000 rpm the free rotor needs about 5 A of torque current, but carries about 46 A of Id.

How much negative Id? The **closed-loop FW regulator** (hand-edit H1) decides. It watches the voltage the current regulators ask for. If that is above 85 % of the circle, it makes Id more negative, slowly. If it is below, it relaxes. Id is never allowed above zero. Iq is trimmed so the current stays inside its limit.

The generated model's own FW block could not do this job, because it worked from voltages computed before FW. That is why we added the closed-loop version. It is the one change the logs prove (section 4).

### Current loops and the speed loop

- Two **current loops**, one for Id and one for Iq, run fast. They turn a wanted current into a voltage.
- One **speed loop** runs slower on top. It turns "speed too low" into a wanted Iq.

The speed loop has a floor: its Iq reference cannot go below zero. It can ask for push, never for a brake. Remember this for the runaway story.

**Why gains are stuck at ×0.8.** The firmware has two gain sets. The high set only switches on when speed is at least 400 rpm and measured torque is at least 10 Nm (hand-edit H6). The free rotor never makes 10 Nm. So every loop always runs at 0.8 times its profile value. The loops that actually run are:

| Loop | Profile gains | What runs |
|---|---|---|
| Id PI | 0.17 / 3.5 | 0.136 / 2.8 |
| Iq PI | 0.15 / 4.0 | 0.12 / 3.2 |
| Speed PI | 4 / 8 | 3.2 / 6.4 |

When you compare tuning runs, compare the effective values.

### Current-sense lag and τ (the important one)

**The problem.** The phase currents reach the controller about 0.2 ms after they flowed in the motor. The ADC conversion itself takes only a few µs. The rest is somewhere in the sensor and analog path. The cause is not found yet. This is the **current-sense lag**.

A 0.2 ms delay sounds small. But at 6000 rpm the rotor turns about 38 electrical degrees in 0.2 ms. At 8800 rpm it turns about 57°.

**The camera analogy.** Imagine a photo of the spinning rotor, taken 0.2 ms ago. You then measure the arrow in that photo against a protractor that is lined up with where the rotor is *now*. The arrow looks turned by 38°. Nothing is wrong with the arrow. Your protractor is just in the wrong place.

That is exactly what happened. The firmware splits the phase currents into Id and Iq using the rotor angle *now*. Part of the field current (Id) was counted as torque current (Iq). A free rotor that needed about 5 A of torque current showed 34–64 A of Iq. The **measured current frame** was rotated away from the **voltage frame**.

**The fix.** The currents are steady sine waves. For a steady sine wave, "this is old by a time τ" is the same as "this is turned by a fixed angle". So we do not need to chase the delay. We just use the angle the rotor had back then:

> angle for the split = angle now − electrical speed × τ

τ is set live over CAN (0x115). It boots at 0. This is hand-edit H3.

**What τ is not.**
- It is **not an observer**. There is no motor model, no states and no feedback. It is one multiply and one subtract.
- It **corrects which way the current vector points**. It does **not** make the reading arrive sooner. The current loops still act on 0.2 ms-old data. That is fine at their present speed. A predictor or observer would only be the next step if much faster current loops are ever wanted.

**Today's τ sweep (29 Oct, `29_10/sweep.trc`), 6000 rpm holds**

| τ | Id | Iq | \|I\| | Vd, Vq |
|---|---|---|---|---|
| 0 | −33.7 A | 33.5 A | 47.5 A | −2.4 V, 24.1 V |
| 150 µs | −42.8 A | 13.7 A | 44.9 A | same |
| 210 µs | −45.8 A | 5.1 A | 46.1 A | same |

Before the run we predicted Iq ≈ 5 A and Id ≈ −46 A for 210 µs. The log matched.

How to read this:
- **The voltages did not change.** So the motor's real operating point never moved. Only the controller's view of the current was corrected. The picture rotated back.
- **All three holds put the true lag at 205–208 µs.** Three different settings agree, which is strong evidence.

**τ = 250 µs ran away.** On the ramp up, from about 5850 rpm, the speed ran off to 7400 rpm in 1.5 s while the reference stayed at 6000. The speed loop had already cut Iq_ref to 0, its floor. It could not brake. FW kept pushing Id_ref more negative (−45 A to −74 A). The currents then blew up (−148 / −274 A) and the drive tripped. The bus rose to 66 V.

Why: with τ too big, the controller over-rotates the current it measures. A more negative Id then makes real *forward* torque while measured Iq still reads 0. More speed means more FW, which means more negative Id, which means more torque. That is positive feedback, and the speed loop cannot brake against it. With τ too small, the same coupling brakes instead. That wastes current but stays stable.

So **undershoot is safe and overshoot is not.** That is why we chose **τ = 200 µs**, just under the measured 205–208 µs.

Two caveats. The trip was reported as `BUS_VOLTAGE_UV_ERROR` although the bus was 47–48 V at the time. We suspect a fault-reporting bug, the same as an earlier unexplained UV trip, and it is not investigated. Do not trust the UV label; read Id, Iq and rpm instead. Also, fast telemetry was off in this log, so the runaway is seen only through 10 Hz frames.

### Output-delay compensation (a different delay)

This one is on the **voltage side**. The voltage the controller computes is applied about 85 µs later, after the PWM update. In that time the rotor has moved on, so the voltage lands at the wrong angle. Hand-edit H2 aims the voltage ahead by 85 µs of rotation.

Two different delays, two different fixes:

| | Where | Size | Fix |
|---|---|---|---|
| Current-sense lag | current in, before the split | about 205–208 µs | split with an older angle (τ) |
| Output delay | voltage out | 85 µs | apply the voltage at a newer angle |

Output-delay compensation works below base speed: the voltage-angle error fell from +6…+11° to −3…+2°. The 85 µs figure was fitted while the current frame was still rotated, so **re-fit it now that τ is set**. τ compensation needs it switched on (the build fails otherwise).

### The two oscillations at 9000 rpm

Two separate things appear near 9000 rpm. Do not mix them up.

- **Reference alternation, about 3.8 Hz.** Id_ref and Iq_ref swing in opposite directions. At 10 Hz telemetry it looks like a flip from frame to frame with a slow beat. The 3.8 Hz is real, not an aliasing artefact. It starts at roughly 180 rpm per bus volt. 10500 rpm at 49 V is 214 rpm per volt, so more supply voltage should help.
- **Speed hunting at about 9 Hz.** This is a mode in the speed loop. 10 Hz telemetry shows it aliased as 0.3–2 Hz. Its loop crossover is about 53 rad/s with only 22–25° of margin, with a 30 ms speed filter inside it. That matches.

Suspects: the FW regulator gain (`FW_CL_KI` 200 gives about 50–60 rad/s at 9000 rpm, on top of the speed loop) and the speed PI (4/8). Their crossovers sit on each other at 9000 rpm, so only an A/B test can tell which one owns the 3.8 Hz. In all 6 trips so far the story was the same: alternation grows, speed jumps, the d-axis is lost, and the bus rises last.

### Uncontrolled generation

When the inverter switches stop, the motor can still charge the DC link through the inverter's diodes. That happens once the line back-EMF peak is above the bus voltage. On TW-7 (λ about 10.7 mWb) at 49 V, that starts at about 5250 rpm. At 10500 rpm it reaches about 102 V.

So **any trip above about 5250 rpm pumps the DC link.** The overspeed trip (10800 rpm) is no exception, because its reaction is PWM-off. The power stage is rated for this. The supply is not protected. It is not "regen": regen is controlled braking current, and this is not controlled.

### Why the speed loop cannot brake

Its Iq_ref floors at zero. On a free rotor with a supply that cannot absorb power, a braking command would also push energy back into the bus. So the missing brake is both a design limit and, on this bench, a protection. It is also what turns the τ-too-big case from a mild error into a runaway. Whether to let the loop brake is an open decision (section 5).

---

## 3. What we changed, and why

"Verdict" in plain words: **works** (a log shows it), **not tested yet**, **doubtful** (the reason given was refuted or came from another motor), **remove before vehicle**.

| Change | What it does | Verdict |
|---|---|---|
| Closed-loop FW (H1) | Lets FW actually act on the voltage | **Works.** Iq tracking error went from −49…−74 A to about 0. Peak modulation index fell from 1.05–1.08 to 0.92–0.94. |
| Output-delay comp, 85 µs (H2) | Aims the voltage ahead | **Works** below base speed. Re-fit with τ set. |
| Current-sense lag comp, τ (H3) | Fixes the rotated current picture | **Works.** Today's sweep. Use 200 µs. |
| Circle limit with d-axis priority (H4, H5) | Safety net on voltage size | **Not tested yet.** Clip ratio 0–2 %, so it never mattered. Keep, but give it no credit. |
| Gain-set switch (H6) | Restores the high/low gain switch | **Stuck at ×0.8** on this bench. Not a defect on the bench. |
| λ = 11.21 mWb | Datasheet flux | **Doubtful.** Measured about 10.4–10.8 by two methods. Plan to set 0.0107. |
| Ld/Lq derate 15/15 | "Smoother at high speed" | **Doubtful.** It scales with torque × speed, and the rotated frame fakes about 5 Nm. Use 0/0 on the bench. |
| `FW_CL_KI` 200 | FW regulator speed | **Doubtful.** Designed on the XR motor. May drive the 3.8 Hz alternation. |
| `FW_CL_VREF` 85 % | Voltage target for FW | **Doubtful.** It was said to remove alternation. The logs show none at 90 and some at 85. |
| Speed PI 4/8, 30 ms filter | Outer loop | **Doubtful.** Copied from XR. Fits the 9 Hz mode. |
| Bus model: 10 ms filter of Vdc, capped at OP_VOLTAGE | Stops a regen spike from releasing FW | **Works** as a safeguard (a 90 V event in `first.trc`). `OP_VOLTAGE` 51 is **doubtful**: it is above the supply. |
| New divider table | Correct Vdc reading on this board | **Works** for this board. **Remove before vehicle:** it also rewrote the old XR/HiLoad entry, which would read 30–40 % high. |
| Overspeed trip 10800, `SPORTS_MAX_SPEED` 10500 | Bench safety and target | **Not tested yet** (never fired). Both are above the 10000 rpm datasheet maximum. |
| 10 Hz telemetry (`CAN_BUS_CYCLE` 100 ms) | Makes analysis possible | **Works.** **Remove before vehicle:** 5× load on vehicles. |
| 200 Hz fast telemetry, CAN TX queue fix, Parser2, GUI panel | See the 9 Hz mode; stop CAN stalls | **Not tested yet** on a real bus or log. Commit only after a log parses. |
| ECO→SPORTS hack, speed-only mode | Bench control | **Remove before vehicle** (the ECO→SPORTS hack is ungated). |

Housekeeping: H1 to H10 are **hand-edits** in generated code. Regenerating from the Simulink model removes them silently. `fdcan.c` is a CubeMX file and has the same problem.

---

## 4. What the logs proved so far

- **The current-sense lag is a pure delay of about 205–210 µs.** 68 holds were checked, and an RC (first-order) fit did not fit. That is why a simple angle shift is the right form of fix. Today's three holds agree at 205–208 µs.
- **λ is about 10.7 mWb, not 11.21.** Two independent methods gave 10.4–10.8. Pole pairs = 5 are confirmed within about 3 %.
- **Closed-loop FW is the one change proven** against run-to-run spread. Spread is large: speed σ was 5.9 vs 14.8 rpm for the same config at 8000 rpm. A single run showing improvement is not proof.
- **τ works, and its sign and size are right** (today's sweep). The real operating point did not move. Only the reading did.
- **Overshoot is dangerous.** τ = 250 µs produced a runaway and a trip.
- Not proven: VREF 85, derate 15/15, the circle limit, the bus-model cap's effect on oscillations. Each is a single run.

---

## 5. What is next, in order

Change one thing per flash and score it against `tw7-baseline-1028`.

1. **τ = 200 µs staircase, 6000 → 9000 rpm, fast telemetry ON.** Watch for the runaway: if rpm goes above the reference by about 100 while Iq_ref is 0, **STOP**. Then re-fit the 85 µs delay, λ and Ld.
2. **Speed PI 2/4 vs 4/8.** Aims at the 9 Hz mode.
3. **`FW_CL_KI` 100 vs 200.** Aims at the 3.8 Hz alternation.
4. **Higher supply, 54–58 V, with `OP_VOLTAGE` matched.** Probably needed for 10500 rpm.
5. **Open decision:** whether the speed loop may command braking. It would remove the runaway but needs somewhere for the energy to go.

Later, once things are stable: VREF 90 vs 85, derate 0/0 vs 15/15, and λ = 0.0107. Before anything goes toward master or a vehicle, work through the "remove or fix" list in `TW7_BRINGUP_CHANGES.md`.

---

## 6. Bench cheat sheet

**Setting τ (GUI, `tools/pegasus_gui.py`, tab "Isense tau / tlm")**
- Change τ only at **3000 rpm or below**. The GUI asks you to confirm above that.
- Send the value, then **check the "Tau applied (us)" field** (it is read back from the controller). Do not assume it took.
- τ boots at **0**. It is only set over CAN, so a reset puts it back to 0. After any reset or trip, check "Tau applied" and send it again.
- Use **200 µs**. Never go near 250.

**Fast telemetry**
- The firmware boots with it **off**. Click "Fast tlm ON" before every run (200 Hz, about 60 kbit/s). The GUI shows a live rate.
- Without it you cannot see the 9 Hz mode or the start of a runaway.

**Abort rules (STOP at once)**
- rpm above reference by about 100, while Iq_ref is 0 (runaway).
- Id_ref running steadily more negative while speed rises (the same runaway, earlier).
- Any trip above about 5250 rpm: the bus will rise. Watch it.
- Ignore the "UV" label on a runaway trip; the cause is not low voltage.

**Scoring a log**
```
python tools/Parser2.py <log> --scorecard --fw-rev worktree
```
`--fw-rev worktree` reads the config from the files on disk. Use it while the flashed firmware is the working tree. Otherwise give the git revision that was flashed. A wrong rev gives wrong tuning state; Parser2 warns when it can tell. The scorecard is written next to the log as `<log>_scorecard.csv`.

**Before you start each run:** fast telemetry ON, "Tau applied" shows what you expect, supply at its set voltage, and the one change you are testing is the only difference from baseline.
