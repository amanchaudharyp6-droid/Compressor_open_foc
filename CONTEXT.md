# Pegasus v4 motor controller

Field-oriented control firmware for permanent-magnet traction motors. This glossary pins the terms used when reading telemetry and tuning the current and field-weakening loops.

## Voltage headroom

**Voltage circle**:
The largest dq voltage magnitude the inverter can produce without over-modulation: the DC link voltage over √3.
_Avoid_: SVM limit, Vmax (on its own)

**Modulation index**:
The magnitude of the dq voltage pair divided by the voltage-circle radius; 1.00 is on the circle. Telemetry reports the peak over each frame window.
_Avoid_: Mod index for V_s/Vdc

**Demanded modulation index**:
The modulation index of the voltage pair the current regulators asked for, before the circle limit. It can exceed 1.00, and how far it does shows how short of voltage the regulators are.

**Applied modulation index**:
The modulation index of the voltage pair actually sent to the modulator, after the circle limit. It never exceeds 1.00 when the limit is enabled.

**Clip ratio**:
The share of control steps in a telemetry window in which the circle limit shortened the demanded voltage pair.
_Avoid_: Saturation count, duty of limit

**d-axis priority**:
The rule for shortening a demanded pair that leaves the voltage circle: Vd is kept and Vq gets whatever voltage is left.

## Current measurement

**Current-sense lag**:
The time between the phase currents existing in the motor and the controller using them. Above base speed it shows up as a rotation of the measured dq current by electrical speed × lag.
_Avoid_: ADC delay (the ADC conversion itself is a few µs)

**Sense-filter loss**:
The drop in measured current amplitude caused by the RC filter in the sense chain: 1/√(1 + (ωe·RC)²), or about 19 % at 10500 rpm. τ can't correct it, because τ only rotates the current.

**Stale phase sample**:
The phase-V reading used by the Park transform is one PWM period older than phase U, because the ADC interrupt fires after the first injected conversion. It shows as a ripple at twice the electrical frequency in Id/Iq, plus extra lag.

**True current**:
The motor's actual id and iq, recovered from the logged values by undoing τ, the sense filter, the stale phase sample and the remaining sense delay. The textbook voltage equations hold for true current, not logged current.

**Measured current frame**:
The dq frame the controller's current readings are expressed in. It matches the voltage frame only when the current-sense lag is compensated.

**Voltage frame**:
The dq frame the commanded voltages are applied in, anchored to the rotor by the encoder offset and the output-delay compensation.

**Boot angle error**:
How far the controller's rotor-angle reference sits from the true rotor angle because of the start-up reading of the encoder's PWM output. It stays fixed for a whole power cycle and varies by about 3° (1σ) between power-ups, which fits a frame-length error in the PWM decode (4096 assumed, 4119 per the datasheet as read in review). It rotates the voltage frame and the measured current frame together.
_Avoid_: Encoder offset (that is the calibrated EEPROM value, which is added on top)

**Alignment shift**:
A lasting change in the angle between the encoder's reading and the rotor's magnets. Unlike a boot angle error, it survives power cycles, and an autotune re-zeroes it without undoing it.
_Avoid_: Encoder fault (the encoder's signals can be healthy while this happens)

**Back-EMF angle**:
The angle of the dq voltage left after subtracting the modelled resistive and inductive drops from Vd/Vq; 0° in a perfect frame. It doesn't depend on flux linkage, so comparing it at the same speed across power-ups measures the boot angle error.

**Negative-sequence error**:
A part of the measured phase current that rotates against the rotor. In the dq frame it shows up as a ripple at twice the electrical frequency.

## High-speed behaviour

**Iq floor**:
The lowest Iq_ref the speed loop can ask for: 0 A on the bench, because the supply can't sink current. At the floor the speed loop has nothing left to slow the rotor.

**Runaway margin**:
How far the frame can shift before Iq_ref reaches the Iq floor: about 20° electrical at 10500 rpm with τ 130. Today it comes mostly from the uncorrected measurement error, so it shrinks when the measurement is fixed.

**Complex-vector PI**:
A current PI with a speed-scaled cross term in each integrator, which cancels the plant's d-q cross-coupling so tracking no longer depends on the feed-forward.
_Avoid_: Decoupling (on its own; that usually means the feed-forward terms)

**Uncontrolled generation**:
The motor charging the DC link through the inverter's diodes, because its line back-EMF peak is above the link voltage while the switches are off. On TW-7 (measured flux linkage about 10.7 mWb) at 49 V this starts at about 5250 rpm and reaches about 102 V at 10500 rpm.
_Avoid_: Regen (regen is controlled braking current)

**Reference alternation**:
Id_ref and Iq_ref swinging in opposite directions on consecutive telemetry frames. 200 Hz telemetry puts its true frequency at 3.2–3.5 Hz; at 10 Hz telemetry it looks like a frame-to-frame flip with a slow beat.
_Avoid_: Oscillation (on its own)

**Speed hunting**:
The rotor speed wandering around a constant speed reference. Two kinds appear in field weakening: the slow wander that comes with reference alternation, and a speed-loop mode at about 9 Hz that 10 Hz telemetry shows aliased as 0.3–2 Hz.
_Avoid_: Oscillation (on its own)

**Hum**:
The electrical noise heard from the motor at high speed. Not yet tied to a frequency or a cause.

## Bench testing

**Cleared (a speed)**:
The motor passes at that speed: ramp up at the bench rate, hold for 10–20 s with no trip, no reference alternation and speed σ under about 10 rpm, then ramp down to zero without a trip, twice in a row.

**Hand-edit**:
A change made directly in the C generated from the Simulink model. Regenerating the code from the model silently removes it.

## Telemetry

**V_s/Vdc**:
The model's pre-field-weakening voltage estimate, computed from the MTPA current references, over the sensed DC link. It is not a modulation index.
_Avoid_: Modulation index, Mod_Index
