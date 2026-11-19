# Pegasus ATS — FOC Motor Controller Firmware

Centre-Aligned PWM firmware for the **Pegasus ATS** electric vehicle motor controller.
Targets an **STM32H743ZITx** (Cortex-M7 @ 200 MHz, AHB 100 MHz) and implements a dual-rate
Field-Oriented Control (FOC) loop generated from a MATLAB/Simulink model.

---

## Table of Contents

1. [Getting Started](#getting-started)
2. [Project Overview](#project-overview)
3. [Hardware Specifications](#hardware-specifications)
4. [Field-Oriented Control — Brief](#field-oriented-control--brief)
5. [FOC Model — High-Level Overview](#foc-model--high-level-overview)
6. [Repository Structure](#repository-structure)
7. [Firmware Architecture](#firmware-architecture)
8. [State Machine](#state-machine)
9. [Configuration & Feature Flags](#configuration--feature-flags)
10. [Motor & Controller Parameters](#motor--controller-parameters)
11. [Multi-Motor Support](#multi-motor-support)
12. [Vehicle Variants](#vehicle-variants)
13. [Torque Management](#torque-management)
14. [Hill Hold](#hill-hold)
15. [CAN Protocol](#can-protocol)
16. [Angle Calibration & Position Sensing](#angle-calibration--position-sensing)
17. [EEPROM — Angle Calibration Persistence](#eeprom--angle-calibration-persistence)
18. [OTA & Bootloader](#ota--bootloader)
19. [Fault & Error Handling](#fault--error-handling)
20. [Building & Flashing](#building--flashing)
21. [API Documentation](#api-documentation)

---

## Getting Started

New to this branch? Read this section first — it is the shortest path from clone to a
running vehicle.

### What this branch is

`OTA_Reintegrated` carries the vehicle-integration firmware **plus** the CAN bootloader,
so the build produces three images (Bootloader + App1 + App2) rather than one. Everything
below assumes that layout. See [OTA & Bootloader](#ota--bootloader).

### 1. Check your configuration before you build

Two compile-time selections decide what firmware you get. **Both live in headers, and both
are frequently changed in working trees — always check them before building for a vehicle.**

| What | Where | Options |
|---|---|---|
| Vehicle variant | [`Core/Inc/vehicle_config.h`](Core/Inc/vehicle_config.h) | `HIRANGE_XR` / `HILOAD_XR` / `HIRANGE_TR` — set exactly one to 1 |
| Motor profile | [`Core/Inc/motor_profiles/motor_select.h`](Core/Inc/motor_profiles/motor_select.h) | `MOTOR_PROFILE_DEFAULT` (96 V, 1024 PPR) / `MOTOR_PROFILE_LIVGUARD` (86.4 V, 512 PPR) / `MOTOR_PROFILE_HIRANGE_TR` (76.8 V, 1024 PPR) |

These are guarded, not merely documented — a wrong combination fails the build rather than
producing a mis-tuned image:

- Exactly one vehicle variant must be selected, or `#error`.
- Each variant carries a full config block (envelope, torque table, expo, speed-PI, slew,
  version bytes, `RPM_TO_KMPH`). `HIRANGE_TR` is the 76.8 V platform.
- `MOTOR_PROFILE_LIVGUARD` and `HILOAD_XR` are bound **both** ways — HiLoad XR always runs the
  LivGuard motor. The reverse direction is the safety-relevant one: HiLoad XR on the 96 V default
  profile would set `BUS_DC_OV_LIMIT` to 120 on an 86.4 V pack.
- `MOTOR_PROFILE_HIRANGE_TR` is bound to `HIRANGE_TR` **both** ways — the 76.8 V profile and the
  TR variant must be selected together (each carries the 76.8 V bus / OV-UV thresholds).
- `HILL_HOLD_ENABLE` requires `THROTTLE_BASED_REF` (see [Hill Hold](#hill-hold)).

The variant also auto-selects the CAN dashboard speed factor and the firmware/config version
bytes, so there is no separate flag to forget when switching vehicles.

### 2. Build

**Use Docker.** A host `cmake` invocation will fail — the checked-in build cache points at
the container's `/work` path.

```bash
docker compose build            # once
docker compose run --rm builder # builds all three images into ./Build
```

Output in `Build/`: `Pegasus_v4-Bootloader.bin`, `Pegasus_v4-App1.bin`, `Pegasus_v4-App2.bin`
(plus `.elf` / `.hex` / `.map`). `Build.bat` additionally runs `patch_image_header.py` to
stamp the CRC and size into the App1 image header — an OTA image is **not** valid without
this step.

If you add a new `.c` file, add it to `FIRMWARE_FILES` in [`CMakeLists.txt`](CMakeLists.txt)
or it will silently not be compiled.

### 3. Flash

**First time on a board (needs SWD/ST-Link)** — all three images, at their fixed addresses:

```bash
Flash.bat        # mass-erase, then Bootloader @0x08000000, App1 @0x08040000, App2 @0x08080000
```

**Thereafter (OTA over CAN)** — PCAN adapter + `Flash_Console.exe`, App1 slot only. The
bootloader is already resident, so this is the normal iteration loop; no ST-Link needed.

> If OTA transfers run slowly, confirm `OTA_FDCAN_SETUP()` is being called — it narrows the
> CAN filters to the OTA IDs only. Without it, live vehicle traffic floods the Rx path
> during the transfer.

### 4. First boot on a new motor — angle calibration

A freshly flashed board has no valid rotor offset in EEPROM, so it runs the alignment routine
before it will drive:

- `Read_EEPROM_at_init()` finds no valid magic/CRC → `d.start_alignment = 1`
- Boot enters `ANGLE_CALIB`, the autotune routine spins the motor open-loop and measures the
  CW/CCW offsets, then stores them and resets into normal `FOC_START` operation.

To force a re-calibration later: send CAN `0x729` with RPM ≈ 0 (invalidates EEPROM page 0 and
resets), or set `FORCE_ANGLETUNE = 1`. Note that **bumping `EEPROM_MAGIC_NUM` in
[`board_config.h`](Core/Inc/board_config.h) also invalidates stored offsets** and silently
forces a re-alignment on the next boot — check it if a board unexpectedly re-calibrates.

### 5. Where to look when tuning

| Symptom / goal | Start here |
|---|---|
| Launch feel, pedal response | `TRQ_REQ_SLEW_UP_NM_S`, `THR_EXPO_*` — [Torque Management](#torque-management) |
| Torque ceiling, power, thermal derate | Torque-speed table + `SPORTS_MAX_TORQUE` in `Torque_Management.h` |
| Top speed oscillation | `SPEED_PI_KP` / `SPEED_PI_KI` |
| Battery current breach | DCLO section of [Torque Management](#torque-management) |
| Rollback on gradients | [Hill Hold](#hill-hold) |
| Whine / sag at 24–35 km/h | Field-weakening `FW_RST_FACTOR` vs `FW_F2M` / `FW_M2F` |

### Known in-flight work

| Item | State |
|---|---|
| AD2S1210 resolver (`Ad2s1210.c`, `Resolver_Position.c`) | Driver + SPI1 present and initialising, but **nothing calls `AD2S1210_Init()`** and the control loop still runs off the quadrature encoder. See [HANDOFF_AD2S1210.md](HANDOFF_AD2S1210.md). |
| Phase-correction model | Lives on `origin/Phase_correction_branch`, **not merged**. Replaces `isr_angle_calib()` with a Simulink `isr_phase_correction()`. See [Angle Calibration & Position Sensing](#angle-calibration--position-sensing). |
| Hill-hold brake handoff | Code path is complete but **dormant** — the VCU does not currently publish the brake frame, so `can_d.brake_command_active` never asserts. |
| Overboost power budget | Machinery present but **inert** (`PWR_CONT_W == PWR_BOOST_W`). |

---

## Project Overview

This firmware drives an Interior Permanent Magnet Synchronous Motor (IPMSM) for an
electric two-wheeler drivetrain. The control algorithm is:

- **MTPA** (Maximum Torque Per Ampere) — minimises copper losses at partial load
- **FWC** (Field Weakening Control) — extends operating speed beyond back-EMF limit
- **Centre-aligned PWM** (Space Vector Modulation) at 10 kHz

The core control code (`FOC_Loop.c / .h`) is auto-generated from a Simulink model
and integrated into an STM32H7 HAL/BSP layer. All tunable parameters are exposed over CAN
for live adjustment without firmware reflashing.

> **Naming note:** the generated model was originally `FOC_MTPA_FWC_FF` and is now
> `FOC_Loop` (`FOC_Loop.c/.h`, `FOC_Loop_U` / `FOC_Loop_Y`, `FOC_Loop_step0/step1`). The old
> name survives only in Simulink `hilite_system` comments inside generated headers.

**Firmware / Config Version:** selected by vehicle variant in `vehicle_config.h` —
HiRange XR: FW 0.0C.01 / Config 5.00.00 · HiLoad XR: FW 0.0D.00 / Config 2.00.00 · HiRange TR: FW 0.0B.01 / Config 2.00.00  
**Hardware Version:** 4.03.01  
**Device ID (CAN):** 0x03

---

## Hardware Specifications

| Item | Value |
|---|---|
| MCU | STM32H743ZITx (single-core Cortex-M7) |
| CPU clock | **200 MHz** (SYSCLK, D1CPRE = /1) |
| AHB / AXI (HCLK) | **100 MHz** (HPRE = /2) |
| FPU | FPv5-SP-D16 (single-precision hardware float) |
| PWM Timer | TIM1 (APB2, 100 MHz) — centre-aligned, dual-rate: ~2 kHz during current-sense calibration, ~10 kHz during FOC run. Dead-time 100 counts |
| Encoder Timer | TIM2 (APB1, 100 MHz) — quadrature encoder, counts/rev = `4 × ENCODER_PPR` (4096 on the default profile) |
| Free-running Timer | TIM5 (APB1, 100 MHz) — PSC 99 → 1 MHz, 32-bit, encoder PWM capture |
| 1 ms Tick Timer | TIM17 (APB2, 100 MHz) — PSC 39 → 2.5 MHz, ARR 2499 → 1 kHz heartbeat & deferred EEPROM writes |
| Current Sensing | ADC2 — injected conversions on PHASE_U and PHASE_V |
| Analog Sensing | ADC1 (DMA) — throttle, controller temp, motor temp, bus DC, aux DC |
| ADC kernel clock | 37.5 MHz (PLL2P) |
| CAN Bus | FDCAN2 |
| EEPROM | AT24-compatible, I2C2 address 0xA0, page size 8 bytes |
| Bus Voltage | Per motor profile — default 96 V (OV 120 V, UV 75 V); LivGuard 86.4 V (OV 105 V, UV 65 V); HiRange TR 76.8 V (OV 96 V, UV 60 V) |
| Aux Voltage | 12 V (UV limit: 9 V) |

### Clock Tree

Configured in `SystemClock_Config()` / `PeriphCommonClock_Config()` in
[`Core/Src/main.c`](Core/Src/main.c). There is **no external crystal** — everything derives
from the internal HSI:

```
HSI 64 MHz (HSI_DIV1)
   │
   ├─ PLL1  M=4 → 16 MHz ref  ·  N=25 → VCO 400 MHz (WIDE)  ·  P=2
   │     └─► SYSCLK 200 MHz ─┬─ D1CPRE /1 ─► CPU (Cortex-M7)  200 MHz
   │                          └─ HPRE   /2 ─► HCLK / AXI / AHB 100 MHz
   │                                            ├─ APB1 /2 → PCLK1  50 MHz  (timers 100 MHz)
   │                                            ├─ APB2 /1 → PCLK2 100 MHz  (timers 100 MHz)
   │                                            ├─ APB3 /1 →       100 MHz
   │                                            └─ APB4 /1 →       100 MHz
   │
   └─ PLL2  M=4 → 16 MHz ref  ·  N=9 + FRACN 3072 (×9.375) → VCO 150 MHz (MEDIUM)  ·  P=4
         └─► ADC kernel clock 37.5 MHz
```

Power/latency: LDO supply, **VOS3** voltage scaling, **FLASH_LATENCY_2**.

**Why both numbers appear.** APB1 runs at /2, so by the STM32H7 timer rule (`TIMPRE = 0`,
prescaler ≠ 1 → ×2) its timer kernel clock is back at 100 MHz — the same as APB2's. So
**every timer in this design sees 100 MHz**, which is what the `board_config.h` comments mean
by "@ 100 MHz timer clock". Only the CPU, and anything clocked from it, runs at 200 MHz.

Derived PWM rates, for checking against `board_config.h`:

| Timer | Prescaler | Counter clock | Period | Rate |
|---|---|---|---|---|
| TIM1 run | `TIM1_PSC_RUN` 1 | 50 MHz | centre-aligned, 2 × ARR 2499 | **10.0 kHz** |
| TIM1 calib | `TIM1_PSC_CALIB` 9 | 10 MHz | centre-aligned, 2 × ARR 2499 | **2.0 kHz** |
| TIM17 | `TIM17_PSC` 39 | 2.5 MHz | ARR 2499 + 1 | **1.0 kHz** |
| TIM5 | `TIM5_PSC` 99 | 1 MHz | ARR `0xFFFFFFFF` | free-running (1 µs tick) |

> **The DWT cycle counter runs at the CPU clock, i.e. 200 MHz.** `apply_torque_scaling()`
> derives its `dt` as `d_cyc / SystemCoreClock`, and on a single-core H743
> `SystemCoreClock` is the CM7 frequency (not `SystemD2Clock`), so that division is correct
> at 200 MHz without changes. If you ever hardcode a cycle-to-seconds constant, use
> 200 MHz — not the 100 MHz timer figure.

---

## Field-Oriented Control — Brief

FOC decouples the two fundamental current components of an AC motor so each can be
controlled independently with a simple PI loop — the same way a separately-excited DC
motor is controlled.

**Reference Frame Transformation**

The three-phase stator currents (Ia, Ib, Ic) are transformed into a two-axis rotating
frame that is locked to the rotor flux vector:

```
(Ia, Ib, Ic)  →  Clarke (αβ stationary)  →  Park (dq rotating)  →  (Id, Iq)
```

- **Id (flux component)** — current aligned with the rotor magnet flux. Controlling Id = 0
  in the motoring region maximises torque-per-ampere. Making Id negative reduces air-gap
  flux, enabling field weakening at high speed.
- **Iq (torque component)** — current 90° ahead of the flux. Torque ≈ Kt × Iq.

Two independent PI controllers (one for Id, one for Iq) produce voltage demands Vd and
Vq. An inverse Park + inverse Clarke transform converts these back to three-phase voltage
references, and Space Vector Modulation (SVM) generates the PWM duty cycles.

**MTPA (Maximum Torque Per Ampere)**

For an IPMSM, inductance anisotropy (Ld ≠ Lq) creates a reluctance torque component.
MTPA computes the optimal Id/Iq split so that the required torque is produced with the
least total stator current magnitude — minimising resistive losses.

**FWC (Field Weakening Control)**

Above the base speed the back-EMF reaches the inverter voltage limit. FWC injects a
controlled negative Id current to reduce the air-gap flux, allowing the motor to run faster
at the cost of reduced torque capability. The FWC loop monitors the stator voltage vector
magnitude Vs vs. the available Vmax, adjusting Id via an additional integrating controller.

---

## FOC Model — High-Level Overview

`Core/Src/FOC_Loop.c` and `Core/Inc/FOC_Loop.h` are auto-generated from
a MATLAB/Simulink multi-rate model and should not be hand-edited.

### Sample Rates

| Function | Rate | Responsibility |
|---|---|---|
| `FOC_Loop_step0()` | 100 µs (10 kHz) | Fast inner loop — Clarke/Park, PI current controllers, SVPWM output |
| `FOC_Loop_step1()` | 1 ms (1 kHz) | Outer loop — speed PI, MTPA operating point, FWC integrator, rate limiters |

Both are dispatched by `rt_OneStep()` in [`Core/Src/Helper_Functions.c`](Core/Src/Helper_Functions.c),
which owns the 10:1 rate decimation and sets `rtmSetErrorStatus(FOC_Loop_M, "Overrun")` if a
step is re-entered before the previous one finished.

### Model Inputs (`FOC_Loop_U`)

| Field | Type | Description |
|---|---|---|
| `PhaseCurrent[3]` | float | Reconstructed 3-phase currents from ADC (A) |
| `angle` | float | Electrical rotor angle from encoder (rad) |
| `Ref_Speed_mech_rpm` | float | Speed reference from CAN or throttle (RPM) |
| `ActualSpeed_mech_radsec` | float | Measured rotor speed (rad/s) |
| `Speed_1_Torque_0` | bool | Control mode: `true` = speed, `false` = torque |
| `RefTrq` | float | Torque reference when in torque mode (Nm) |
| `BusVoltage_V` | float | Measured DC bus voltage (V) |
| `Drive_State` | float | F/N/R drive direction enum |
| `Motor_Parameters` | Bus | Ld, Lq, Rs, Lambda, pole pairs, power limits |
| `MTPA_PID` | Bus | All PID gains for Speed, Id, and Iq controllers |
| `FWC_values` | Bus | Field weakening gains: Kfw, Kaw, M2F, F2M, reset factor |
| `Regen` | Bus | Regen braking parameters (speed threshold, max torque) |
| `Rate_limiter` | Bus | Ramp rates for speed reference and current references |
| `Throttle_input` | Bus | Throttle voltage, min/max, hysteresis band |
| `Id_Iq_MTPA_limit` | Bus | Hard clamping limits on Id/Iq references |
| `IIR_Filter_Coefficient` | Bus | Filter constants for current feedback filtering |
| `Is_max` | float | Maximum stator current magnitude (A) |
| `RPM_min_limit` | float | Minimum RPM change limit for rate limiting |
| `Iq_Torque_ratio` | float | Nm per A conversion factor |

### Model Outputs (`FOC_Loop_Y`)

| Field | Type | Description |
|---|---|---|
| `Va, Vb, Vc` | float | Phase voltage references for SVM (V) |
| `Vd, Vq` | float | d-q axis voltage demands after current PI |
| `Vd_PID, Vq_PID` | float | Raw d-q PI outputs (before feed-forward) |
| `V_s, V_max` | float | Stator voltage magnitude and available maximum (V) |
| `Id, Iq` | float | Measured d-q currents from Park transform (A) |
| `Id_ref_MTPA, Iq_ref_MTPA` | float | MTPA computed current references (A) |
| `Id_ref_final, Iq_ref_final` | float | Final references after FWC and limiting |
| `Id_error, Iq_error` | float | Current control error signals |
| `Speed_error` | float | Speed control error (RPM or rad/s) |
| `Iq_gen` | float | Quadrature current demanded by speed PI (A) |
| `T_gen, T_ref_FNR` | float | Generated torque estimate; F/N/R direction-adjusted torque ref |
| `Actual_torque_from_meas` | float | Torque estimated from measured Iq (Nm) |
| `Actual_torque_from_ref` | float | Torque estimated from reference Iq (Nm) |
| `Ld, Lq` | float | Operating-point inductances used by the model (H) |
| `Regen_Mode` | float | Regen braking active flag |
| `MTPA_FWC_condition` | bool | Currently in FWC mode |
| `PID_Switch` | bool | High-speed vs. low-speed PID set active |
| `Throttle_State` | ThrottleState | Current throttle state (FORWARD/REVERSE/NEUTRAL/Error) |
| `Throttle_en, Throttle_error` | bool | Throttle enable and error flags |
| `Ref_speed_out_RPM` | float | Filtered speed reference passed to drive |

### Internal Control Blocks

```
                 ┌──────────────────────────────────────────────────┐
                 │                  FOC_Loop Model                   │
                 │                                                    │
  Speed Ref ───► │  Speed PI (1ms)  ─► Iq_gen                       │
  Torque Ref ──► │                                                    │
                 │  MTPA Lookup ───► Id_ref_MTPA, Iq_ref_MTPA       │
                 │                                                    │
  Vs vs Vmax ──► │  FWC Integrator ─► ΔId (field weakening)         │
                 │                                                    │
  PhaseCurrent ► │  Clarke ─► Park ─► Id, Iq (measured)             │
  angle ────────► │                                                    │
                 │  Id PI (100µs) ──► Vd                             │
                 │  Iq PI (100µs) ──► Vq                             │
                 │                                                    │
  Vd, Vq ──────► │  Inv Park ─► Inv Clarke ─► Va, Vb, Vc            │
                 │                      (SVM inputs)                 │
                 │                                                    │
                 │  FNR State Machine, Throttle State Machine         │
                 │  Rate Limiters, Power Limiter, Regen Mode         │
                 └──────────────────────────────────────────────────┘
```

### Dual-Speed PID Sets

The model contains two sets of PID gains for the Speed, Id, and Iq controllers:

- **High-speed set** — standard gains for operation above `LOW_SPEED_TO_HIGH_SPEED_SWITCH_RPM` (400 RPM)
- **Low-speed set** — scaled by `LOW_SPEED_SCALING` (0.8×) for stable operation at low speed

The active set is selected by `PID_Switch` based on speed, current magnitude, and torque.

### Supporting Models

| File | Description |
|---|---|
| `Open_FOC0.c/.h` | Open-loop FOC used during angle calibration (no encoder feedback) |
| `Current_Ref_Estimator.c/.h` | MTPA/FWC current-reference generation, split out of the main model |
| `MCU_Protections.c/.h` | Simulink-generated protection logic (OV, UV, OC, OT monitoring) — gated by `PROTECTION_MODEL` |

---

## Repository Structure

```
pegasus-fork/
├── Core/
│   ├── Inc/
│   │   ├── main.h                      Typedefs + stitches the three config layers together
│   │   ├── board_config.h              Hardware constants: timers, ADC, EEPROM, math, protection counts
│   │   ├── vehicle_config.h            Vehicle variants, feature flags, drive modes, throttle, regen, hill hold
│   │   ├── motor_profiles/
│   │   │   ├── motor_select.h          Selects the active motor via MOTOR_PROFILE macro
│   │   │   ├── motor_default.h         Pegasus default — 96 V pack, 1024 PPR encoder
│   │   │   ├── motor_livguard.h        LivGuard — 86.4 V pack, 512 PPR encoder
│   │   │   └── motor_hirange_tr.h      HiRange TR — 76.8 V pack, 1024 PPR encoder
│   │   ├── app_types.h                 Enums: currSession, fnr_state_t, power_mode_t, errors_nums
│   │   ├── enums.h                     Bootloader/OTA shared enums (OTA_ID, image slots)
│   │   ├── FOC_Loop.h                  MATLAB-generated model structs and function prototypes
│   │   ├── FOC_Loop_types.h            Generated bus type definitions (PID_MTPABus, FWCBus, etc.)
│   │   ├── isr_handlers.h              ADC2 injected-conversion ISR paths (see Src)
│   │   ├── tim17_handlers.h            1 kHz tick handler
│   │   ├── can_rx_handlers.h           CAN Rx message dispatch
│   │   ├── fdcan.h                     CAN Tx/Rx prototypes + OTA_FDCAN_SETUP/RESTORE
│   │   ├── Helper_Functions.h          rt_OneStep, set_Initial_angle, power_mode_fnr_switch, etc.
│   │   ├── Eeprom.h                    EEPROM read/write + page 0 layout comment
│   │   ├── Torque_Management.h         Torque envelope, derating, power/speed/DCLO caps
│   │   ├── Hill_Hold.h                 Anti-rollback hold module
│   │   ├── Ad2s1210.h                  AD2S1210 resolver-to-digital converter driver
│   │   ├── Resolver_Position.h         Resolver absolute angle → TIM2 incremental tracking
│   │   ├── spi.h                       SPI1 (hspi1) for the RDC
│   │   ├── Speed_Sensing.h             Encoder-based speed calculation
│   │   ├── NTC_Temp_Reading.h          NTC thermistor temperature conversion
│   │   ├── Rate_Limiter.h              Configurable ramp limiter
│   │   ├── LPF.h                       First-order low-pass filter helper
│   │   ├── throttle_map.h              Throttle voltage → RPM/torque lookup + pedal expo
│   │   ├── adc.h / dma.h / gpio.h / i2c.h / tim.h   CubeMX per-peripheral headers
│   │   └── [Simulink runtime headers]  rt_*.h, rtw_*.h, rtGetNaN.h, etc.
│   │
│   ├── Src/
│   │   ├── main.c                      Entry point, superloop, FDCAN Rx, OTA message handling
│   │   ├── isr_handlers.c              ADC2 ISR paths: curr-sens calib, angle calib, FOC run
│   │   ├── tim17_handlers.c            1 kHz tick: fault checks, EEPROM store, reset sequencing
│   │   ├── can_rx_handlers.c           Per-ID CAN Rx handlers
│   │   ├── Helper_Functions.c          rt_OneStep dual-rate scheduler, FNR/power-mode switch
│   │   ├── FOC_Loop.c                  MATLAB-generated FOC model (step0, step1, initialize)
│   │   ├── Current_Ref_Estimator.c     Generated MTPA/FWC current-reference model
│   │   ├── Torque_Management.c         Envelope, thermal derate, power/speed/DCLO torque caps
│   │   ├── Hill_Hold.c                 Arrest → position-hold anti-rollback state machine
│   │   ├── Ad2s1210.c / Resolver_Position.c / spi.c   Resolver path (present, not yet wired in)
│   │   ├── fdcan.c / fdcan_AL.c        CAN Tx queue, Send_Data_On_CAN_4xx/7xx, OTA filters
│   │   ├── Eeprom.c                    AT24 I2C EEPROM driver + page 0 calibration layout
│   │   ├── Limit_Errors.c              Voltage, current, temperature fault checks
│   │   ├── Sensor_Disconnection_Checks.c
│   │   ├── Speed_Sensing.c             TIM2 encoder → RPM, rad/s
│   │   ├── NTC_Temp_Reading.c          NTC ADC → Celsius lookup
│   │   ├── Rate_Limiter.c / LPF.c      Ramp limiter, low-pass filter
│   │   ├── Open_FOC0.c                 Open-loop FOC for angle calibration
│   │   ├── MCU_Protections.c           Protection model step function
│   │   ├── throttle_map.c              Throttle mapping tables + pedal expo
│   │   ├── adc.c / dma.c / gpio.c / i2c.c / tim.c   CubeMX per-peripheral init
│   │   └── [HAL/startup files]         stm32h7xx_it.c, hal_msp.c, syscalls.c, etc.
│   │
│   ├── Bootloader/                     CAN bootloader (BootloaderMain.c, BootloaderFunctions.c)
│   └── Libs/
│       └── libAngle_Offset_101.a       Precompiled angle autotune library
│
├── Drivers/                            STM32H7 HAL + CMSIS drivers (unmodified)
├── LinkerScripts/
│   ├── MemoryMap.ld                    Flash/RAM regions shared by all three images
│   ├── Bootloader.ld                   Bootloader @ 0x08000000 (100 K)
│   ├── App1.ld                         App slot 1 @ 0x08040000 (256 K)
│   └── App2.ld                         App slot 2 @ 0x08080000 (256 K)
├── docker/                             build.sh, flash.sh, udev rules for the container
├── Dockerfile / docker-compose.yml     Pinned toolchain (see DOCKER.md)
├── cmake/arm-none-eabi.cmake           Toolchain file
├── CMakeLists.txt                      Builds Bootloader + App1 + App2
├── patch_image_header.py               Stamps CRC + size into the OTA image header
├── Build.bat / Build.sh                Windows / Linux build scripts
├── Flash.bat                           SWD flash of all three images (OpenOCD)
├── HANDOFF_AD2S1210.md                 Resolver port state + next steps
├── DOCKER.md                           Container build guide
└── README.md                           This file
```

---

## Firmware Architecture

### Execution Contexts

```
┌─────────────────────────────────────────────────────────────────┐
│  MAIN LOOP (background, non-real-time)                          │
│  ─ EEPROM deferred writes (eeprom_write_pending flag)           │
│  ─ CAN autotune reset request handler                           │
│  ─ FNR / power-mode switch (power_mode_fnr_switch)              │
│  ─ ADC1 analog value update (Vdc, temps, throttle)              │
│  ─ Throttle map → torque request → torqueRate request limiter   │
│  ─ apply_torque_scaling() → writes d.torque_final               │
│  ─ Speed reference rate-limiting (CAN-based or throttle-based)  │
│  ─ CAN Tx queue drain (500 ms cycle, 2 ms slots)                │
│  ─ OTA image transfer / apply when an OTA session is active     │
└─────────────────────────────────────────────────────────────────┘
       ▲ signals via volatile flags
┌─────────────────────────────────────────────────────────────────┐
│  TIM17 ISR — 1 kHz (tim17_handlers.c)                           │
│  ─ Fault checks (voltage, temperature, encoder — 1 ms cadence)  │
│  ─ EEPROM offset store (signals main loop via pending flags)     │
│  ─ Drive reset sequencing                                        │
└─────────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────────┐
│  ADC2 Injected Conversion ISR (TIM1 update event)               │
│  Dispatched by cS in main.c → isr_handlers.c:                   │
│  ─ isr_curr_sens_calib()  CURR_SENS_CALIB, ~500 µs (2 kHz)      │
│  ─ isr_angle_calib()      ANGLE_CALIB — Open_FOC0 + autotune    │
│  ─ isr_foc_start()        FOC_START, 100 µs (10 kHz)            │
│      ├─ HillHold_Apply(d.torque_final) → RefTrq floor           │
│      └─ rt_OneStep() → FOC_Loop_step0 / step1                   │
│          ├─ step0 (every ISR): Clarke, Park, Id/Iq PI, inv, SVM │
│          └─ step1 (every 10th): speed PI, MTPA, FWC, limiters   │
│  (TIM1 switches 2 kHz → 10 kHz once current offsets are done)   │
└─────────────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────────────┐
│  FDCAN Rx ISR — HAL_FDCAN_RxFifo0Callback                       │
│  ─ Vehicle/debug frames → can_rx_handlers.c                     │
│  ─ OTA frames 0x6FA / 0x7FE / 0x7FF handled in main.c           │
└─────────────────────────────────────────────────────────────────┘
```

**ISR/background data ownership.** The superloop writes `d.torque_final` (the driver's torque
demand); the FOC ISR reads it and writes `FOC_Loop_U.RefTrq`. That split is load-bearing —
`Hill_Hold` must read the demand from a variable it does not itself write, or its `max()`
floor becomes a torque ratchet. See [Hill Hold](#hill-hold).

### Boot Sequence

```
Power-on reset
    │
    ▼
Bootloader @0x08000000 ─ validates the active app image header (magic/CRC/size),
    │                     then jumps to App1 @0x08040000 (or App2)
    ▼
HAL_Init + clock config (HSI → PLL1, CPU 200 MHz / AHB 100 MHz, FPU on)
    │
    ▼
Peripheral init (GPIO, DMA, TIM1/2/4/5/17, ADC1/2, FDCAN2, I2C2, SPI1)
    │
    ▼
Stop all TIM1 PWM channels (safe state)
    │
    ▼
Read_EEPROM_at_init()
    ├── Valid page 0 found? → load cw/ccw offsets, set start_alignment=0
    └── Invalid / corrupt? → start_alignment=1 (trigger angle calib on next boot)
    │
    ▼
set_Initial_angle()  ─ read encoder PWM, compute initial angle
    │
    ▼
Model init: FOC_Loop_initialize(), Current_Ref_Estimator_initialize(),
            Open_FOC0_initialize()  ─ set all model inputs to defaults
    │
    ▼
Send heartbeat CAN frame (0x400)
    │
    ▼
Start ADC injected conversion → ISR takes over
    │
    ▼
Main loop (background tasks)
```

---

## State Machine

The `currSession cS` variable drives the top-level state machine:

```
INIT
  │  (after set_Initial_angle completes)
  ▼
CURR_SENS_CALIB ─── accumulate 4096 ADC samples ──► compute ia/ib offsets
  │
  ├── ENABLE_FAULTS path:
  │     Initial_Fault_Check() → OK → HAL_GPIO_WritePin(GATE_EN)
  │          ├── start_alignment==1 → ANGLE_CALIB
  │          └── otherwise          → FOC_START
  │
  └── DISABLE_FAULTS path:
        ├── start_alignment==1 || FORCE_ANGLETUNE → ANGLE_CALIB
        └── otherwise                              → FOC_START

ANGLE_CALIB ─── AngleAutotuningRoutine (Open_FOC0) runs every AUTOTUNE_UPDATE_MS
  │   AutotuneComplete → d.offset_angle_mech_cw/ccw computed
  ▼
ANGLE_OFFSET_STORE ─── eeprom_write_pending=1 (main loop performs I2C write)
  ▼
ANGLE_CALIB_DONE ─── (write confirmed) → HAL_NVIC_SystemReset()

FOC_START ──► running: rt_OneStep(), CAN Tx, FNR switching
  ├── CAN 0x111 reset (RPM≈0) → RESET_STATE → FOC disabled
  ├── CAN 0x729 autotune req (RPM≈0) → EEPROM_Invalidate_Page0 → reset
  └── Fault detected → CONT_ERROR → drive disabled

CONT_ERROR ─── drive outputs off, error broadcast on CAN
RESET_STATE ─── drive disabled, MCU reset follows
```

---

## Configuration & Feature Flags

Configuration lives across three layers that `main.h` stitches together:

- [`Core/Inc/board_config.h`](Core/Inc/board_config.h) — hardware/MCU constants (timers, ADC scales, EEPROM, math, controller-scope protection).
- [`Core/Inc/motor_profiles/`](Core/Inc/motor_profiles/) — motor-specific values (see [Multi-Motor Support](#multi-motor-support)).
- [`Core/Inc/vehicle_config.h`](Core/Inc/vehicle_config.h) — vehicle variant, feature flags, drive modes, throttle, regen, hill hold (below).

Change and recompile to reconfigure. Include order matters: `motor_select.h` is included
just before `vehicle_config.h` so the motor/vehicle pairing guard can see `MOTOR_PROFILE`.

### Control Mode Flags

Active configuration is **vehicle integration mode** (`VH_CAN_ID=1`, throttle-based references). Toggle `DEBUG_CAN_ID` / `VH_CAN_ID` and the `CAN_BASED_*` / `THROTTLE_BASED_*` flags to switch between bench-tuning and vehicle modes.

| Flag | Default | Description |
|---|---|---|
| `CAN_BASED_REF` | 0 | Speed/torque reference from CAN (0x102) |
| `THROTTLE_BASED_REF` | 1 | Speed/torque reference from ADC throttle |
| `CAN_BASED_DIR` | 0 | F/N/R direction from CAN (0x108) |
| `THROTTLE_BASED_DIR` | 1 | F/N/R from physical pins |
| `CAN_BASED_THRV` | 0 | Throttle voltage from CAN (0x110) |
| `THR_BASED_THRV` | 1 | Throttle voltage from ADC |

### Safety / Debug Flags

| Flag | Default | Description |
|---|---|---|
| `ENABLE_FAULTS` | 1 | Enable all fault checks before motor start |
| `DISABLE_FAULTS` | 0 | Skip fault pre-checks (bench testing) |
| `ENABLE_ENC_ERRORS` | 1 | Enable encoder signal error detection |
| `FORCE_ANGLETUNE` | 0 | Always run angle calibration on boot |
| `PROTECTION_MODEL` | 0 | Enable Simulink MCU protection model |
| `EST_CYC_CNT` | 1 | Measure ISR cycle count via DWT |
| `HILL_HOLD_ENABLE` | 1 | Anti-rollback hold — requires `THROTTLE_BASED_REF` (build fails otherwise) |

### CAN ID Profile Flags

| Flag | Default | Description |
|---|---|---|
| `DEBUG_CAN_ID` | 0 | Use 0x10x Rx / 0x40x Tx CAN IDs (PC tuning tool) |
| `VH_CAN_ID` | 1 | Use 0x109, 0x110, 0x12A Rx / 0x70x Tx IDs (vehicle integration) |

---

## Motor & Controller Parameters

Per-motor tunables live in [`Core/Inc/motor_profiles/motor_*.h`](Core/Inc/motor_profiles/); the values below are for the **default** profile. See [Multi-Motor Support](#multi-motor-support) for the LivGuard profile or to add a new motor.

### Motor Nameplate

| Parameter | Symbol | Value |
|---|---|---|
| Stator Resistance | Rs | 0.01047 Ω |
| d-axis Inductance | Ld | 148 µH |
| q-axis Inductance | Lq | 386 µH |
| PM Flux Linkage | λ | 0.0263 Wb |
| Pole Pairs | P | 3 |
| Torque Constant | Kt | ≈ 0.1473 Nm/A (= 80 Nm / (384 × √2 A)) |

### Electrical Limits

| Parameter | Value |
|---|---|
| Peak Phase Current (board protection) | 550 A |
| Motor Peak Current | 384 A RMS (543 A peak) |
| SVM Voltage Limit | ≈ 33.49 V (= OP_VOLTAGE / √3 = 58 / √3) |
| Operating Bus Voltage | 58 V |
| OV Trip | 80 V |
| UV Trip | 40 V |
| Motor OT Limit | 120 °C |
| Controller OT Limit | 80 °C |

### Speed & Torque Limits

| Parameter | ECO | SPORTS | REV |
|---|---|---|---|
| Max Speed | 4000 RPM | 5000 RPM | 1500 RPM |
| Max Torque | 72 Nm | 72 Nm | 72 Nm |
| Power Limit | 5 kW | 6.5 kW | — |

### Default PID Gains

| Loop | Kp | Ki | Kd | Back-Kaw |
|---|---|---|---|---|
| Speed | 4.0 | 8.0 | 1e-5 | √2 |
| Id (flux) | 0.4 | 6.0 | 1e-5 | 3.873 |
| Iq (torque) | 0.3 | 7.0 | 1e-5 | 4.830 |

### Field Weakening Parameters

| Parameter | Value |
|---|---|
| FW_KFW | 0.001 |
| FW_KAW | 10.0 |
| FW_M2F (motoring→FW threshold) | 0.7 |
| FW_F2M (FW→motoring hysteresis) | 0.5 |
| FW_RST_FACTOR | 0.7 |

---

## Multi-Motor Support

A single branch supports multiple PMSM motors via compile-time profile selection. The FOC Simulink model is motor-agnostic; only parameters differ per motor.

### Switching Profiles

Active profile is chosen in [`Core/Inc/motor_profiles/motor_select.h`](Core/Inc/motor_profiles/motor_select.h):

```c
#define MOTOR_PROFILE_DEFAULT     0   // Pegasus default (96 V pack, 1024 PPR)
#define MOTOR_PROFILE_LIVGUARD    1   // LivGuard (86.4 V pack, 512 PPR)
#define MOTOR_PROFILE_HIRANGE_TR  2   // HiRange TR (76.8 V pack, 1024 PPR)

#ifndef MOTOR_PROFILE
#define MOTOR_PROFILE   MOTOR_PROFILE_DEFAULT
#endif
```

Override from the build system instead of editing the header: `-DMOTOR_PROFILE=MOTOR_PROFILE_LIVGUARD`.

### What a Profile Carries

Everything that legitimately changes per motor:

| Category | Macros |
|---|---|
| Electrical | `RS`, `LD`, `LQ`, `LAMBDA`, `POLEPAIRS` |
| Current & torque | `MOTOR_PEAK_ARMS`, `CURR_TORQUE_RATIO` |
| Pack voltage | `OP_VOLTAGE` (derives `SVM_VOLTAGE_LIMIT`), `BUS_DC_OV_LIMIT`, `BUS_DC_UV_LIMIT` |
| Motor thermal | `MOTOR_TEMP_OT_LIMIT`, `MTR_NTC_R25` |
| Encoder | `ENCODER_PPR` (derives `TIM2_ARR` and `COUNTS_TO_RADS`) |
| Speed envelope | `SPEED_REF_RPM_MAX`, `MIN_RPM_FOR_MOTOR_START` |
| Alignment | `ALIGN_DUTY`, `ALIGNMENT_PULSE_MS`, `ALIGNMENT_REP_COUNT`, offset seeds |
| FOC tuning | Speed/Id/Iq PID gains + back-Kaw, FW_* gains, LOW_SPEED_* regime switch |

Controller-scope protection (`MAX_PHASE_CURRENT`, `CONTRL_TEMP_OT_LIMIT`, …) stays in `board_config.h` — same controller hardware regardless of motor.

### Adding a New Motor

1. Copy `motor_default.h` to `motor_<name>.h` and update every value.
2. Add a new ID constant and dispatch branch in `motor_select.h`.
3. Select via `MOTOR_PROFILE=<id>` and rebuild.
4. On first boot the alignment routine writes new offsets to EEPROM; no firmware change needed to re-align.

---

## Vehicle Variants

Orthogonal to the motor profile: the **motor profile** describes the machine, the **vehicle
variant** describes the platform it is bolted into. Selected in
[`Core/Inc/vehicle_config.h`](Core/Inc/vehicle_config.h) by setting exactly one flag to 1:

```c
#define HIRANGE_XR          1
#define HILOAD_XR           0
#define HIRANGE_TR          0    /* 76.8 V platform — needs MOTOR_PROFILE_HIRANGE_TR */
```

**HiRange TR (76.8 V)** shares the HiRange chassis and motor with XR but runs the 76.8 V pack
via its own `MOTOR_PROFILE_HIRANGE_TR`. It differs from XR in three deliberate ways: a **flipped
motor direction** (`MOTOR_DIRECTION = DIR_NORMAL` with `REGEN_POLARITY = false`, the coupled pair
enforced by the direction/regen guard), a **72 Nm launch on the 5.655 kW power line** (holding
back from the motor's 80 Nm using the 384 Arms head-room), and a **regen-fade band shifted down**
to 2240–2800 RPM for the lower-voltage field-weakening knee.

### What the variant selects

| Category | Notes |
|---|---|
| Firmware / config version bytes | Reported on CAN; HiRange XR = FW 0.0C.01 / Cfg 5.00.00, HiLoad XR = FW 0.0D.00 / Cfg 2.00.00, HiRange TR = FW 0.0B.01 / Cfg 2.00.00 |
| Drive-mode envelopes | `ECO_MAX_*` / `SPORTS_MAX_*` torque, power, speed |
| Torque-speed table | 14-point piecewise-linear curve + its derated twin |
| Pedal expo | `THR_EXPO_ECO/SPORTS/REV` |
| Speed-limiter PI gains | `SPEED_PI_KP` / `SPEED_PI_KI` |
| Slew rates | `TRQ_REQ_SLEW_*`, `TRQ_SLEW_ACCEL/DECEL_*` |
| Dashboard speed factor | `RPM_TO_KMPH` picked automatically — HiRange XR/TR 0.01044, HiLoad 0.01072. **No manual flag**, so it cannot go stale when switching vehicles. |

### Guards

The variant machinery is enforced at compile time rather than by convention:

| Guard | Effect |
|---|---|
| `(HIRANGE_XR + HILOAD_XR + HIRANGE_TR) != 1` | `#error` — exactly one variant |
| `MOTOR_PROFILE_LIVGUARD` without `HILOAD_XR`, or `HILOAD_XR` with a different profile | `#error` — bound both ways; HiLoad XR always runs LivGuard, and the 96 V profile would put the OV trip 15 V too high on its 86.4 V pack |
| `MOTOR_PROFILE_HIRANGE_TR` without `HIRANGE_TR`, or `HIRANGE_TR` with a different profile | `#error` — the 76.8 V TR profile and the TR variant are bound both ways |
| `MOTOR_DIRECTION` / `REGEN_POLARITY` mismatch | `#error` — must move together (`DIR_FLIP`↔`true`, `DIR_NORMAL`↔`false`) |
| `HILL_HOLD_ENABLE && !THROTTLE_BASED_REF` | `#error` — hill hold needs `d.torque_final` as its driver-demand source |
| `TRQ_REQ_SLEW_UP > TRQ_SLEW_ACCEL_*` | `_Static_assert` — request limiter must stay the binding one |
| `HH_MAX_TORQUE > MAX_TORQUE` | `_Static_assert` |

---

## Torque Management

`Core/Src/Torque_Management.c` / `Core/Inc/Torque_Management.h` implement all torque scaling between the throttle map output and the FOC model's `RefTrq` input. Called from the main loop at while-loop cadence with a measured `dt`.

**Vehicle variants:** drive-mode envelopes, the torque-speed table, pedal expo, speed-PI gains, slew rates, and the `torqueRate` request limiter are all selected per variant by exactly one of `HIRANGE_XR` / `HILOAD_XR` / `HIRANGE_TR` in `vehicle_config.h` (compile-time guarded). Values below are for the reference **HiRange XR** variant.

Upstream of this pipeline, `throttle_map.c` maps pedal voltage → torque request with a per-mode **pedal expo** (`THR_EXPO_*` in `vehicle_config.h`): the normalized pedal `x` is shaped as `(1-k)·x + k·x²`, so `k = 0` is linear and `k = 1` full quadratic. Full pedal is unchanged at any `k` — only partial-pedal sensitivity is reshaped. The request then passes through the `torqueRate` **request limiter** in `main.c` (`TRQ_REQ_SLEW_*`, XR: 25 Nm/s up) — **this is the launch-feel knob**: it is the binding limiter for driver transients and runs on the honest ISR-tick timebase. It tracks through brake-command cuts (target 0) so its state never goes stale, and `Rate_Limiter.c` clamps a resumed path's dt to 100 ms so a paused limiter cannot jump.

### Pipeline (order of operations inside `apply_torque_scaling`)

1. **Torque-speed envelope** — 14-point piecewise-linear curve per variant caps torque based on current speed. For HiRange XR the table peaks at 40 Nm and tracks the 5.655 kW power line from ~2500 RPM. A parallel *derated* envelope at reduced values is used during thermal derating.
2. **Temperature derating (per-sensor latches, worst case wins)** — IIR-filtered controller and motor temperatures each carry an independent latch with release hysteresis; both blend factors are evaluated every cycle and the deeper derate takes both the envelope and `Pmax`, so one sensor's latch can never mask the other overheating. Latches drive the CAN derate flags.
3. **Mode torque scale + hard clamp** — Multiplies the envelope by `TORQUE_SCALE_ECO` (0.4) or `TORQUE_SCALE_SPORTS` (1.0) then hard-clamps to `ECO_MAX_TORQUE` (16 Nm) / `SPORTS_MAX_TORQUE` (40 Nm, equal to the XR table peak — the street-safe launch limit; steady-state launch torque, not transient shaping, was what lifted a wheel on gradients).
4. **Power cap** — `Pmax_eff / ω_mech` limits the torque cap (`SPORTS_MAX_POWER` 5655 W / `ECO_MAX_POWER` 2262 W for XR). The overboost budget machinery is **currently inert** (`PWR_CONT_W == PWR_BOOST_W == SPORTS_MAX_POWER`); see the header comment for how to re-enable it. Temperature derating reduces `Pmax_eff` on top and remains the hard thermal backstop.
5. **Speed limit (PI + anti-windup)** — A PI controller on the speed error (`vmax - speed`) outputs a torque cap. Its integrator converges to the steady torque that holds the vehicle at `vmax` (e.g. coasting down a grade), eliminating the cut-to-zero / slow-recovery limit cycle the old open-loop taper produced. Output is floored at 0 Nm (regen is owned by the FOC model); conditional-integration anti-windup keeps it dormant below the limit and prevents wind-down past zero on steep grades.
6. **DCLO (closed-loop DC current limit)** — The battery current is estimated at loop rate from local FOC quantities (`I_dc ≈ 1.5(Vd·Id + Vq·Iq) / (η̂·Vdc)`, ADC-measured bus voltage) and regulated against the BMS-published discharge limit (`0x12A`) by a feedforward power cap plus a PI torque cap (regulation target 98% of DCLO). The calibration scalar `η̂` is adapted each `0x110` frame — from steady windows only, since real load transients would pair the instantaneous estimate against an up-to-one-frame-old measurement — so the estimate tracks the BMS-measured battery current. A tighten-only backstop trim on the raw measurement guards 100% of DCLO in case the estimator faults; it arms only on a sustained breach (3 consecutive frames), is capped at 20% of DCLO, and releases on a fixed 2 s time constant. If the BMS goes stale (> 2 s) the limit falls back to `min(last, DCLO_DEFAULT)` and calibration freezes. Below `SOC_BREAK` (10%) an absolute guard subtracts up to 10 A from the DCLO budget.
7. **Torque slew rate** — Protection-release smoothing on the final command (XR: 80 Nm/s both modes; HiLoad: 40/80; 100 Nm/s deceleration), scaled by measured `dt` (DWT cycle counter). Launch feel is owned by the upstream request limiter (see above); this slew only shapes how fast torque *returns* when a cap releases (speed-PI after a crest, DCLO, temp derate). A static assert enforces `TRQ_REQ_SLEW_UP ≤ TRQ_SLEW_ACCEL_*` so the request limiter stays the binding one. The `dt` floor at 100 µs can inflate this slew's effective rate on a fast superloop, but since it's non-binding for feel, that only makes recovery slightly quicker. Innermost, the FOC model applies a final `Torque_gen` ramp (`FOC_TRQ_RAMP_*` in the motor profile: HiRange XR 150 Nm/s up / 150 down at 10 kHz) as a pure guard — asserted (for whichever variant is active) to be the fastest layer in both directions so it never binds over the layers above; it also smooths hill-hold / FNR / regen steps that enter below the superloop.

### Key Parameters

| Parameter | Value | Description |
|---|---|---|
Values shown are the active **HiRange XR** variant (see `vehicle_config.h` for HiLoad XR).

| Parameter | Value | Description |
|---|---|---|
| `TORQUE_SCALE_ECO/SPORTS` | 0.4 / 1.0 | Envelope multiplier before mode clamp |
| `SPORTS_MAX_TORQUE` | 40 Nm | SPORTS launch clamp = XR table peak (street-safe launch limit) |
| `SPEED_PI_KP` | 0.05 | Speed-limiter proportional gain (Nm/RPM) — vehicle-validated |
| `SPEED_PI_KI` | 0.25 | Speed-limiter integral gain (Nm/(RPM·s)) |
| `MOTOR_TEMP_DERATE_START/END` | 130 / 150 °C | Motor derate range |
| `CTRL_TEMP_DERATE_START/END` | 75 / 80 °C | Controller derate range |
| `MIN_POWER_AT_FULL_DERATE_W` | 2000 W | Pmax floor at full derate |
| `PWR_BOOST_W` / `PWR_CONT_W` | = `SPORTS_MAX_POWER` | Overboost currently inert (see header comment to re-enable) |
| `TRQ_REQ_SLEW_UP_NM_S` | 25 Nm/s | **Launch-feel knob** — request limiter, honest ISR timebase (vehicle_config.h); 0→40 Nm in 1.6 s |
| `TRQ_SLEW_ACCEL_ECO/SPORTS_NM_S` | 80 / 80 | Command slew — protection-release smoothing only (must stay > request rate) |
| `TRQ_SLEW_DECEL_NM_S` | 100 Nm/s | Max torque ramp-down rate |
| `THR_EXPO_ECO/SPORTS/REV` | 0.0 / 0.95 / 0.95 | Pedal expo (0 = linear, 1 = quadratic); partial-pedal shaping only, WOT unchanged |
| `DCLO_DEFAULT` | 100 A | Boot / stale-BMS DC current limit |
| `DCLO_PI_MARGIN` | 0.02 | DCLO regulation target = 98% of the limit |
| `DCLO_PI_KP` / `DCLO_PI_KI` | 0.3 / 4.0 | DCLO PI gains (Nm per A / Nm per A·s) |
| `DCLO_ETA_INIT` | 0.95 | I_dc estimator calibration seed (adapts from 0x110) |
| `DCLO_BMS_TIMEOUT_MS` | 2000 ms | 0x110/0x12A staleness fallback |
| `SOC_BREAK` | 0.10 | SOC threshold for absolute DCLO guard |

---

## Hill Hold

`Core/Src/Hill_Hold.c` / `Core/Inc/Hill_Hold.h` hold the vehicle against rollback on a
gradient. `HillHold_Apply()` runs from `isr_foc_start()` at 10 kHz, decimated internally to
1 kHz, and returns a torque **floor** — it can only raise the driver's demand, never lower it.

### Design

A single speed PI on a **0 RPM setpoint**, gated by a state machine. Speed is taken in the
"drive frame" — `v_drive = drive_sign · d.RPM`, positive when the vehicle moves the way the
engaged direction wants — so one sign convention covers Forward and Reverse and the
`MOTOR_DIRECTION` flip needs no handling.

```
                 rollback past HH_ENTRY_RPM, brake released
  HH_INACTIVE ─────────────────────────────────────────────────► HH_ACTIVE
   (armed)                                                    (PI holds v_drive
       ▲                                                       at 0 RPM:
       │ rollback stopped                                      HH_KP·err + integ)
       │                                                          │        │
  HH_COOLDOWN ◄─── torque reached 0 ─── HH_RELEASING ◄────────────┘        │
   (spent; re-arm                        (bleed at            HH_MAX_HOLD_MS
    on driver action)                  HH_RELEASE_RATE_NM_S)      spent    │
                                                                           │
                        driver takeover: v_drive > HH_EXIT_RPM             │
                        AND driver demand > hold torque                    │
                                            ┌──────────────────────────────┘
                                            ▼
                                       HH_STANDBY ──── rollback past ────► HH_ACTIVE
                            (output bleeds to 0, grade   HH_ENTRY_RPM
                             estimate retained, decays
                             at HH_STANDBY_DECAY_NM_S)
```

Brake, neutral, FNR change, or a switch out of torque mode resets to `HH_INACTIVE` from any
state.

The integrator is the module's estimate of **the torque this gradient needs**, and the loop
lives or dies on protecting it:

- **Wind up** only while genuinely rolling back (`v_drive < -HH_DEADBAND_RPM`).
- **Freeze** inside the standstill deadband. At zero speed the error is zero for *any*
  torque, so an integrator left running has nothing to converge on and pumps straight past
  the holding value. Freezing lets it settle at whatever arrested the rollback.
- **Unwind** at `HH_UNWIND_GAIN`× when forward motion is the *hold's own* surplus, so an
  overshoot self-corrects instead of creeping uphill — but **only while the hold still owns
  the torque**. Forward motion under the driver's throttle says nothing about the hold being
  too strong, and dumping the estimate there is what turned a throttle blip into a violent
  rollback/catch limit cycle once the driver lifted again.
- **Clamp** to `[0, HH_MAX_TORQUE]` — push-only, and the clamp doubles as anti-windup.

The P term stays on the raw error, so with a 0 RPM setpoint `Kp·err` is `−Kp·v` — pure
damping in both directions, including inside the deadband where the integrator is frozen.

Every state routes its output through a **slew limiter** (`HH_SLEW_NM_S`). A state edge must
never step the torque reference: entering the hold otherwise applies `Kp·HH_ENTRY_RPM` in one
1 kHz tick and leaving it drops the whole hold torque at once, and the driveline takes either
as a shock and rings on the lash.

> **Known limitation.** Because the integrator is frozen at standstill, the held torque is
> whatever the arrest converged to rather than a uniquely determined value. A two-phase
> rewrite (speed-PI arrest handing off to an encoder **position** spring, which does converge
> on any gradient) was prototyped and reverted; if the hold ever misbehaves on a gradient
> this loop cannot settle, that is the structural answer.

### Release, brake, and gating

- **Driver takeover** requires *both* forward motion above `HH_EXIT_RPM` *and* the driver's
  demand exceeding the hold torque. Releasing on speed alone would let the hold's own settling
  motion trip it, cutting torque and dropping back into a rollback/catch cycle.
- **Takeover does not reset the estimate.** It hands off to `HH_STANDBY`, where the output
  bleeds to zero (so it can never fight the throttle) but `hh_integ` is retained and only
  decays slowly. The grade has not changed just because the driver pulled away, so a
  lift-and-roll-back on the same slope re-engages at roughly the right torque instead of
  rebuilding from zero into the `HH_MAX_TORQUE` clamp.
- **Brake, neutral, and mode change** reset the hold outright at the top of every 1 kHz step,
  which also re-arms a `HH_COOLDOWN`. The brake path is dormant in practice — the VCU does not
  publish the brake frame, so `can_d.brake_command_active` never asserts.
- **FNR gating** is on the FOC chart state (`FOC_Loop_Y.Throttle_State`), not the requested
  direction. Outside Forward/Reverse the hold torque has no defined sign, so it is discarded
  rather than remembered.
- **Torque mode only.** In speed mode the superloop stops maintaining `d.torque_final`, so
  the result is not written to `RefTrq`. The module is still stepped so it observes the mode
  change and resets, leaving no stale hold torque to apply on the way back.
- **Hold budget.** Holding at 0 RPM parks DC current in one phase pair, so `HH_MAX_HOLD_MS`
  caps how long the same windings are cooked before `HH_RELEASING` bleeds the torque out.

Telemetry is on **CAN 0x710**: `[4]` active flag, `[5]` state code (0 INACTIVE / 1 ACTIVE /
2 RELEASING / 3 COOLDOWN / 4 STANDBY), `[6..7]` hold torque Nm ×10 as u16 LE.

### The ratchet bug (fixed — do not reintroduce)

`HillHold_Apply()` **must** be passed `d.torque_final`, a variable this module never writes.
It was originally fed `FOC_Loop_U.RefTrq` — its own output. That made it a read-modify-write
on shared state: between two superloop writes, the ISR read its own floored output back as
"driver demand", so `max()` ratcheted the command to the highest torque in the window and the
driver-takeover test ended up comparing the hold torque against itself. Both the header and
the call site carry warnings, and `#if HILL_HOLD_ENABLE && !THROTTLE_BASED_REF` fails the
build rather than ratchet silently (the `CAN_BASED_REF` path has no separate driver-demand
variable yet).

### Parameters

Values are the active **HiRange XR** variant, vehicle-validated — see `vehicle_config.h`.

| Parameter | Value | Description |
|---|---|---|
| `HH_ENTRY_RPM` | 100 RPM | Rollback speed that engages the hold |
| `HH_EXIT_RPM` | 150 RPM | Forward speed for driver-takeover release |
| `HH_DEADBAND_RPM` | 15 RPM | Speed magnitude below this = held; integrator freezes |
| `HH_UNWIND_GAIN` | 3.0 | Integrator unwind multiplier on the hold's own overshoot |
| `HH_KP` / `HH_KI` | 0.055 / 1.1 | Hold gains (Nm/RPM, Nm/(RPM·s)) |
| `HH_MAX_TORQUE` | 25 Nm | Hold torque ceiling (asserted ≤ `MAX_TORQUE`) |
| `HH_MAX_HOLD_MS` | 5000 ms | Max continuous hold; 0 = unlimited |
| `HH_RELEASE_RATE_NM_S` | 8 Nm/s | Bleed-off rate after the hold budget is spent |
| `HH_SLEW_NM_S` | 150 Nm/s | Max hold-torque rate of change (no stepped state edges) |
| `HH_STANDBY_DECAY_NM_S` | 5 Nm/s | Grade-estimate decay while the driver drives |
| `HH_TS` | 0.001 s | Loop period (1 kHz, decimated 10:1 from the FOC ISR) |

---

## CAN Protocol

### RX Messages (DEBUG_CAN_ID mode: 0x10x)

| ID | Name | Payload Description |
|---|---|---|
| 0x102 | Speed reference | Bytes [1:0] → `d.speed_ref` (RPM) |
| 0x103 | Motor parameters | Lambda, Ld, Lq (write to FOC model inputs) |
| 0x104 | FW parameters | Kfw, Kaw, M2F, F2M, Rst_Factor |
| 0x105 | Speed PID | Kp, Ki, Kd for speed controller |
| 0x106 | Id PID | Kp, Ki, Kd for flux (Id) controller |
| 0x107 | Iq PID | Kp, Ki, Kd for torque (Iq) controller |
| 0x108 | Direction | Byte[0] → `can_d.direction` (F/N/R) |
| 0x109 | Power mode | Byte[0] → `can_d.power_mode` (ECO/SPORTS) |
| 0x110 | Throttle voltage | Bytes [1:0] → throttle inst. voltage (mV) |
| 0x111 | Reset / stop | Byte[0]=1 + RPM≈0 → goto RESET_STATE |
| 0x729 | Angle autotune | Byte[0]=1 + RPM≈0 → invalidate EEPROM + reset |

### RX Messages (VH_CAN_ID mode: 0x1xx — vehicle integration)

| ID | Name | Payload Description |
|---|---|---|
| 0x109 | BMS — SOC + pack voltage | Bytes [1:0] → SOC (× 0.0001, per-unit 0.0–1.0); Bytes [3:2] → pack voltage (× 0.1, V) |
| 0x110 | Throttle voltage | Bytes [1:0] → throttle inst. voltage (mV) |
| 0x12A | DCLO / DCLI | Bytes [1:0] → DCLI (× 0.1 A); Bytes [3:2] → DCLO (× 0.1 A) |

### TX Messages (DEBUG_CAN_ID mode: 0x4xx)

| ID | Function | Payload |
|---|---|---|
| 0x401 | Speed & current | RPM ref, RPM measured, Id, Iq |
| 0x402 | Current refs & encoder | Iq_ref, Id_ref, encoder count |
| 0x403 | Voltages & temperatures | Vbus, Vaux, controller temp, motor temp, error code, actual torque (+80 offset) |
| 0x404 | Motor inductances + Rs | Ld_LUT, Lq_LUT, Lambda_LUT, Rs (× 1E6, bytes 6–7) |
| 0x405 | Alignment status | Alignment flag, offset angle, throttle voltage |
| 0x406 | FW status | Kfw, Kaw (live FOC model values), Vs (stator voltage), Vmax |
| 0x407 | Speed PID live values | Kp, Ki, Kd (current operating values), M2F, F2M |
| 0x408 | Id PID live values | Kp, Ki, Kd, Rst_Factor |
| 0x409 | Iq PID live values | Kp, Ki, Kd, Vs (byte 6), Vmax (byte 7) |
| 0x410 | Additional control data | Error code, firmware version, FWC flag |

### TX Messages (VH_CAN_ID mode: 0x7xx — vehicle integration)

| ID | Payload |
|---|---|
| 0x705 | Motor temp, controller temp |
| 0x706 | Bus voltage, speed (km/h), error code, FWC/regen flags |
| 0x708–0x709 | Current and historic error codes |
| 0x710 | Throttle percent, throttle voltage |
| 0x715 | RPM, bus voltage, 12 V aux voltage |
| 0x716 | Vrms, Irms |
| 0x717 | RPM |
| 0x724 | Torque reference |

CAN Tx is queued into a circular buffer (15 frames) and drained in 2 ms time slots
keyed to DEVICE_ID (0x03) to avoid bus collisions with other ECUs.

---

## Angle Calibration & Position Sensing

Rotor position currently comes from the **quadrature encoder on TIM2**. Two other paths exist
in the tree at different stages of readiness — know which is live before debugging an angle
problem.

### Live path — encoder + autotune offset

`isr_angle_calib()` in [`Core/Src/isr_handlers.c`](Core/Src/isr_handlers.c) drives the motor
open-loop (`Open_FOC0`) under the precompiled `libAngle_Offset_101.a` autotune routine. It
sweeps CW and CCW, produces `f_Offset_CW` / `f_Offset_CCW`, and on
`AutotuneCompleteflag` advances `cS = ANGLE_OFFSET_STORE` so the superloop persists them to
EEPROM. Normal running then applies the offset:

```c
d.mech_angle = ((float)d.encoder_count * COUNTS_TO_RADS) - d.ATA_Offset;
d.elec_angle = fmodf(d.mech_angle * POLEPAIRS, TWO_PI);
```

### Not yet wired — AD2S1210 resolver

`Ad2s1210.c` / `Resolver_Position.c` / `spi.c` are present and SPI1 initialises, but
**nothing calls `AD2S1210_Init()`** and the control loop does not read from them. The design
is: SPI absolute angle at startup → TIM2 incremental tracking after the first Z pulse (PE7
EXTI). Full state and next steps in [HANDOFF_AD2S1210.md](HANDOFF_AD2S1210.md).

> Watch for this class of bug when bringing it up: a missing `AD2S1210_ReadAngle` prototype
> once made the compiler assume an implicit `int` return. The function returns `float`, so on
> the hard-float EABI the result came back in `s0` while the caller read `r0` — every resolver
> angle read returned register garbage. Fixed, but it was invisible while the encoder path was
> the active one.

### Not merged — phase-correction model

`origin/Phase_correction_branch` replaces the `ANGLE_CALIB` dispatch with a Simulink
`isr_phase_correction()` (`Phase_correction.c`, model v1.21) that feeds encoder angle and
phase currents into a model driving `Open_FOC0`. Before merging, note that as written it reads
the encoder **without** subtracting `d.ATA_Offset` and never advances `cS`, so it does not
produce or store an offset the way the autotune path does — confirm whether that is intended
(bench identification only) or unfinished. The branch also bundles unrelated changes:
`EEPROM_MAGIC_NUM`, `MOTOR_PROFILE`, and `ENABLE_DEBUG` in `Build.bat`.

---

## EEPROM — Angle Calibration Persistence

The rotor angle offset is measured once (angle autotune routine) and stored in EEPROM
page 0 so subsequent boots skip the alignment routine.

**Page 0 Layout (8 bytes at address 0x0000)** — see the comment block in
[`Core/Inc/Eeprom.h`](Core/Inc/Eeprom.h):

| Bytes | Field | Notes |
|---|---|---|
| 0 | `EEPROM_MAGIC_NUM` | Validates page is written — **current value 0x21** |
| 1–2 | `offset_high` | uint16, big-endian |
| 3–4 | `offset_angle_mech_cw` | CW direction offset, uint16 big-endian |
| 5–6 | `offset_angle_mech_ccw` | CCW direction offset, uint16 big-endian |
| 7 | CRC8 | XOR checksum over bytes 0–6 |

On boot, `Read_EEPROM_at_init()` checks the magic number and CRC. If valid,
`d.start_alignment = 0` and FOC starts immediately in closed-loop. If invalid or
page is erased, `d.start_alignment = 1` and the calibration boot path runs. A CAN `0x729`
command (RPM ≈ 0) invalidates the page and resets; `FORCE_ANGLETUNE = 1` forces calibration
on every boot.

> **`EEPROM_MAGIC_NUM` is a calibration invalidation switch.** Bumping it in
> [`board_config.h`](Core/Inc/board_config.h) makes every previously-calibrated board discard
> its stored offsets and silently re-align on the next boot. That is sometimes what you want
> after changing the offset semantics — but it is a real side effect, so check this macro
> first if a board re-calibrates unexpectedly after a merge.

---

## OTA & Bootloader

This branch carries the CAN bootloader alongside the application, which is why the build
produces three images instead of one.

### Flash layout

Defined in [`LinkerScripts/MemoryMap.ld`](LinkerScripts/MemoryMap.ld):

| Region | Origin | Length | Contents |
|---|---|---|---|
| Bootloader | `0x08000000` | 100 K | CAN bootloader, image validation, slot selection |
| App1 | `0x08040000` | 256 K | Application slot 1 (the OTA target) |
| App2 | `0x08080000` | 256 K | Application slot 2 |
| CommonData | `0x080C0000` | 40 K | Reserved, currently unused |
| SharedRAM | `0x20000000` | 10 K | `.shared_memory` — flags shared between bootloader and apps |

`SharedRAM` is carved off the front of DTCMRAM and marked `NOLOAD`, so it survives the jump
from bootloader to application and is how the two sides communicate.

### Image header

Each app image begins with a 32-byte `image_hdr_t` (magic `0x9696`, version 2). CRC and
payload size are **not** produced by the compiler — [`patch_image_header.py`](patch_image_header.py)
stamps them into the `.bin` after linking:

```bash
python patch_image_header.py Build/Pegasus_v4-App1.bin
```

`Build.bat` runs this automatically. An image that skips it will be rejected by the
bootloader.

### OTA CAN protocol

| ID | Role |
|---|---|
| `0x6FA` | OTA session control / command |
| `0x7FE` | OTA data payload |
| `0x7FF` | OTA response |

Handled in [`Core/Src/main.c`](Core/Src/main.c) from the FDCAN Rx callback. On session start
`OTA_FDCAN_SETUP()` reprograms the CAN filters to accept **only** these three IDs and reject
the rest of the bus; `OTA_FDCAN_RESTORE()` puts the normal filters back after a failed
transfer.

> That filter isolation is a fix, not an optimisation. Without it the controller accepted
> live vehicle traffic during a transfer, and the Rx path spent its time on frames the OTA
> session did not care about — transfers ran extremely slowly. If OTA throughput regresses,
> check `OTA_FDCAN_SETUP()` is still being reached.

### SWD vs OTA

**SWD (first time / recovery)** — `Flash.bat` mass-erases and writes all three images via
OpenOCD at their fixed addresses. Required once per board, or whenever the bootloader itself
changes.

**OTA (normal iteration)** — PCAN adapter plus `Flash_Console.exe`, targeting the App1 slot.
No ST-Link needed once a bootloader is resident.

---

## Fault & Error Handling

Faults are tracked in the `errors er` struct and broadcast over CAN.

| Code | Name | Trigger |
|---|---|---|
| 1 | `CURR_SENS_ERROR` | Phase current ADC outside [14000, 40000] counts at rest |
| 2 | `BUS_VOLTAGE_OV_ERROR` | Vbus > 80 V |
| 3 | `BUS_VOLTAGE_UV_ERROR` | Vbus < 40 V |
| 4 | `AUX_VOLTAGE_OV_ERROR` | Vaux overvoltage |
| 5 | `AUX_VOLTAGE_UV_ERROR` | Vaux < 9 V |
| 6 | `ID_IQ_OC_ERROR` | |Id| or |Iq| exceeds limit |
| 7 | `PHASE_CURR_ERROR` | Phase current magnitude fault |
| 8 | `MTR_TEMP_OT_ERROR` | Motor NTC over-temperature |
| 9 | `MTC_TEMP_OT_ERROR` | Controller NTC over-temperature |
| 10 | `MTR_TEMP_DISCONNECTION_ERROR` | NTC open / short circuit |
| 11 | `MTC_TEMP_DISCONNECTION_ERROR` | NTC open / short circuit |
| 12 | `EEPROM_WRITE_ERROR` | I2C write failed |
| 13 | `EEPROM_READ_ERROR` | I2C read failed |
| 14 | `ENCODER_A_B_ERROR` | A/B quadrature signal lost |
| 15 | `ENCODER_Z_ERROR` | Index pulse not detected |
| 16 | `ENCODER_PWM_ERROR` | PWM absolute encoder signal fault |
| 17 | `ENCODER_5V_ERROR` | Encoder 5 V supply fault |
| 18 | `STALL_PROTECTION` | Stall condition detected |

Any fault sets `cS = CONT_ERROR` and calls `disable_drive()`, which disables the
TIM1 PWM outputs and the gate-driver enable pin (GPIOD.15).

---

## Building & Flashing

### Docker — the supported path

Use the container. It guarantees everyone builds with the **exact** same tool versions
(`arm-none-eabi-gcc 10.3-2021.10`, `cmake 3.27.5`, `openocd xPack 0.11.0-5`), and no local
ARM toolchain / CMake / Python is needed:

```bash
docker compose build            # once
docker compose run --rm builder # builds all three images into ./Build
```

Full guide: [DOCKER.md](DOCKER.md).

> **A host `cmake` invocation will fail.** The checked-in build cache holds the container's
> `/work` paths, so configuring on the host errors out rather than falling back. Build in
> Docker, or delete `Build/` and reconfigure from scratch if you genuinely need a host build.

### Build output

The build produces three images plus the bootloader, all in `Build/`:

| File | Flashed to |
|---|---|
| `Pegasus_v4-Bootloader.{elf,hex,bin}` | `0x08000000` |
| `Pegasus_v4-App1.{elf,hex,bin}` | `0x08040000` |
| `Pegasus_v4-App2.{elf,hex,bin}` | `0x08080000` |

`Build.bat` then runs `patch_image_header.py` on the App1 binary to stamp its CRC and size —
see [OTA & Bootloader](#ota--bootloader). An OTA image is invalid without it.

### Adding source files

New `.c` files must be added to `FIRMWARE_FILES` in [`CMakeLists.txt`](CMakeLists.txt).
Nothing globs the source tree, so a file that is not listed is silently never compiled — the
build succeeds and the feature simply is not there.

### Build scripts

```bash
bash Build.sh    # Linux / WSL
Build.bat        # Windows — also runs patch_image_header.py
```

`Build.bat` passes `-DENABLE_DEBUG`; keep it at `0` for images that go on a vehicle.

### Flashing

```bash
Flash.bat        # SWD via OpenOCD: mass-erase, then Bootloader + App1 + App2
```

For OTA over CAN (the normal iteration loop once a bootloader is resident), use a PCAN
adapter with `Flash_Console.exe` against the App1 slot.

### STM32CubeIDE

The `.ioc` and a `.launch` configuration are still checked in for peripheral editing and
debugging, but the CMake/Docker path is what produces release images — the CubeIDE project
does not build the three-image layout.

## API Documentation

The repository includes a Doxygen configuration for the firmware, bootloader, STM32 HAL/CMSIS
sources, host-side Python tools, and Markdown documentation. Build the HTML reference from the
repository root with Doxygen installed:

```bash
doxygen Doxyfile
```

Open `docs/html/index.html` to browse the overview and source summaries. Doxygen uses
Graphviz to generate function call/caller graphs, include graphs, and directory
hierarchies; open a function page to see its graphs. Build outputs and generated
documentation are excluded from the input and Git, respectively. `EXTRACT_ALL` keeps
symbols visible even before they have Doxygen comments; add `/** @brief ... */` comments
to declarations and document parameters and return values as APIs are updated.
