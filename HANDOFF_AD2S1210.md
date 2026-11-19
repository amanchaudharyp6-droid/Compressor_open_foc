# Session Handoff — AD2S1210 RDC port to `OTA_Reintegrated`

**Date:** 2026-07-20 · **Branch:** `OTA_Reintegrated` · **Written by:** Claude Code session on home laptop.
To resume on another machine: pull this branch, then tell Claude Code to read this file.

## Task

Bring **only** the AD2S1210 resolver-to-digital converter support from
`dev/PEG_v4/FOC_Model_Based/Resolver/v1.1.0` into `OTA_Reintegrated` — without dragging in the
rest of that branch (it has diverged heavily: DCLO logic, Offset0, Angle_to_Speed, throttle/regen
changes, etc.).

## What was done (all in the commit that adds this file)

**Copied verbatim from the resolver branch (self-contained, no external deps):**
- `Core/Inc/Ad2s1210.h`, `Core/Src/Ad2s1210.c` — RDC driver. SPI1 Mode 1, 8-bit, MSB-first,
  hardware NSS. Reads 24-bit position/velocity + fault register. `extern SPI_HandleTypeDef hspi1`.
- `Core/Inc/Resolver_Position.h`, `Core/Src/Resolver_Position.c` — position manager: SPI absolute
  angle at startup → TIM2 incremental tracking after first Z pulse (PE7 EXTI). `extern htim2`
  (already defined in `Core/Src/tim.c` on this branch).
- `Drivers/STM32H7xx_HAL_Driver/{Inc,Src}/stm32h7xx_hal_spi.{h,c}` and `..._spi_ex.{h,c}` —
  SPI HAL did not exist on this branch.

**Supporting edits (adapted, NOT copied — see "structural difference" below):**
- `Core/Inc/stm32h7xx_hal_conf.h` — enabled `HAL_SPI_MODULE_ENABLED`.
- `CMakeLists.txt` — SPI HAL + `spi.c` added to `COMMON_FILES`; `Ad2s1210.c` +
  `Resolver_Position.c` added to `FIRMWARE_FILES`.
- **NEW** `Core/Src/spi.c` + `Core/Inc/spi.h` — `hspi1`, `MX_SPI1_Init()` (config copied exactly
  from resolver branch: prescaler 8 → 6.25 MHz, CPOL=0/CPHA=1, NSS hard output), plus
  `HAL_SPI_MspInit/DeInit` (PD7 MOSI, PG9 MISO, PG10 NSS, PG11 SCK, AF5, SPI123 clock = PLL).
- `Core/Src/gpio.c` — AD2S1210 control pins: PG3 RESET (out, high), PG4 RES1 (out, high),
  PG5 RES0 (out, low), PG6 A1 / PG7 A0 (out, low), PG8 SAMPLE (out, high). 14-bit mode, normal
  (position) mode. `AD2S1210_Init()` does NOT configure GPIOs itself — it relies on these.
- `Core/Src/main.c` — `#include "spi.h"` and `MX_SPI1_Init();` after `MX_TIM4_Init();`.

**Structural difference vs the resolver branch (important if you diff/merge later):**
The resolver branch puts `hspi1` in `main.c`, `MX_SPI1_Init` + GPIO in `Core/Src/Peripheral_Init.c`,
and MSP in `stm32h7xx_hal_msp.c`. On `OTA_Reintegrated`, `Peripheral_Init.c` is fully commented out
and live peripheral code is CubeMX per-peripheral files (`tim.c`, `i2c.c`, `gpio.c`, central MSP
file has only `HAL_MspInit`). The port follows *this* branch's structure (`spi.c`/`spi.h` pair,
MSP inside `spi.c`, control pins in `gpio.c`).

## Verified

`docker compose run --rm builder` (local cmake is broken; `Build.bat` nukes `Build/`): all three
targets (Bootloader, App1, App2) compile, link, and get CRC-patched. App text ≈ 85 KB, 32.5% of
region. No new warnings.

## Deliberately NOT ported (next steps, in rough order)

1. **Nothing calls the driver yet.** SPI1 is initialized, but `AD2S1210_Init()` /
   `AD2S1210_ReadAll()` / `ResolverPosition_*` are never invoked. On the resolver branch, `main()`
   calls `AD2S1210_Init()` before `set_Initial_angle()` (guarded by `RESOLVER_ENABLED`, defined in
   its `vehicle_config.h`). Port that startup wiring when ready.
2. ISR/FOC-loop integration (`isr_handlers.c` on resolver branch, ~100 real changed lines) and the
   Z-pulse handover call (`ResolverPosition_OnZPulse`-style logic in `HAL_GPIO_EXTI_Callback`).
3. `Offset0` + `Angle_to_Speed` modules and `data` struct fields (`elec_angle_resolver`,
   `omega_e_resolver`, `mech_angle_resolver`, …) — resolver-branch features, not required by the
   driver files.
4. Unrelated resolver-branch tweaks that were skipped on purpose: TIM4 IRQ priority 5→7, PD5 input,
   `z_count > 4` reset, regen/low-speed `#define`s in `main.h`, `delay_us()` helper (driver does
   not use it — it uses `__NOP` loops + `HAL_Delay`).

## Check before running on hardware

`Core/Inc/Resolver_Position.h` hardcodes `RESOLVER_COUNTS_PER_REV 16384` and
`RESOLVER_POLE_PAIRS 4` with "update to your motor" comments — confirm against the target motor
profile (`Core/Inc/motor_profiles/`). `TIM2_ARR` comes from the motor profile headers and already
exists on this branch.
