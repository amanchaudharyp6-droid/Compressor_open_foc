#!/usr/bin/env bash
# ==============================================================================
#  docker/flash.sh — flash Bootloader + App1 + App2 via OpenOCD (ST-Link)
#
#  Linux equivalent of Flash.bat. Only usable when the ST-Link USB device is
#  passed through to the container (Linux hosts, or Windows via usbipd-win).
#  See DOCKER.md → "Flashing". On most setups it is simpler to flash from the
#  host with STM32CubeProgrammer / host OpenOCD using the .bin files this
#  container produced in Build/.
# ==============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

IFACE="interface/stlink.cfg"
TARGET="target/stm32h7x.cfg"

# ---- Preflight: is the ST-Link actually visible inside the container? --------
echo "==> USB devices seen inside the container:"
lsusb || true
if ! lsusb | grep -qiE 'ST-?LINK|STMicroelectronics'; then
    echo
    echo "!! No ST-Link found on the container's USB bus." >&2
    echo "   On Windows/macOS you must attach it into WSL2/the VM first with" >&2
    echo "   usbipd-win — see DOCKER.md -> 'Flashing'. Aborting." >&2
    exit 1
fi

echo "==> Mass erase"
openocd -f "$IFACE" -f "$TARGET" -c "init; reset halt; stm32h7x mass_erase 0; exit"

echo "==> Program Bootloader @ 0x08000000"
openocd -f "$IFACE" -f "$TARGET" -c "program Build/Pegasus_v4-Bootloader.bin reset verify exit 0x8000000"

echo "==> Program App1 @ 0x08040000"
openocd -f "$IFACE" -f "$TARGET" -c "program Build/Pegasus_v4-App1.bin reset verify exit 0x8040000"

echo "==> Program App2 @ 0x08080000"
openocd -f "$IFACE" -f "$TARGET" -c "program Build/Pegasus_v4-App2.bin reset verify exit 0x8080000"

echo "==> Flash complete."
