#!/usr/bin/env bash
# ==============================================================================
#  docker/build.sh — reproducible in-container firmware build
#
#  Linux equivalent of Build.bat, run inside the pinned toolchain container.
#  Produces the same artifacts (Bootloader / App1 / App2 .elf/.hex/.bin) into
#  Build/, then stamps CRC + size into the App image headers.
#
#  Not meant to be run on the host directly — invoke via docker-compose or
#  `docker run` (see DOCKER.md). It assumes arm-none-eabi-gcc, cmake and
#  python3 are on PATH, which the container guarantees.
# ==============================================================================
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

BUILD_DIR="Build"
BUILD_TYPE="Debug"

echo "==> Toolchain in use:"
arm-none-eabi-gcc --version | head -n1
cmake --version | head -n1

echo "==> Clean configure into ${BUILD_DIR}/"
rm -rf "$BUILD_DIR"
cmake -S . -B "$BUILD_DIR" \
      -G "Unix Makefiles" \
      --toolchain "$ROOT/arm-none-eabi-gcc.cmake" \
      -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
      -DENABLE_DEBUG=0

echo "==> Build ($(nproc) jobs)"
cmake --build "$BUILD_DIR" --parallel "$(nproc)"

echo "==> Patch App image headers (CRC + data_size)"
python3 patch_image_header.py "$BUILD_DIR/Pegasus_v4-App1.bin"
python3 patch_image_header.py "$BUILD_DIR/Pegasus_v4-App2.bin"

echo "==> Done. Artifacts in ${BUILD_DIR}/:"
ls -1 "$BUILD_DIR"/*.elf "$BUILD_DIR"/*.hex "$BUILD_DIR"/*.bin 2>/dev/null || true
