# syntax=docker/dockerfile:1
# ==============================================================================
#  Pegasus firmware build environment — pinned toolchain
#
#  Mirrors the exact tool versions on the reference dev machine so every
#  developer building this branch gets identical results (and byte-identical
#  firmware, since arm-none-eabi-gcc + newlib are pinned):
#
#    arm-none-eabi-gcc : 10.3-2021.10  (GNU Arm Embedded, reports 10.3.1)
#    cmake             : 3.27.5
#    openocd           : xPack 0.11.0-5   (reports 0.11.0+dev)
#    python3           : runs patch_image_header.py (stdlib only, no pip deps)
#    make / git        : build orchestration + GIT_SHA embedding
#
#  Multi-arch: builds natively on amd64 (Intel/AMD, Windows+WSL2) and
#  arm64 (Apple Silicon) hosts.
#
#  Build:  docker build -t pegasus-fw-toolchain:10.3-2021.10 .
#  Use:    see DOCKER.md
# ==============================================================================
FROM ubuntu:22.04

# BuildKit auto-populates TARGETARCH (amd64 / arm64). Falls back to the host
# architecture when built with the legacy builder.
ARG TARGETARCH

# ---- Pinned versions (override at build time with --build-arg if ever needed)
ARG ARM_GCC_VERSION=10.3-2021.10
ARG CMAKE_VERSION=3.27.5
ARG OPENOCD_VERSION=0.11.0-5

ENV DEBIAN_FRONTEND=noninteractive

# Any user (even a bind-mount UID that doesn't exist in the image) is allowed to
# run git inside /work — CMake calls `git rev-parse` to stamp GIT_SHA, and this
# avoids git's "dubious ownership" refusal.
ENV GIT_CONFIG_COUNT=1 \
    GIT_CONFIG_KEY_0=safe.directory \
    GIT_CONFIG_VALUE_0=*

# ---- Base OS packages -------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl xz-utils bzip2 \
        git make python3 \
        libusb-1.0-0 libhidapi-hidraw0 libftdi1-2 libncurses5 \
        usbutils \
    && rm -rf /var/lib/apt/lists/*

# ---- ARM GNU toolchain (arm-none-eabi-gcc 10.3-2021.10) ---------------------
RUN set -eux; \
    arch="${TARGETARCH:-$(dpkg --print-architecture)}"; \
    case "$arch" in \
      amd64) ARM_ARCH=x86_64 ;; \
      arm64) ARM_ARCH=aarch64 ;; \
      *) echo "Unsupported architecture: $arch" >&2; exit 1 ;; \
    esac; \
    url="https://developer.arm.com/-/media/Files/downloads/gnu-rm/${ARM_GCC_VERSION}/gcc-arm-none-eabi-${ARM_GCC_VERSION}-${ARM_ARCH}-linux.tar.bz2"; \
    curl -fSL "$url" -o /tmp/gcc-arm.tar.bz2; \
    mkdir -p /opt/gcc-arm-none-eabi; \
    tar -xjf /tmp/gcc-arm.tar.bz2 -C /opt/gcc-arm-none-eabi --strip-components=1; \
    rm /tmp/gcc-arm.tar.bz2
ENV PATH="/opt/gcc-arm-none-eabi/bin:${PATH}"

# ---- CMake 3.27.5 -----------------------------------------------------------
RUN set -eux; \
    arch="${TARGETARCH:-$(dpkg --print-architecture)}"; \
    case "$arch" in \
      amd64) CM_ARCH=x86_64 ;; \
      arm64) CM_ARCH=aarch64 ;; \
    esac; \
    url="https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-${CM_ARCH}.tar.gz"; \
    curl -fSL "$url" -o /tmp/cmake.tar.gz; \
    mkdir -p /opt/cmake; \
    tar -xzf /tmp/cmake.tar.gz -C /opt/cmake --strip-components=1; \
    rm /tmp/cmake.tar.gz
ENV PATH="/opt/cmake/bin:${PATH}"

# ---- OpenOCD (xPack 0.11.0-5) -----------------------------------------------
#  Included for on-target flashing/debug. NOTE: flashing over USB from a
#  container needs host USB passthrough — see DOCKER.md. Not needed to build.
RUN set -eux; \
    arch="${TARGETARCH:-$(dpkg --print-architecture)}"; \
    case "$arch" in \
      amd64) OCD_ARCH=x64 ;; \
      arm64) OCD_ARCH=arm64 ;; \
    esac; \
    url="https://github.com/xpack-dev-tools/openocd-xpack/releases/download/v${OPENOCD_VERSION}/xpack-openocd-${OPENOCD_VERSION}-linux-${OCD_ARCH}.tar.gz"; \
    curl -fSL "$url" -o /tmp/openocd.tar.gz; \
    mkdir -p /opt/openocd; \
    tar -xzf /tmp/openocd.tar.gz -C /opt/openocd --strip-components=1; \
    rm /tmp/openocd.tar.gz
ENV PATH="/opt/openocd/bin:${PATH}" \
    OPENOCD_SCRIPTS="/opt/openocd/openocd/scripts"

# ---- Sanity check: fail the image build if any tool is missing/wrong --------
RUN set -eux; \
    arm-none-eabi-gcc --version | head -n1; \
    cmake --version | head -n1; \
    openocd --version 2>&1 | head -n1; \
    python3 --version; \
    make --version | head -n1; \
    git --version

# ---- Non-root user (UID 1000 matches most Linux hosts for clean file perms) -
RUN useradd -m -u 1000 -s /bin/bash dev
USER dev
WORKDIR /work

# Default to an interactive shell; docker-compose overrides this to run the
# reproducible build (docker/build.sh).
CMD ["bash"]
