# Reproducible Build Environment (Docker)

This container pins the **exact** toolchain versions used on the reference dev
machine, so everyone building this branch gets identical results regardless of
what's installed on their laptop. This is the fix for the "works on my machine /
fails on my colleague's" problem.

## Pinned versions

| Tool | Version | Source |
|---|---|---|
| `arm-none-eabi-gcc` | **10.3-2021.10** (reports `10.3.1`) | ARM GNU Embedded Toolchain |
| `cmake` | **3.27.5** | Kitware official binary |
| `openocd` | **xPack 0.11.0-5** (reports `0.11.0+dev`) | xpack-dev-tools |
| `python3` | Ubuntu 22.04 (3.10.x) | runs `patch_image_header.py` (stdlib only) |
| `make` / `git` | Ubuntu 22.04 | build orchestration + `GIT_SHA` stamp |

> The compiler + bundled newlib are what determine the firmware bytes, so those
> are the critical pins. `make`/`git` only orchestrate the build.

The image is **multi-arch**: it builds natively on Intel/AMD (`amd64`, incl.
Windows + WSL2) and Apple Silicon (`arm64`) hosts.

---

## Prerequisites

Install **Docker Desktop** (Windows/macOS) or Docker Engine + the Compose plugin
(Linux). Nothing else — no ARM toolchain, no CMake, no Python on the host.

---

## Quick start

From the repository root:

```bash
# 1) Build the toolchain image (once; ~600 MB download, cached afterwards)
docker compose build

# 2) Build the firmware — artifacts land in ./Build on your host
docker compose run --rm builder
```

`Build/` will contain the Bootloader / App1 / App2 `.elf`, `.hex` and `.bin`
(App headers CRC-patched), exactly as `Build.bat` produces natively.

### Get an interactive toolchain shell

```bash
docker compose run --rm builder bash
# then, inside the container:
bash docker/build.sh          # full clean build
arm-none-eabi-gcc --version   # poke around
```

### Without Compose (plain Docker)

```bash
docker build -t pegasus-fw-toolchain:10.3-2021.10 .

# Windows PowerShell:
docker run --rm -v "${PWD}:/work" -w /work pegasus-fw-toolchain:10.3-2021.10 bash docker/build.sh
# Linux / macOS:
docker run --rm -v "$PWD:/work" -w /work pegasus-fw-toolchain:10.3-2021.10 bash docker/build.sh
```

---

## Building a specific motor profile

Pass CMake defines through by editing `docker/build.sh`, or run cmake directly
in a shell. Example for the LivGuard profile:

```bash
docker compose run --rm builder bash -c '
  cmake -S . -B Build -G "Unix Makefiles" \
    --toolchain "$PWD/arm-none-eabi-gcc.cmake" \
    -DCMAKE_BUILD_TYPE=Debug -DENABLE_DEBUG=0 \
    -DMOTOR_PROFILE=MOTOR_PROFILE_LIVGUARD &&
  cmake --build Build --parallel $(nproc) &&
  python3 patch_image_header.py Build/Pegasus_v4-App1.bin &&
  python3 patch_image_header.py Build/Pegasus_v4-App2.bin
'
```

---

## Flashing

OpenOCD is baked into the image and there's a dedicated `flasher` service that
runs privileged + as root so it can open the raw ST-Link USB device. It runs
[`docker/flash.sh`](docker/flash.sh) — the Linux equivalent of `Flash.bat`
(mass-erase → Bootloader → App1 → App2, verified).

The only hard part is getting the ST-Link *into* the container. That differs by
host.

### Linux host (easiest)

The ST-Link is already on `/dev/bus/usb`, so just run:

```bash
docker compose run --rm flasher
```

For **non-privileged** flashing on Linux, install the OpenOCD udev rules on the
**host** (copy [`docker/60-openocd.rules`](docker/60-openocd.rules) to
`/etc/udev/rules.d/`, then `sudo udevadm control --reload && sudo udevadm
trigger`, and replug). Privileged/root (the default `flasher` service) works
without them.

### Windows host (Docker Desktop + WSL2)

Docker Desktop containers live inside a WSL2 VM that can't see Windows USB
devices. You bridge that with [`usbipd-win`](https://github.com/dorssel/usbipd-win),
which forwards the USB device into WSL2 where Docker can reach it.

**One-time setup (Admin PowerShell):**

```powershell
winget install usbipd            # install usbipd-win, then reopen PowerShell
```

**Each flashing session:**

1. Plug in the ST-Link. List USB devices and find it:
   ```powershell
   usbipd list
   ```
   Look for `STMicroelectronics STLink` and note its **BUSID** (e.g. `2-4`).

2. Bind it (first time per device only; Admin PowerShell):
   ```powershell
   usbipd bind --busid 2-4
   ```

3. Attach it into WSL2 (every session, and after each unplug):
   ```powershell
   usbipd attach --wsl --busid 2-4
   ```

4. Flash:
   ```powershell
   docker compose run --rm flasher
   ```

5. When done, release it back to Windows (optional):
   ```powershell
   usbipd detach --busid 2-4
   ```

**Verify passthrough before flashing.** The flash script prints `lsusb` and
aborts if it doesn't see an ST-Link. To check manually:

```powershell
docker compose run --rm flasher lsusb
```

If the ST-Link isn't listed, the `usbipd attach` didn't reach the container —
re-run step 3, confirm Docker Desktop's WSL2 integration is on, and make sure
STM32CubeProgrammer / ST-Link GDB server isn't holding the device open on
Windows.

### macOS host

Docker Desktop on macOS also runs in a VM with no native USB passthrough, and
there's no clean equivalent of usbipd. **Flash from the host** with
STM32CubeProgrammer against the `.bin` files in `Build/`.

### Simplest fallback (any OS)

If USB passthrough is fighting you, just build in the container and flash from
the **host** with STM32CubeProgrammer / host OpenOCD, pointing at the `.bin`
files in `Build/`. The container only needs to *produce* the firmware; flashing
it is host-side. Load addresses (from `Flash.bat`):

| Image | Address |
|---|---|
| `Pegasus_v4-Bootloader.bin` | `0x08000000` |
| `Pegasus_v4-App1.bin` | `0x08040000` |
| `Pegasus_v4-App2.bin` | `0x08080000` |

---

## Notes & troubleshooting

- **File ownership (Linux hosts only):** the container runs as UID 1000. If your
  host user isn't UID 1000, `Build/` files may be owned by 1000. Uncomment the
  `user:` line in `docker-compose.yml` (or add `--user "$(id -u):$(id -g)"` to
  `docker run`). On Windows/macOS this is a non-issue.
- **`GIT_SHA`:** CMake runs `git rev-parse` to stamp the build. The image sets
  `safe.directory=*` so this works even though the mounted repo is owned by a
  different user than the container's.
- **First build is slow** (downloads the ARM toolchain, CMake, OpenOCD). It's
  cached in the image afterwards — only re-downloaded if you change a pinned
  version or rebuild with `--no-cache`.
- **Pinning a different version later:** override at build time, e.g.
  `docker compose build --build-arg CMAKE_VERSION=3.28.0`, or edit the `ARG`
  defaults at the top of the `Dockerfile`.
- **CI reuse:** the same image is a drop-in for CI — mount the checkout and run
  `docker/build.sh`.
