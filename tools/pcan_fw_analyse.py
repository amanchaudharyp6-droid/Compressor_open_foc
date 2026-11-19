#!/usr/bin/env python3
"""
Field-weakening / over-modulation analysis from PCAN .trc drive-cycle logs.

    python tools/pcan_fw_analyse.py "<folder containing .trc files>" [--sign=+1|-1]

Pure stdlib (no numpy/pandas) so it runs anywhere. Streams the files, so a
multi-hundred-MB log set is fine.

-----------------------------------------------------------------------------
CAN DECODE  (HiRange XR, VH_CAN_ID build — verified against Core/Src/fdcan_AL.c,
unchanged between commit 5a6013d 2026-05-08 and HEAD). All u16 LITTLE-ENDIAN.

  0x705  [0]    motor temp      byte - 50   degC
         [1]    ctrl temp       byte - 50   degC
         [2..3] Iq feedback     u16  - 512  A
         [4..5] Id feedback     u16  - 512  A
         [6..7] offset CW       u16  / 100  rad
  0x706  [0]    drive mode      0x08 SPORTS+FWD, 0x02 ECO+FWD, 0x04 REV, 0x01 N
         [1]    flags           bit2 regen, bit3 Vbus<=40, bit4 MTC overtemp
         [4..5] speed           u16  / 10   km/h
  0x724  [0..1] torque ref      u16  / 10   Nm
         [2..3] Vq              u16  - 62   V
         [4..5] Vd              u16  - 62   V
         [7]    session state cS

GOTCHAS THAT COST TIME ONCE ALREADY:
  * THE SIGN OF Iq FOR FORWARD MOTORING IS NOT FIXED ACROSS VEHICLES. The
    gearbox orientation changes as the mech team iterates, which flips
    MOTOR_DIRECTION / REGEN_POLARITY, and Iq (plus regen Iq) flips with it:
        Id_ref_MTPA = K - sqrt(K^2 + Is^2/2)          -> ALWAYS <= 0
        Iq_ref_MTPA = sqrt(...) * sign(T_ref_final)   -> CARRIES THE SIGN
    So Id is orientation-invariant and only Iq flips. This script therefore
    DETECTS the motoring sign from the data instead of assuming it, and all
    the maths below uses |Iq|. Override with --sign=+1 / --sign=-1 if you
    need to force it. On the 20-07-2026 HiRange XR log the detected sign is
    NEGATIVE; hardcoding Iq > 0 on that log yields ~2% of samples, all
    regen/reverse outliers, and confidently wrong results.
  * Detection anchor: regen is capped at REGEN_MAX_TORQUE (5 Nm on XR, ~22 A
    peak), so any sample with |Iq| above IQ_MOTORING_A is unambiguously
    motoring. torque_ref on 0x724 is NOT usable for this -- it comes through
    as a positive magnitude regardless of direction.
  * These frames are on the 500 ms CAN queue drain, i.e. ~2 Hz. Fine for
    steady-state binning over tens of thousands of samples; useless for
    current ripple or cycle-by-cycle saturation dynamics.
  * Vd/Vq use a +62 offset, so anything below -62 V clips to raw 0. The script
    counts those; if the count is non-zero the voltage stats are floored.
  * Id_ref / Iq_ref / V_s / V_max are only on the 0x40x frames, which are
    "#if DEBUG_CAN_ID" and therefore ABSENT from a VH_CAN_ID log.

-----------------------------------------------------------------------------
WHAT THE THREE TESTS MEAN

The FW law (Current_Ref_Estimator.c) rotates the current vector along a
constant-|Is| circle from the MTPA angle toward +/-pi. integ = 1 is pure MTPA
(no weakening); integ = 0 puts all current on the -d axis (zero torque).
MTPA is a pure function of |Iq| with NO speed dependence, therefore:

  TEST 1  Id vs the MTPA locus.  FW inert -> every sample lands on one curve
          at all speeds.  FW acting -> Id sits BELOW it at high speed.
  TEST 2  Current angle off the -d axis, normalised by the MTPA angle. This
          is the ACTUAL weakening, which under voltage saturation reflects
          physics rather than the commanded reference -- so treat it as
          corroboration, not as a direct read of the FW integrator.
  TEST 3  Commanded |V| against the SVM circle. This is the decisive one:
          Vd and Vq are clamped INDEPENDENTLY at +/-Vbus/sqrt(3), so the pair
          can command sqrt(2) x that. Over-modulation shows up here.
"""
import glob, math, os, sys

# --- HiRange XR / motor_default.h -------------------------------------------
LAMBDA, LD, LQ, RS, POLES = 0.0263, 0.148e-3, 0.386e-3, 0.01047, 3.0
OP_VOLTAGE   = 96.0
VMAX_SVM     = OP_VOLTAGE / math.sqrt(3.0)     # 55.43 V phase-peak
RPM_PER_KMPH = 1.0 / 0.01044                   # RPM_TO_KMPH_HI_RANGE
K            = LAMBDA / (4.0 * (LQ - LD))      # 27.63 A

# Above this |Iq| a sample cannot be regen (REGEN_MAX_TORQUE 5 Nm ~= 22 A peak
# on HiRange XR), so its Iq sign is the motoring sign for this vehicle.
IQ_MOTORING_A = 60.0

BANDS = [(1500,2000),(2000,2500),(2500,3000),(3000,3500),(3500,4000),
         (4000,4500),(4500,5000),(5000,5500),(5500,6500)]


def id_mtpa_at_iq(iq):
    """MTPA d-axis current for a given q-axis current."""
    return K - math.sqrt(K * K + 0.5 * iq * iq)


def id_mtpa_at_is(iss):
    """MTPA d-axis current for a given current-vector magnitude."""
    return (2.0 * K - math.sqrt(4.0 * K * K + 3.0 * iss * iss)) / 3.0


def u16(b, i):
    return b[i] | (b[i + 1] << 8)


def band(rpm):
    for lo, hi in BANDS:
        if lo <= rpm < hi:
            return (lo, hi)
    return None


def iter_frames(path):
    """Yield (id, bytes) for the three MCU frames in a PCAN v1.1 text log."""
    with open(path, "r", errors="ignore") as fh:
        for line in fh:
            if not line or line[0] == ";":
                continue
            f = line.split()
            # "  N)  time  Rx  ID  DLC  b0 .. b7"
            if len(f) < 13 or not f[0].endswith(")"):
                continue
            mid = f[3]
            if mid not in ("0705", "0706", "0724"):
                continue
            try:
                yield mid, [int(x, 16) for x in f[5:13]]
            except ValueError:
                continue


def detect_motoring_sign(files):
    """Return (+1|-1|0, n_pos, n_neg) — the Iq sign of forward motoring.

    Orientation-agnostic: counts only samples too large to be regen, inside
    SPORTS+FORWARD. Whichever sign dominates there is motoring.
    """
    npos = nneg = 0
    for path in files:
        mode = None
        for mid, b in iter_frames(path):
            if mid == "0706":
                mode = b[0]
            elif mid == "0705" and mode == 0x08:
                iq = u16(b, 2) - 512.0
                if iq >= IQ_MOTORING_A:
                    npos += 1
                elif iq <= -IQ_MOTORING_A:
                    nneg += 1
    if npos == 0 and nneg == 0:
        return 0, 0, 0
    return (1 if npos >= nneg else -1), npos, nneg


def main(trc_dir, forced_sign=None):
    locus, gstat, vstat = {}, {}, {}
    nclip_vq = nclip_vd = n = 0

    files = sorted(glob.glob(os.path.join(trc_dir, "*.trc")))
    if not files:
        sys.exit(f"no .trc files in {trc_dir}")
    print(f"{len(files)} trc files")

    if forced_sign:
        sign = forced_sign
        print(f"motoring Iq sign FORCED to {sign:+d}\n")
    else:
        sign, npos, nneg = detect_motoring_sign(files)
        if sign == 0:
            sys.exit(f"no samples above |Iq| = {IQ_MOTORING_A:.0f} A — cannot "
                     "determine the motoring sign; pass --sign=+1 or --sign=-1")
        print(f"motoring Iq sign detected: {sign:+d}  "
              f"(|Iq| > {IQ_MOTORING_A:.0f} A: {npos} positive, {nneg} negative)")
        if min(npos, nneg) > 0.05 * max(npos, nneg):
            print("  WARNING: both signs well represented — gearbox orientation may "
                  "have changed mid-log, or the threshold is too low. Check before "
                  "trusting the results.")
        print()

    for path in files:
        speed = mode = vdr = vqr = None
        for mid, b in iter_frames(path):
            if mid == "0706":
                mode, speed = b[0], u16(b, 4) / 10.0
                continue
            if mid == "0724":
                vqr, vdr = u16(b, 2), u16(b, 4)
                continue

            # 0x705 is the primary sample; the other two forward-fill.
            if speed is None or vdr is None or mode != 0x08:
                continue
            iq  = u16(b, 2) - 512.0
            idd = u16(b, 4) - 512.0
            if iq * sign <= 5.0:            # motoring only, either orientation
                continue
            iss = math.hypot(idd, iq)
            if iss < 25.0:                  # angle ill-conditioned below this
                continue
            bd = band(speed * RPM_PER_KMPH)
            if bd is None:
                continue
            n += 1

            ib = int(abs(iq) // 25) * 25
            d = locus.setdefault(bd, {}).setdefault(ib, [0, 0.0, 0.0])
            d[0] += 1; d[1] += idd; d[2] += id_mtpa_at_iq(iq)

            phi  = math.atan2(abs(iq), -idd)
            idm  = id_mtpa_at_is(iss)
            iqm  = math.sqrt(max(iss * iss - idm * idm, 0.0))
            phim = math.atan2(iqm, -idm)
            if phim > 1e-3:
                g = phi / phim
                s = gstat.setdefault(bd, [0, 0.0, 9e9, -9e9])
                s[0] += 1; s[1] += g
                s[2] = min(s[2], g); s[3] = max(s[3], g)

            vq, vd = vqr - 62.0, vdr - 62.0
            if vqr == 0: nclip_vq += 1
            if vdr == 0: nclip_vd += 1
            vm = math.hypot(vd, vq)
            v = vstat.setdefault(bd, [0, 0.0, -9e9, 0, 0.0, 0.0])
            v[0] += 1; v[1] += vm; v[2] = max(v[2], vm)
            if vm > VMAX_SVM: v[3] += 1
            v[4] += vq; v[5] += vd
        print(f"  {os.path.basename(path)[:52]:52s} n={n}", flush=True)

    print(f"\nusable SPORTS forward-motoring samples: {n}")
    print(f"Vq clipped at the -62 V encode floor: {nclip_vq}   Vd: {nclip_vd}\n")

    print("=" * 94)
    print("TEST 1  Id vs MTPA locus   (delta = measured - MTPA; negative = weakening)")
    print("=" * 94)
    print(f"{'band(rpm)':>14} {'|Iq|bin':>8} {'n':>7} {'Id meas':>9} {'Id MTPA':>9} {'delta':>8}")
    for bd in BANDS:
        for ib in sorted(locus.get(bd, {})):
            c, si, sm = locus[bd][ib]
            if c < 100:
                continue
            print(f"{bd[0]:5d}-{bd[1]:<8d} {ib:>8d} {c:>7d} "
                  f"{si/c:>9.1f} {sm/c:>9.1f} {(si-sm)/c:>8.2f}")

    print("\n" + "=" * 94)
    print("TEST 2  actual weakening of the current vector (1.000 = on the MTPA locus)")
    print("=" * 94)
    print(f"{'band(rpm)':>14} {'n':>8} {'mean':>9} {'min':>9} {'max':>9}")
    for bd in BANDS:
        if bd not in gstat:
            continue
        c, s, lo, hi = gstat[bd]
        print(f"{bd[0]:5d}-{bd[1]:<8d} {c:>8d} {s/c:>9.4f} {lo:>9.4f} {hi:>9.4f}")

    print("\n" + "=" * 94)
    print(f"TEST 3  commanded voltage vector vs the SVM circle ({VMAX_SVM:.1f} V at 96 V)")
    print("=" * 94)
    print(f"{'band(rpm)':>14} {'n':>8} {'mean|V|':>9} {'max|V|':>9} "
          f"{'mean Vq':>9} {'mean Vd':>9} {'%over':>8}")
    for bd in BANDS:
        if bd not in vstat:
            continue
        c, sv, mx, no, sq, sd = vstat[bd]
        print(f"{bd[0]:5d}-{bd[1]:<8d} {c:>8d} {sv/c:>9.1f} {mx:>9.1f} "
              f"{sq/c:>9.1f} {sd/c:>9.1f} {100.0*no/c:>7.1f}%")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    opts = [a for a in sys.argv[1:] if a.startswith("--")]
    if len(args) != 1:
        sys.exit('usage: pcan_fw_analyse.py "<folder with .trc files>" [--sign=+1|-1]')
    forced = None
    for o in opts:
        if o.startswith("--sign="):
            forced = int(o.split("=", 1)[1])
            if forced not in (1, -1):
                sys.exit("--sign must be +1 or -1")
    main(args[0], forced)
