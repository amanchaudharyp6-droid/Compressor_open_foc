#!/usr/bin/env python3
"""
Pegasus dyno GUI - send speed ref / F-N-R / throttle voltage over CAN and watch
the debug telemetry.  Needs the DEBUG_CAN_ID + CAN_BASED_REF/DIR/THRV build.
Bus is 250 kbit/s (FDCAN 50 MHz, prescaler 10, 20 tq) - same as angle_autotune_gui.py.

With CAN_SPEED_ONLY=1 in the firmware (vehicle_config.h) it holds Forward + minimum
throttle voltage itself, so the GUI only needs to send the speed reference.

    pip install python-can
    python tools/pegasus_gui.py                                   # PCAN_USBBUS1 @ 250k
    python tools/pegasus_gui.py --interface pcan --channel PCAN_USBBUS1 --bitrate 250000
    python tools/pegasus_gui.py --sim                             # fake controller, no hardware
    python tools/pegasus_gui.py --tune-csv bench_tuning.csv       # also append tuning TX to a CSV
    python tools/pegasus_gui.py --trace-dir D:/logs               # default folder for CAN trace logs

CAN trace log (panel at the bottom): Start/Stop writes every frame the GUI receives AND
transmits to <folder>/pegasus_YYYYmmdd_HHMMSS.trc, a PCAN-View FILEVERSION 1.1 trace
that tools/Parser2.py reads. "Parse last log" then runs Parser2 on it (--fw-rev worktree
--scorecard, output next to the log) and shows the scorecard. The bus is opened without
receive filters, so every ID reaches the log. Rows are written by a separate thread.

TX (standard 11-bit IDs, little-endian u16), resent every 100 ms while ENABLE is on:
    0x102  speed reference, RPM
    0x113  power mode 0=ECO 1=SPORTS
  only when "Send F/N/R + throttle" is ticked (firmware without CAN_SPEED_ONLY):
    0x114  throttle voltage, mV
    0x108  direction  1=F 2=R 3=N
One-shot (PCAN single-shot, no auto-retransmit - a retransmitted reset/autotune frame
would be re-delivered on every reboot):
    0x111 reset (RPM<=10),  0x729 angle autotune (RPM<=10, needs CAN_AUTOTUNE_TRIGGER)

Live tuning (Tuning panel; sent ONLY on an explicit "Send", always the full 8-byte
frame - every frame overwrites all its fields in the firmware, can_rx_handlers.c):
    0x105/0x106/0x107  speed / Id / Iq PI: u16 Kp*0.01, Ki*0.01, Kd*1e-5
                       (firmware also sets the low-speed gains to value * LOW_SPEED_SCALING)
    0x103  Lambda*1e-5, Ld*1e-8, Lq*1e-7, Rs*1e-6 (u16 each)
    0x104  u16 Kfw*1e-3, Kaw*1e-3, u8 M_to_F*0.01, F_to_M*0.01, Rst_Factor*0.01
           (dead while FW_CLOSED_LOOP=1, i.e. on the HIRANGE_TR/TW-7 build)
    0x115  current-sense delay tau, u16 us (firmware clamps at 400 us)
    0x116  fast telemetry, byte0 1=on 0=off
  Values are quantised the way the firmware decodes them (truncated to the LSB);
  each entry shows the value that will actually be applied.

RX (the firmware transmits these as 29-bit EXTENDED ids):
    0x401  rpm_ref, rpm, Id+512, Iq+512
    0x403  Vbus*100, Vaux*100, Tmtr+50, Tmc+50, err, torque+80
    0x405  throttle_state, cS, overrun, pw_state, offset*1000, thrV mV
    0x406  bytes 0-1 = applied current-sense tau, us (FOC_ISENSE_DELAY_COMP build)
    0x420  fast tlm (when on): Id_ref, Iq_ref, Id, Iq   int16, 0.01 A
    0x421  fast tlm (when on): Vd, Vq (0.01 V), rpm (1 rpm, signed), Vdc (0.01 V)
    0x422  fast tlm (when on): Vd_FF, Vq_FF, Vq_FF before clamp, FF limit 0.577*Vbus (int16, 0.01 V)
    0x40A  encoder diagnostics, 10 Hz (logged only): count_at_z, z_count, Count_From_Duty (u16)
    0x72A  (std) autotune status 0=running 1=done 2=EEPROM store failed
"""
import argparse
import csv
import datetime
import json
import math
import os
import queue
import struct
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Callable, NamedTuple, Optional

import can

TX_PERIOD_S = 0.1
RX_STALE_S = 2.0          # firmware sends telemetry on a 500 ms cycle
ID_SPEED, ID_DIR, ID_THRV, ID_PWR = 0x102, 0x108, 0x114, 0x113
ID_RESET, ID_AUTOTUNE = 0x111, 0x729
ID_MOTOR, ID_FW, ID_SPEED_PI, ID_ID_PI, ID_IQ_PI = 0x103, 0x104, 0x105, 0x106, 0x107
ID_TAU, ID_FAST_TLM = 0x115, 0x116

DIR_CODE = {"F": 1, "R": 2, "N": 3}
ERRORS = ["NO_ERROR", "CURR_SENS", "BUS_OV", "BUS_UV", "AUX_OV", "AUX_UV", "ID_IQ_OC",
          "PHASE_CURR", "MTR_OT", "MTC_OT", "MTR_NTC_DISC", "MTC_NTC_DISC", "EEPROM_WR",
          "EEPROM_RD", "ENC_A_B", "ENC_Z", "ENC_PWM", "ENC_5V", "STALL"]
SESSIONS = ["INIT", "ANGLE_CALIB", "ANGLE_CALIB_DONE", "ANGLE_OFFSET_STORE", "OPEN_FOC_START",
            "CURR_SENS_CALIB", "FOC_START", "CONT_ERROR", "RESET_STATE"]
THR_STATES = {1: "Forward", 2: "Reverse", 3: "Neutral", 4: "Throttle_Error"}

# ---------------------------------------------------------------------------
# Live tuning frames - pure functions, no CAN/Tk (tested by
# tools/host_tests/test_gui_frames.py). Layouts from Core/Src/can_rx_handlers.c.
# ---------------------------------------------------------------------------
LOW_SPEED_SCALING = 0.8       # Core/Inc/motor_profiles/motor_hirange_tr.h:117
LOW_SPEED_SWITCH_RPM = 400    # LOW_SPEED_TO_HIGH_SPEED_SWITCH_RPM, motor_hirange_tr.h:113
TAU_MAX_US = 400              # FOC_ISENSE_DELAY_MAX_S, Core/Inc/vehicle_config.h:566
TAU_CONFIRM_RPM = 3000        # ask before sending 0x115 above this speed (or speed unknown)
GAIN_CONFIRM_RPM = 6000       # ask before sending PI / motor / FW frames above this speed


def f32(x):
    """Round to IEEE single precision (the firmware's float)."""
    return struct.unpack("<f", struct.pack("<f", float(x)))[0]


class TField(NamedTuple):
    key: str
    label: str
    lsb: float                        # firmware: value = (float)raw * lsb  (float32)
    nbytes: int                       # 2 = u16 LE, 1 = u8; packed in order from byte 0
    max_value: Optional[float] = None  # tighter limit than the raw range (firmware clamp)


class TFrame(NamedTuple):
    arb_id: int
    title: str
    fields: tuple
    low_scaled: bool = False                 # firmware also sets *_low = value * LOW_SPEED_SCALING
    confirm_rpm: Optional[float] = None      # confirm above this |rpm| (None = never)
    check: Optional[Callable] = None         # frame-level check on the raw values, raises ValueError


def _check_fw(raw):
    # same invariants as the #error guards in motor_hirange_tr.h:127-132
    if not raw["F_to_M"] < raw["M_to_F"]:
        raise ValueError("F_to_M must be < M_to_F (FW hysteresis)")
    if not raw["Rst_Factor"] <= raw["F_to_M"]:
        raise ValueError("Rst_Factor must be <= F_to_M")


PI_FIELDS = (TField("Kp", "Kp", 1e-2, 2), TField("Ki", "Ki", 1e-2, 2), TField("Kd", "Kd", 1e-5, 2))


TUNE_FRAMES = {
    ID_SPEED_PI: TFrame(ID_SPEED_PI, "Speed PI", PI_FIELDS, True, GAIN_CONFIRM_RPM),
    ID_ID_PI: TFrame(ID_ID_PI, "Id PI", PI_FIELDS, True, GAIN_CONFIRM_RPM),
    ID_IQ_PI: TFrame(ID_IQ_PI, "Iq PI", PI_FIELDS, True, GAIN_CONFIRM_RPM),
    ID_MOTOR: TFrame(ID_MOTOR, "Motor params", (
        TField("Lambda", "Lambda (Wb)", 1e-5, 2), TField("Ld", "Ld (H)", 1e-8, 2),
        TField("Lq", "Lq (H)", 1e-7, 2), TField("Rs", "Rs (ohm)", 1e-6, 2)),
        False, GAIN_CONFIRM_RPM),
    ID_FW: TFrame(ID_FW, "FW params", (
        TField("Kfw", "Kfw", 1e-3, 2), TField("Kaw", "Kaw", 1e-3, 2),
        TField("M_to_F", "M_to_F", 1e-2, 1), TField("F_to_M", "F_to_M", 1e-2, 1),
        TField("Rst_Factor", "Rst_Factor", 1e-2, 1)),
        False, GAIN_CONFIRM_RPM, _check_fw),
    ID_TAU: TFrame(ID_TAU, "Isense delay tau", (
        TField("tau_us", "tau (us)", 1.0, 2, TAU_MAX_US),), False, TAU_CONFIRM_RPM),
    ID_FAST_TLM: TFrame(ID_FAST_TLM, "Fast telemetry", (
        TField("on", "on", 1.0, 1, 1),), False, None),
}

# Firmware boot values (HIRANGE_TR profile = TW-7 bench, motor_select.h:11). The
# firmware boots from the can_d initialiser, Core/Src/main.c:108, which takes:
BOOT = {
    ID_SPEED_PI: {"Kp": 4.0, "Ki": 8.0, "Kd": 1e-5},     # SPEED_KP/KI/KD  motor_hirange_tr.h:194-196
    ID_ID_PI: {"Kp": 0.17, "Ki": 3.5, "Kd": 1e-5},       # ID_KP/KI/KD     motor_hirange_tr.h:203-205
    ID_IQ_PI: {"Kp": 0.15, "Ki": 4.0, "Kd": 1e-5},       # IQ_KP/KI/KD     motor_hirange_tr.h:212-214
    ID_MOTOR: {"Lambda": 0.01121, "Ld": 0.0000635,       # LAMBDA/LD/LQ/RS motor_hirange_tr.h:23-26
               "Lq": 0.0000879, "Rs": 0.00616},
    ID_FW: {"Kfw": 0.002, "Kaw": 12.0, "M_to_F": 0.85,   # FW_KFW/KAW/M2F_PCT/F2M_PCT/RST_FACTOR_PCT
            "F_to_M": 0.55, "Rst_Factor": 0.45},         #   motor_hirange_tr.h:121-125
    ID_TAU: {"tau_us": 0},                               # FOC_ISENSE_DELAY_S vehicle_config.h:565
    ID_FAST_TLM: {"on": 0},                              # can_d.fast_tlm_on not initialised -> 0
}


def quantise(field, value):
    """Engineering value (number or text) -> raw integer, truncated to the LSB like the
    spec'd firmware scaling (0.136 at LSB 0.01 -> 13). Raises ValueError if not sendable."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{field.key}: not a number ({value!r})") from None
    if not math.isfinite(v):
        raise ValueError(f"{field.key}: not a finite number")
    if v < 0:
        raise ValueError(f"{field.key}: negative values are not allowed")
    raw = math.floor(v / field.lsb + 1e-6)       # 1e-6 LSB: 0.29/0.01 = 28.999... -> 29
    raw_max = (1 << (8 * field.nbytes)) - 1
    if raw > raw_max:
        raise ValueError(f"{field.key}: {v:g} above the frame maximum {fmt(raw_max * field.lsb)}")
    if field.max_value is not None and raw * field.lsb > field.max_value:
        raise ValueError(f"{field.key}: {v:g} above the firmware limit {field.max_value:g}")
    return raw


def applied(field, raw):
    """Value the firmware stores for this raw field (float32 decode)."""
    return f32(f32(raw) * f32(field.lsb))


def low_speed(value):
    """Low-speed gain the firmware sets alongside (value * LOW_SPEED_SCALING, float32)."""
    return f32(f32(value) * f32(LOW_SPEED_SCALING))


def encode_frame(frame, values):
    """All field values (dict key -> number/text) -> (8-byte payload, raw dict).
    Every field is required: the firmware overwrites all of them."""
    missing = [f.key for f in frame.fields if f.key not in values]
    if missing:
        raise ValueError(f"missing field(s) {', '.join(missing)} - a frame overwrites all fields")
    raw = {f.key: quantise(f, values[f.key]) for f in frame.fields}
    if frame.check:
        frame.check(raw)
    data, pos = bytearray(8), 0       # always DLC 8: the firmware RX buffer keeps stale bytes
    for f in frame.fields:
        data[pos:pos + f.nbytes] = raw[f.key].to_bytes(f.nbytes, "little")
        pos += f.nbytes
    return bytes(data), raw


def decode_frame(frame, data):
    """Payload -> values the firmware will apply (dict key -> float32)."""
    out, pos = {}, 0
    for f in frame.fields:
        out[f.key] = applied(f, int.from_bytes(data[pos:pos + f.nbytes], "little"))
        pos += f.nbytes
    return out


def decode_fast_tlm(arb_id, data):
    """0x420/0x421/0x422 (fdcan_AL.c Fast_Tlm_Tick_1ms) -> dict, or {} for other ids."""
    a, b, c, d = struct.unpack_from("<hhhh", data)
    if arb_id == 0x420:
        return {"Id_ref": a / 100, "Iq_ref": b / 100, "Id": c / 100, "Iq": d / 100}
    if arb_id == 0x421:
        return {"Vd": a / 100, "Vq": b / 100, "rpm": c, "Vdc": d / 100}
    if arb_id == 0x422:
        return {"Vd_FF": a / 100, "Vq_FF": b / 100, "Vq_FF_unsat": c / 100, "FF_lim": d / 100}
    return {}


def fmt(v):
    return f"{v:.6g}"


def describe(frame, data):
    """Decoded firmware values for a log line."""
    vals = decode_frame(frame, data)
    txt = " ".join(f"{k}={fmt(v)}" for k, v in vals.items())
    if frame.low_scaled:
        txt += "  (low: " + " ".join(f"{k}={fmt(low_speed(v))}" for k, v in vals.items()) + ")"
    return txt


def send_single_shot(bus, arb_id, data):
    """One transmission, no auto-retransmit (PCAN only; same trick as angle_autotune_gui.py)."""
    if hasattr(bus, "m_objPCANBasic") and hasattr(bus, "m_PcanHandle"):
        from can.interfaces.pcan.pcan import TPCANMsg, PCAN_MESSAGE_STANDARD
        m = TPCANMsg()
        m.ID, m.LEN = arb_id, len(data)
        m.MSGTYPE = PCAN_MESSAGE_STANDARD.value | 0x08      # PCAN_MESSAGE_SINGLESHOT
        for i, b in enumerate(data):
            m.DATA[i] = b
        st = bus.m_objPCANBasic.Write(bus.m_PcanHandle, m)
        if int(st) != 0:
            raise RuntimeError(f"PCAN single-shot write failed (0x{int(st):X})")
        return
    bus.send(can.Message(arbitration_id=arb_id, is_extended_id=False, data=data))


def name(table, i):
    return table[i] if 0 <= i < len(table) else str(i)


# ---------------------------------------------------------------------------
# CAN trace log: PCAN-View FILEVERSION 1.1 .trc, the layout tools/Parser2.py
# reads. No Tk, tested by tools/host_tests/test_trace_log.py.
# ---------------------------------------------------------------------------
TRC_HEADER = (
    ";$FILEVERSION=1.1",
    ";$STARTTIME={ole:.10f}",
    ";",
    ";   Start time: {start}",
    ";   Generated by pegasus_gui.py (PCAN-View 1.1 layout)",
    ";",
    ";   Message Number",
    ";   |         Time Offset (ms)",
    ";   |         |        Type",
    ";   |         |        |        ID (hex)",
    ";   |         |        |        |     Data Length",
    ";   |         |        |        |     |   Data Bytes (hex) ...",
    ";   |         |        |        |     |   |",
    ";---+--   ----+----  --+--  ----+---  +  -+ -- -- -- -- -- -- --",
)
_TRC_IDLE = object()


class TraceWriter:
    """Writes frames to a .trc from its own thread: log_rx()/log_tx() only put a tuple on a
    queue, so the receive loop and the Tk thread never wait for the disk. The file is
    flushed every FLUSH_S, so a crash loses at most the last second.

    Offsets are ms since `start`, 0.1 ms resolution. Received frames keep the driver's
    (hardware) spacing: the driver clock is mapped onto the PC clock once, using the
    smallest delay seen, and made non-decreasing; sent frames are stamped with the PC
    clock when they were handed to the bus."""
    QUEUE_MAX = 200000          # ~8 min of 400 frames/s; beyond that frames are counted, not stored
    FLUSH_S = 1.0

    def __init__(self, path, start=None):
        self.path = path
        self.start = time.time() if start is None else start
        self.frames = 0         # rows written to the file
        self.dropped = 0        # frames not written (backlog full or the file failed)
        self.error = None       # first write error, text
        self._q = queue.Queue(self.QUEUE_MAX)
        self._fh = None
        self._thread = None
        self._clk = None        # PC clock minus driver clock, smallest seen
        self._last_rx = 0.0

    def open(self):
        dt = datetime.datetime.fromtimestamp(self.start)
        ole = (dt - datetime.datetime(1899, 12, 30)).total_seconds() / 86400.0
        stamp = f"{dt.month}/{dt.day}/{dt.year} {dt:%H:%M:%S}.{dt.microsecond // 1000:03d}.0"
        fh = open(self.path, "w", newline="", encoding="ascii")
        try:
            fh.write("".join(h.format(ole=ole, start=stamp) + "\r\n" for h in TRC_HEADER))
            fh.flush()
        except OSError:
            fh.close()
            raise
        self._fh = fh
        self._thread = threading.Thread(target=self._run, name="trc-writer", daemon=True)
        self._thread.start()
        return self

    @staticmethod
    def format_row(n, offset_ms, arb_id, extended, data, tx=False):
        """One PCAN-View 1.1 row: standard ids as 4 hex digits, extended as 8."""
        idtxt = f"{arb_id:08X}" if extended else f"{arb_id:04X}"
        return (f"{n:6d}){offset_ms:12.1f}  {'Tx' if tx else 'Rx'}  {idtxt:>11}  {len(data)}  "
                + "".join(f"{b:02X} " for b in data) + "\r\n")

    def log_rx(self, arb_id, data, extended, hw_ts=None, now=None):
        now = time.time() if now is None else now
        if hw_ts:
            delta = now - hw_ts
            if self._clk is None or delta < self._clk:
                self._clk = delta
            ts = max(hw_ts + self._clk, self._last_rx)
        else:
            ts = max(now, self._last_rx)
        self._last_rx = ts
        self._put(ts, arb_id, data, extended, False)

    def log_tx(self, arb_id, data, extended, now=None):
        self._put(time.time() if now is None else now, arb_id, data, extended, True)

    def _put(self, ts, arb_id, data, extended, tx):
        try:
            self._q.put_nowait((ts, arb_id, bytes(data), extended, tx))
        except queue.Full:
            self.dropped += 1

    @property
    def backlog(self):
        return self._q.qsize()

    def _run(self):
        last_flush = time.monotonic()
        while True:
            try:
                item = self._q.get(timeout=0.25)
            except queue.Empty:
                item = _TRC_IDLE
            if item is None:
                break
            if item is not _TRC_IDLE:
                if self.error is not None:
                    self.dropped += 1
                else:
                    ts, arb_id, data, extended, tx = item
                    try:
                        self._fh.write(self.format_row(self.frames + 1, max(0.0, (ts - self.start) * 1000.0),
                                                       arb_id, extended, data, tx))
                        self.frames += 1
                    except (OSError, ValueError) as exc:
                        self.error = str(exc)
                        self.dropped += 1
            if time.monotonic() - last_flush >= self.FLUSH_S:
                last_flush = time.monotonic()
                self._flush()
        self._flush()

    def _flush(self):
        try:
            self._fh.flush()
        except (OSError, ValueError) as exc:
            if self.error is None:
                self.error = str(exc)

    def close(self, timeout=10.0):
        """Write what is still queued, flush and close. Returns the rows written."""
        if self._thread is not None:
            try:
                self._q.put(None, timeout=timeout)
                self._thread.join(timeout)
            except queue.Full:
                pass
            self._thread = None
        if self._fh is not None:
            try:
                self._fh.close()
            except OSError as exc:
                if self.error is None:
                    self.error = str(exc)
            self._fh = None
        return self.frames


def sim_controller(bus_kwargs, stop):
    """Stand-in for the controller so the GUI can be exercised without hardware."""
    bus = can.Bus(**bus_kwargs)
    rpm = ref = 0.0
    thr = tau = fast = z_count = 0
    last = last_fast = time.time()
    while not stop.is_set():
        m = bus.recv(timeout=0.005)
        if m is not None and not m.is_extended_id and len(m.data) >= 1:
            if m.arbitration_id == ID_SPEED and len(m.data) >= 2:
                ref = float(struct.unpack_from("<H", m.data)[0])
            elif m.arbitration_id == ID_THRV and len(m.data) >= 2:
                thr = struct.unpack_from("<H", m.data)[0]
            elif m.arbitration_id == ID_TAU and len(m.data) >= 2:
                tau = min(struct.unpack_from("<H", m.data)[0], TAU_MAX_US)
            elif m.arbitration_id == ID_FAST_TLM:
                fast = 1 if m.data[0] == 1 else 0
        now = time.time()
        rpm += (ref - rpm) * min(1.0, (now - last) * 2.0)
        if fast and now - last_fast > 0.05:
            last_fast = now - 0.005             # just switched on, or fell far behind
        while fast and now - last_fast >= 0.005:    # catch up like the real queue does after a stall
            last_fast += 0.005
            bus.send(can.Message(arbitration_id=0x420, is_extended_id=True, data=struct.pack(
                "<hhhh", -100, int(rpm / 2), -95, int(rpm / 2) + 20)))
            bus.send(can.Message(arbitration_id=0x421, is_extended_id=True, data=struct.pack(
                "<hhhh", -150, int(rpm), int(rpm), 5100)))
            # feed-forward: Vq FF grows with speed and sits on the 0.577*Vbus clamp above ~980 rpm
            vq_unsat, lim = int(rpm * 3.0), int(0.57735 * 5100)
            bus.send(can.Message(arbitration_id=0x422, is_extended_id=True, data=struct.pack(
                "<hhhh", -int(rpm * 0.8), min(vq_unsat, lim), vq_unsat, lim)))
        if now - last >= 0.5:
            last = now
            z_count += 1
            bus.send(can.Message(arbitration_id=0x40A, is_extended_id=True, data=struct.pack(
                "<HHHH", 1234, z_count & 0xFFFF, 3000, 0)))
            bus.send(can.Message(arbitration_id=0x401, is_extended_id=True, data=struct.pack(
                "<HHHH", int(ref), int(rpm), 512, 512 + int(rpm / 20))))
            bus.send(can.Message(arbitration_id=0x403, is_extended_id=True, data=struct.pack(
                "<HHBBBB", 7680, 1210, 75, 80, 0, 80 + int(rpm / 100))))
            bus.send(can.Message(arbitration_id=0x405, is_extended_id=True, data=struct.pack(
                "<BBBBHH", 1, 6, 0, 0, 0, thr)))
            bus.send(can.Message(arbitration_id=0x406, is_extended_id=True, data=struct.pack(
                "<HHHH", tau, 12000, 0, 0)))
    bus.shutdown()


FAST_ROWS = {"Id_ref": "Id_ref (A)", "Iq_ref": "Iq_ref (A)", "Id": "Id 5ms (A)",
             "Iq": "Iq 5ms (A)", "Vd": "Vd (V)", "Vq": "Vq (V)", "rpm": "RPM (signed)",
             "Vdc": "Vdc 5ms (V)", "Vd_FF": "Vd FF (V)", "Vq_FF": "Vq FF (V)",
             "Vq_FF_unsat": "Vq FF unclamped (V)", "FF_lim": "FF clamp limit (V)"}
PARSER2 = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Parser2.py")
PARSE_TIMEOUT_S = 600
SETTINGS = os.path.join(os.path.expanduser("~"), ".pegasus_gui.json")   # remembered trace folder


class App:
    def __init__(self, root, bus, max_rpm, tune_csv=None, trace_dir=None):
        self.root, self.bus = root, bus
        self.lock = threading.Lock()
        self.tx_on = False
        self.stop_evt = threading.Event()
        self.rx = {}              # name -> value
        self.rx_time = 0.0
        self.note = ""
        self.max_rpm = max_rpm
        self.rpm, self.rpm_time = None, 0.0      # last |rpm| from 0x401 / 0x421
        self.fast_n, self.fast_time = 0, 0.0     # 0x420 count / last arrival
        self.fast_seen = (0, time.time())
        self.tune_csv = tune_csv
        self.groups = {}          # arb_id -> {"vars": {key: StringVar}, "sent": Label}
        self.trace = None         # TraceWriter while logging (read by the rx and tx threads)
        self.last_trace = None    # path of the last closed trace
        self.trace_dir_fixed = trace_dir is not None    # --trace-dir: do not touch the remembered folder
        self.parse_q = queue.Queue()
        self.parsing = False

        root.title("Pegasus dyno GUI")
        root.protocol("WM_DELETE_WINDOW", self.close)
        f = ttk.Frame(root, padding=10)
        f.grid()

        # ---- command panel ----
        c = ttk.LabelFrame(f, text="Command", padding=8)
        c.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        ttk.Label(c, text=f"Speed ref (RPM, max {int(max_rpm)})").grid(row=0, column=0, sticky="w")
        self.speed = tk.IntVar(value=0)
        ttk.Scale(c, from_=0, to=max_rpm, variable=self.speed, length=240,
                  command=lambda v: self.speed.set(int(float(v)))).grid(row=1, column=0, columnspan=2)
        e = ttk.Spinbox(c, from_=0, to=max_rpm, textvariable=self.speed, width=8)
        e.grid(row=0, column=1, sticky="e")

        ttk.Label(c, text="Throttle voltage (mV)").grid(row=2, column=0, sticky="w", pady=(8, 0))
        self.thrv = tk.IntVar(value=800)
        ttk.Spinbox(c, from_=0, to=5000, increment=50, textvariable=self.thrv,
                    width=8).grid(row=2, column=1, sticky="e", pady=(8, 0))

        ttk.Label(c, text="Direction").grid(row=3, column=0, sticky="w", pady=(8, 0))
        self.dir = tk.StringVar(value="N")
        d = ttk.Frame(c)
        d.grid(row=3, column=1, sticky="e", pady=(8, 0))
        for k in "FNR":
            ttk.Radiobutton(d, text=k, value=k, variable=self.dir).pack(side="left")

        ttk.Label(c, text="Power mode").grid(row=4, column=0, sticky="w", pady=(8, 0))
        self.pwr = tk.IntVar(value=0)
        p = ttk.Frame(c)
        p.grid(row=4, column=1, sticky="e", pady=(8, 0))
        ttk.Radiobutton(p, text="ECO", value=0, variable=self.pwr).pack(side="left")
        ttk.Radiobutton(p, text="SPORTS", value=1, variable=self.pwr).pack(side="left")

        self.legacy = tk.BooleanVar(value=False)
        ttk.Checkbutton(c, text="Send F/N/R + throttle (firmware without CAN_SPEED_ONLY)",
                        variable=self.legacy).grid(row=5, column=0, columnspan=2, sticky="w", pady=(8, 0))
        self.tx_btn = tk.Button(c, text="ENABLE TX (off)", bg="#c9c9c9", command=self.toggle_tx)
        self.tx_btn.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(12, 2))
        tk.Button(c, text="STOP  (speed 0) :()", bg="#e06666", fg="white",
                  command=lambda: self.speed.set(0)).grid(row=7, column=0, columnspan=2, sticky="ew")
        ttk.Button(c, text="Reset controller (0x111)", command=self.do_reset).grid(
            row=8, column=0, columnspan=2, sticky="ew", pady=(10, 2))
        ttk.Button(c, text="Angle autotune (0x729)", command=self.do_autotune).grid(
            row=9, column=0, columnspan=2, sticky="ew")

        # ---- telemetry panel ----
        t = ttk.LabelFrame(f, text="Telemetry", padding=8)
        t.grid(row=0, column=1, sticky="nsew")
        self.lbl = {}
        rows = ["RPM ref", "RPM", "Id (A)", "Iq (A)", "Vbus (V)", "Vaux (V)", "Motor T (C)",
                "Ctrl T (C)", "Torque (Nm)", "Error", "State (cS)", "Throttle state",
                "Throttle V (mV)", "Mode", "Overrun", "Tau applied (us)"]
        for i, r in enumerate(rows):
            ttk.Label(t, text=r).grid(row=i, column=0, sticky="w")
            self.lbl[r] = ttk.Label(t, text="-", width=16, anchor="e")
            self.lbl[r].grid(row=i, column=1, sticky="e")
        i = len(rows)
        ttk.Separator(t).grid(row=i, column=0, columnspan=2, sticky="ew", pady=4)
        ttk.Label(t, text="Fast tlm 0x420/1/2").grid(row=i + 1, column=0, sticky="w")
        self.fast_lbl = ttk.Label(t, text="-", width=16, anchor="e")
        self.fast_lbl.grid(row=i + 1, column=1, sticky="e")
        for j, r in enumerate(FAST_ROWS.values()):
            ttk.Label(t, text=r).grid(row=i + 2 + j, column=0, sticky="w")
            self.lbl[r] = ttk.Label(t, text="-", width=16, anchor="e")
            self.lbl[r].grid(row=i + 2 + j, column=1, sticky="e")

        # ---- tuning panel ----
        self.build_tuning(f)

        # ---- log ----
        lg = ttk.LabelFrame(f, text="Tuning log (sent frames)", padding=4)
        lg.grid(row=1, column=0, columnspan=3, sticky="nsew", pady=(8, 0))
        self.log_txt = tk.Text(lg, height=7, width=140, state="disabled", font=("Consolas", 9))
        sb = ttk.Scrollbar(lg, command=self.log_txt.yview)
        self.log_txt.config(yscrollcommand=sb.set)
        self.log_txt.grid(row=0, column=0, sticky="nsew")
        sb.grid(row=0, column=1, sticky="ns")

        # ---- CAN trace log ----
        tl = ttk.LabelFrame(f, text="CAN trace log (every received and sent frame, PCAN-View .trc)",
                            padding=6)
        tl.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        ttk.Label(tl, text="Folder").grid(row=0, column=0, sticky="w")
        self.trace_dir = tk.StringVar(value=trace_dir or self.load_trace_dir())
        ttk.Entry(tl, textvariable=self.trace_dir, width=70).grid(row=0, column=1, padx=4, sticky="ew")
        ttk.Button(tl, text="Browse...", command=self.browse_trace_dir).grid(row=0, column=2)
        self.trace_start_btn = tk.Button(tl, text="Start log", bg="#6aa84f", command=self.start_trace)
        self.trace_start_btn.grid(row=0, column=3, padx=(10, 2))
        self.trace_stop_btn = tk.Button(tl, text="Stop log", bg="#c9c9c9", state="disabled",
                                        command=self.stop_trace)
        self.trace_stop_btn.grid(row=0, column=4, padx=2)
        self.parse_btn = ttk.Button(tl, text="Parse last log", state="disabled", command=self.parse_last)
        self.parse_btn.grid(row=0, column=5, padx=(10, 0))
        self.trace_lbl = ttk.Label(tl, text="not logging", foreground="#666")
        self.trace_lbl.grid(row=1, column=0, columnspan=6, sticky="w", pady=(4, 0))
        tl.columnconfigure(1, weight=1)

        self.status = ttk.Label(f, text="", foreground="#666")
        self.status.grid(row=3, column=0, columnspan=3, sticky="w", pady=(6, 0))
        if tune_csv:
            self.log(f"tuning frames are also appended to {os.path.abspath(tune_csv)}")
        self.log(f"CAN bus {getattr(bus, 'channel_info', '?')}, receive filters: "
                 f"{bus.filters if getattr(bus, 'filters', None) else 'none (all ids accepted)'}")

        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.tx_thread = threading.Thread(target=self.tx_loop, daemon=True)
        self.rx_thread.start()
        self.tx_thread.start()
        self.refresh()

    # ---- tuning panel ----
    def build_tuning(self, parent):
        tp = ttk.LabelFrame(parent, text="Tuning (nothing is sent until you press Send)", padding=8)
        tp.grid(row=0, column=2, sticky="nsew", padx=(8, 0))
        nb = ttk.Notebook(tp)
        nb.grid(row=0, column=0, sticky="nsew")

        tab = ttk.Frame(nb, padding=6)
        nb.add(tab, text="Isense tau / tlm")
        g = self.build_group(tab, TUNE_FRAMES[ID_TAU], 0,
                             note=f"firmware clamps at {TAU_MAX_US} us; confirm above "
                                  f"{TAU_CONFIRM_RPM} rpm")
        pr = ttk.Frame(g)
        pr.grid(row=20, column=0, columnspan=3, sticky="w", pady=(4, 0))
        ttk.Label(pr, text="Presets:").pack(side="left")
        for us in (0, 150, 210, 250):
            ttk.Button(pr, text=f"{us} us", width=7,
                       command=lambda us=us: self.set_fields(ID_TAU, tau_us=us)).pack(side="left")

        ft = ttk.LabelFrame(tab, text="0x116 Fast telemetry (200 Hz, ~85 kbit/s)", padding=6)
        ft.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        self.groups[ID_FAST_TLM] = {"vars": {"on": tk.StringVar(value="0")}}
        tk.Button(ft, text="Fast tlm ON", bg="#6aa84f",
                  command=lambda: self.send_fast_tlm(True)).grid(row=0, column=0, padx=(0, 4))
        tk.Button(ft, text="Fast tlm OFF", bg="#c9c9c9",
                  command=lambda: self.send_fast_tlm(False)).grid(row=0, column=1)
        self.groups[ID_FAST_TLM]["sent"] = ttk.Label(ft, text="not sent (firmware boots OFF)",
                                                     foreground="#666")
        self.groups[ID_FAST_TLM]["sent"].grid(row=1, column=0, columnspan=3, sticky="w")

        tab = ttk.Frame(nb, padding=6)
        nb.add(tab, text="Speed PI")
        g = self.build_group(tab, TUNE_FRAMES[ID_SPEED_PI], 0, note=self.low_note())
        pr = ttk.Frame(g)
        pr.grid(row=20, column=0, columnspan=3, sticky="w", pady=(4, 0))
        ttk.Label(pr, text="Presets:").pack(side="left")
        for kp, ki in ((4, 8), (2, 4)):
            ttk.Button(pr, text=f"Kp {kp} / Ki {ki}",
                       command=lambda kp=kp, ki=ki: self.set_fields(ID_SPEED_PI, Kp=kp, Ki=ki)
                       ).pack(side="left")

        tab = ttk.Frame(nb, padding=6)
        nb.add(tab, text="Id / Iq PI")
        self.build_group(tab, TUNE_FRAMES[ID_ID_PI], 0)
        self.build_group(tab, TUNE_FRAMES[ID_IQ_PI], 1, note=self.low_note())

        tab = ttk.Frame(nb, padding=6)
        nb.add(tab, text="Motor / FW")
        self.build_group(tab, TUNE_FRAMES[ID_MOTOR], 0)
        self.build_group(tab, TUNE_FRAMES[ID_FW], 1,
                         note="UNUSED while FW_CLOSED_LOOP=1 (vehicle_config.h:142, TW-7 build):\n"
                              "the closed-loop FW uses FW_CL_* constants instead")

    @staticmethod
    def low_note():
        return (f"Firmware also sets the low-speed gains (< {LOW_SPEED_SWITCH_RPM} rpm)\n"
                f"to value x {LOW_SPEED_SCALING} (LOW_SPEED_SCALING) - shown as 'low'.")

    def build_group(self, parent, frame, row, note=None):
        g = ttk.LabelFrame(parent, text=f"0x{frame.arb_id:03X} {frame.title}", padding=6)
        g.grid(row=row, column=0, sticky="ew", pady=(0 if row == 0 else 8, 0))
        grp = {"vars": {}, "prev": {}}
        for i, fld in enumerate(frame.fields):
            ttk.Label(g, text=fld.label).grid(row=i, column=0, sticky="w")
            var = tk.StringVar(value=fmt(BOOT[frame.arb_id][fld.key]))
            ttk.Entry(g, textvariable=var, width=11).grid(row=i, column=1, padx=4)
            pv = tk.Label(g, text="", width=34, anchor="w")
            pv.grid(row=i, column=2, sticky="w")
            grp["vars"][fld.key], grp["prev"][fld.key] = var, pv
            var.trace_add("write", lambda *_, fr=frame, fl=fld: self.preview(fr, fl))
        b = ttk.Frame(g)
        b.grid(row=21, column=0, columnspan=3, sticky="ew", pady=(6, 0))
        ttk.Button(b, text="Revert to boot values",
                   command=lambda: self.set_fields(frame.arb_id, **BOOT[frame.arb_id])).pack(side="left")
        tk.Button(b, text=f"Send 0x{frame.arb_id:03X}", bg="#f6b26b",
                  command=lambda: self.tune_send(frame)).pack(side="right")
        grp["sent"] = ttk.Label(g, text="not sent from this GUI session", foreground="#666")
        grp["sent"].grid(row=22, column=0, columnspan=3, sticky="w")
        if note:
            ttk.Label(g, text=note, foreground="#666").grid(row=23, column=0, columnspan=3, sticky="w")
        self.groups[frame.arb_id] = grp
        for fld in frame.fields:
            self.preview(frame, fld)
        return g

    def preview(self, frame, fld):
        var, pv = self.groups[frame.arb_id]["vars"][fld.key], self.groups[frame.arb_id]["prev"][fld.key]
        try:
            raw = quantise(fld, var.get())
        except ValueError as exc:
            pv.config(text=f"REFUSED: {str(exc).split(': ', 1)[-1]}", fg="#c00000")
            return
        v = applied(fld, raw)
        txt = f"-> sends {fmt(v)}"
        if frame.low_scaled:
            txt += f"   low {fmt(low_speed(v))}"
        typed = float(var.get())
        if raw == 0 and typed != 0:
            txt += "  (rounds to 0!)"
        # orange when quantisation changes what was typed (beyond float32 rounding)
        exact = math.isclose(typed, v, rel_tol=1e-6, abs_tol=0.0)
        pv.config(text=txt, fg="#333" if exact else "#b45f06")

    def set_fields(self, arb_id, **vals):
        """Fill entries (presets / revert). Does NOT send."""
        for k, v in vals.items():
            self.groups[arb_id]["vars"][k].set(fmt(v))

    def current_rpm(self):
        """Last |rpm| from telemetry, or None if none / stale."""
        with self.lock:
            rpm, t = self.rpm, self.rpm_time
        if rpm is None or time.time() - t > RX_STALE_S:
            return None
        return abs(rpm)

    def tune_send(self, frame):
        grp = self.groups[frame.arb_id]
        values = {k: v.get() for k, v in grp["vars"].items()}
        try:
            data = encode_frame(frame, values)[0]
        except ValueError as exc:
            messagebox.showerror("Not sent", f"0x{frame.arb_id:03X} {frame.title} NOT sent:\n{exc}")
            self.log(f"REFUSED 0x{frame.arb_id:03X} {frame.title}: {exc}")
            return
        rpm = self.current_rpm()
        if frame.confirm_rpm is not None and (rpm is None or rpm > frame.confirm_rpm):
            spd = "speed UNKNOWN (no live telemetry)" if rpm is None else f"motor at {rpm:.0f} rpm"
            if not messagebox.askyesno(
                    "Confirm torque step",
                    f"{spd} (> {frame.confirm_rpm} rpm check).\n\nSend 0x{frame.arb_id:03X} "
                    f"{frame.title}?\n{describe(frame, data)}\n\nThis applies instantly and is a "
                    f"torque step at speed.", icon="warning"):
                self.log(f"cancelled 0x{frame.arb_id:03X} {frame.title} ({spd})")
                return
        self.transmit_tune(frame, data, rpm)

    def send_fast_tlm(self, on):
        self.groups[ID_FAST_TLM]["vars"]["on"].set("1" if on else "0")
        data, _ = encode_frame(TUNE_FRAMES[ID_FAST_TLM], {"on": 1 if on else 0})
        self.transmit_tune(TUNE_FRAMES[ID_FAST_TLM], data, self.current_rpm())

    def transmit_tune(self, frame, data, rpm):
        err = None
        try:
            self.bus.send(can.Message(arbitration_id=frame.arb_id, is_extended_id=False, data=data))
            self.trace_tx(frame.arb_id, data)
        except (can.CanError, OSError) as exc:
            err = str(exc)
            self.status.config(text=f"TX error: {exc}")
        stamp = datetime.datetime.now()
        dec = describe(frame, data)
        spd = "rpm ?" if rpm is None else f"{rpm:.0f} rpm"
        self.log(f"{'TX' if err is None else 'TX FAILED'} 0x{frame.arb_id:03X} {frame.title:<16} "
                 f"[{data.hex(' ').upper()}]  {dec}  @ {spd}" + (f"  ({err})" if err else ""),
                 stamp)
        self.groups[frame.arb_id]["sent"].config(
            text=(f"sent {stamp:%H:%M:%S}: {dec}" if err is None else f"TX FAILED {stamp:%H:%M:%S}"))
        if self.tune_csv:
            try:
                new = not os.path.exists(self.tune_csv) or os.path.getsize(self.tune_csv) == 0
                with open(self.tune_csv, "a", newline="") as fh:
                    w = csv.writer(fh)
                    if new:
                        w.writerow(["time", "id", "data_hex", "decoded", "rpm", "result"])
                    w.writerow([stamp.isoformat(timespec="milliseconds"), f"0x{frame.arb_id:03X}",
                                data.hex(" ").upper(), dec, "" if rpm is None else f"{rpm:.0f}",
                                "ok" if err is None else f"error: {err}"])
            except OSError as exc:
                self.log(f"CSV write failed: {exc}")

    def log(self, text, stamp=None):
        line = f"{(stamp or datetime.datetime.now()):%H:%M:%S.%f}"[:-3] + "  " + text
        print(line, flush=True)
        self.log_txt.config(state="normal")
        self.log_txt.insert("end", line + "\n")
        self.log_txt.see("end")
        self.log_txt.config(state="disabled")

    # ---- CAN ----
    def send(self, arb_id, data):
        try:
            self.bus.send(can.Message(arbitration_id=arb_id, is_extended_id=False, data=data))
            self.trace_tx(arb_id, data)
        except can.CanError as exc:
            self.status.config(text=f"TX error: {exc}")

    def send_once(self, arb_id, data):
        try:
            send_single_shot(self.bus, arb_id, data)
            self.trace_tx(arb_id, data)
        except Exception as exc:
            self.status.config(text=f"TX error: {exc}")

    # ---- CAN trace log ----
    def trace_tx(self, arb_id, data):
        tr = self.trace
        if tr is not None:
            tr.log_tx(arb_id, data, False)      # every frame the GUI sends is standard-id

    def trace_rx(self, m):
        tr = self.trace
        if tr is not None and not m.is_error_frame and not m.is_remote_frame:
            tr.log_rx(m.arbitration_id, m.data, m.is_extended_id, m.timestamp)

    @staticmethod
    def load_trace_dir():
        try:
            with open(SETTINGS, encoding="utf-8") as fh:
                d = json.load(fh).get("trace_dir")
            if d and os.path.isdir(d):
                return d
        except (OSError, ValueError, AttributeError):
            pass
        return os.getcwd()

    def browse_trace_dir(self):
        d = filedialog.askdirectory(title="Folder for CAN trace logs",
                                    initialdir=self.trace_dir.get() or os.getcwd())
        if d:
            self.trace_dir.set(os.path.normpath(d))

    def start_trace(self):
        if self.trace is not None:
            return
        folder = os.path.abspath(self.trace_dir.get().strip() or os.getcwd())
        path = os.path.join(folder, f"pegasus_{datetime.datetime.now():%Y%m%d_%H%M%S}.trc")
        try:
            os.makedirs(folder, exist_ok=True)
            tw = TraceWriter(path).open()
        except OSError as exc:
            messagebox.showerror("Cannot start log", f"{path}\n\n{exc}")
            self.log(f"trace log NOT started: {exc}")
            return
        self.trace = tw
        if not self.trace_dir_fixed:
            try:
                with open(SETTINGS, "w", encoding="utf-8") as fh:
                    json.dump({"trace_dir": folder}, fh)
            except OSError:
                pass
        self.trace_start_btn.config(state="disabled", bg="#c9c9c9")
        self.trace_stop_btn.config(state="normal", bg="#e06666")
        self.parse_btn.config(state="disabled")
        self.log(f"trace log started: {path}")

    def stop_trace(self):
        tr, self.trace = self.trace, None       # the rx/tx threads stop queueing first
        if tr is None:
            return
        n = tr.close()
        self.last_trace = tr.path
        self.trace_start_btn.config(state="normal", bg="#6aa84f")
        self.trace_stop_btn.config(state="disabled", bg="#c9c9c9")
        self.parse_btn.config(state="normal")
        msg = f"trace log closed: {tr.path} ({n} frames" + (f", {tr.dropped} DROPPED" if tr.dropped else "") + ")"
        if tr.error:
            msg += f"  WRITE ERROR: {tr.error}"
            messagebox.showerror("Trace log", msg)
        self.log(msg)
        self.trace_lbl.config(text=msg, foreground="#c00000" if tr.error or tr.dropped else "#666")

    def parse_last(self):
        if self.trace is not None:
            messagebox.showinfo("Parse last log", "Stop the log first.")
            return
        if not self.last_trace or not os.path.isfile(self.last_trace):
            messagebox.showinfo("Parse last log", "No closed trace log yet.")
            return
        if self.parsing:
            return
        self.parsing = True
        self.parse_btn.config(state="disabled")
        self.trace_lbl.config(text=f"parsing {self.last_trace} ...", foreground="#666")
        threading.Thread(target=self.parse_worker, args=(self.last_trace,), daemon=True).start()

    def parse_worker(self, path):
        """Parser2 in a subprocess, off the Tk thread; the result goes to parse_q."""
        cmd = [sys.executable, PARSER2, path, "--fw-rev", "worktree", "--scorecard",
               "--out", os.path.dirname(path)]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace",
                               timeout=PARSE_TIMEOUT_S,
                               creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            res = (path, cmd, r.returncode, r.stdout, r.stderr)
        except (OSError, subprocess.TimeoutExpired) as exc:
            res = (path, cmd, -1, "", f"{type(exc).__name__}: {exc}")
        self.parse_q.put(res)

    def show_parse(self, res):
        path, cmd, rc, out, err = res
        self.parsing = False
        self.parse_btn.config(state="normal")
        ok = rc == 0 and "Traceback" not in err
        msg = f"Parser2 {'done' if ok else 'FAILED (exit ' + str(rc) + ')'}: {path}"
        self.trace_lbl.config(text=msg, foreground="#666" if ok else "#c00000")
        self.log(msg)
        win = tk.Toplevel(self.root)
        win.title(f"Parser2 - {os.path.basename(path)}")
        head = tk.Label(win, text=msg, anchor="w", fg="#333" if ok else "#c00000", font=("Segoe UI", 9, "bold"))
        head.pack(fill="x", padx=8, pady=(6, 0))
        body = ttk.Frame(win)
        body.pack(fill="both", expand=True, padx=8, pady=6)
        txt = tk.Text(body, width=150, height=32, wrap="none", font=("Consolas", 9))
        sy = ttk.Scrollbar(body, command=txt.yview)
        sx = ttk.Scrollbar(body, orient="horizontal", command=txt.xview)
        txt.config(yscrollcommand=sy.set, xscrollcommand=sx.set)
        txt.grid(row=0, column=0, sticky="nsew")
        sy.grid(row=0, column=1, sticky="ns")
        sx.grid(row=1, column=0, sticky="ew")
        body.rowconfigure(0, weight=1)
        body.columnconfigure(0, weight=1)
        txt.insert("end", "$ " + " ".join(cmd) + "\n\n" + (out or "(no output)\n"))
        if err.strip():
            txt.insert("end", "\n--- stderr ---\n" + err)
        txt.config(state="disabled")
        ttk.Button(win, text="Close", command=win.destroy).pack(pady=(0, 8))

    def tx_loop(self):
        while not self.stop_evt.is_set():
            if self.tx_on:
                try:
                    thr, rpm = int(self.thrv.get()), int(self.speed.get())
                    rpm = max(0, min(int(self.max_rpm), rpm))
                    if self.legacy.get():
                        self.send(ID_THRV, struct.pack("<H", thr))
                        self.send(ID_DIR, bytes([DIR_CODE[self.dir.get()]]))
                    self.send(ID_SPEED, struct.pack("<H", rpm))
                    self.send(ID_PWR, bytes([int(self.pwr.get())]))
                except (tk.TclError, ValueError):
                    pass   # half-typed spinbox value
            time.sleep(TX_PERIOD_S)

    def rx_loop(self):
        while not self.stop_evt.is_set():
            try:
                m = self.bus.recv(timeout=0.2)
            except can.CanError as exc:         # keep the receive loop (and the log) alive
                self.note = f"RX error: {exc}"
                time.sleep(0.1)
                continue
            if m is not None:
                self.trace_rx(m)                # before any filtering: the log gets every frame
            if m is not None and m.arbitration_id == 0x72A and len(m.data) >= 1:
                txt = {0: "autotune running", 1: "autotune DONE - offsets stored",
                       2: "autotune done, EEPROM store FAILED"}.get(m.data[0], "autotune ?")
                with self.lock:
                    self.note = txt
                continue
            if m is None or len(m.data) < 8:
                continue
            d, v = m.data, {}
            now = time.time()
            if m.arbitration_id in (0x420, 0x421, 0x422) and m.is_extended_id:
                fv = decode_fast_tlm(m.arbitration_id, d)
                with self.lock:
                    self.rx.update({FAST_ROWS[k]: (f"{x:.2f}" if k != "rpm" else x)
                                    for k, x in fv.items()})
                    if m.arbitration_id == 0x420:
                        self.fast_n += 1
                        self.fast_time = now
                    elif m.arbitration_id == 0x421:
                        self.rpm, self.rpm_time = abs(fv["rpm"]), now
                continue
            if m.arbitration_id == 0x401:
                r, s, i_d, i_q = struct.unpack_from("<HHHH", d)
                v = {"RPM ref": r, "RPM": s, "Id (A)": i_d - 512, "Iq (A)": i_q - 512}
                with self.lock:
                    self.rpm, self.rpm_time = s, now
            elif m.arbitration_id == 0x403:
                vb, va, tm, tc, er, tq = struct.unpack_from("<HHBBBB", d)
                v = {"Vbus (V)": f"{vb / 100:.2f}", "Vaux (V)": f"{va / 100:.2f}",
                     "Motor T (C)": tm - 50, "Ctrl T (C)": tc - 50, "Error": name(ERRORS, er),
                     "Torque (Nm)": tq - 80}
            elif m.arbitration_id == 0x405:
                ts, cs, ov, pw, _off, thr = struct.unpack_from("<BBBBHH", d)
                v = {"Throttle state": THR_STATES.get(ts, ts), "State (cS)": name(SESSIONS, cs),
                     "Overrun": "YES" if ov else "no", "Mode": "SPORTS" if pw else "ECO",
                     "Throttle V (mV)": thr}
            elif m.arbitration_id == 0x406:
                v = {"Tau applied (us)": struct.unpack_from("<H", d)[0]}
            if v:
                with self.lock:
                    self.rx.update(v)
                    self.rx_time = now

    def refresh(self):
        with self.lock:
            snap, t = dict(self.rx), self.rx_time
            fast_n, fast_t = self.fast_n, self.fast_time
        for k, w in self.lbl.items():
            w.config(text=str(snap.get(k, "-")))
        now = time.time()
        n0, t0 = self.fast_seen
        if fast_t and now - fast_t < 0.5:
            self.fast_lbl.config(text=f"live {(fast_n - n0) / max(now - t0, 1e-3):.0f}/s")
        else:
            self.fast_lbl.config(text="off / stale" if fast_t else "-")
        self.fast_seen = (fast_n, now)
        tr = self.trace
        if tr is not None:
            self.trace_lbl.config(
                text=f"LOGGING {tr.path}: {tr.frames} frames written, backlog {tr.backlog}, "
                     f"dropped {tr.dropped}" + (f", WRITE ERROR {tr.error}" if tr.error else ""),
                foreground="#c00000" if tr.error or tr.dropped else "#274e13")
        try:
            self.show_parse(self.parse_q.get_nowait())
        except queue.Empty:
            pass
        age = now - t if t else None
        if age is None:
            msg = "no telemetry yet"
        elif age > RX_STALE_S:
            msg = f"telemetry STALE ({age:.0f} s)"
        else:
            msg = "telemetry live"
        self.status.config(text=f"{msg}   |   TX {'ON' if self.tx_on else 'off'}"
                           + (f"   |   {self.note}" if self.note else ""))
        self.root.after(250, self.refresh)

    # ---- buttons ----
    def toggle_tx(self):
        self.tx_on = not self.tx_on
        self.tx_btn.config(text="TX ON - sending every 100 ms" if self.tx_on else "ENABLE TX (off)",
                           bg="#6aa84f" if self.tx_on else "#c9c9c9")

    def do_reset(self):
        if messagebox.askyesno("Reset", "Send reset (0x111)? Only acts at RPM <= 10."):
            self.send_once(ID_RESET, bytes([1, 0, 0, 0, 0, 0, 0, 0]))

    def do_autotune(self):
        if messagebox.askyesno("Autotune", "Invalidate stored angle offset and re-run the autotune "
                               "on next boot (0x729)? The motor must be free to spin at ~2000 RPM."):
            self.send_once(ID_AUTOTUNE, bytes([1, 0, 0, 0, 0, 0, 0, 0]))

    def close(self):
        # leave the controller at zero speed, not at the last command
        if self.tx_on:
            self.tx_on = False
            time.sleep(TX_PERIOD_S * 1.5)
            for _ in range(3):
                self.send(ID_SPEED, struct.pack("<H", 0))
                time.sleep(0.02)
        self.stop_trace()       # flush and close the .trc so it is complete
        self.stop_evt.set()
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--interface", default="pcan")
    ap.add_argument("--channel", default="PCAN_USBBUS1")
    ap.add_argument("--bitrate", type=int, default=250000)
    ap.add_argument("--max-rpm", type=int, default=10500, help="slider/command cap (default 10500, TW-7 bench overspeed target)")
    ap.add_argument("--sim", action="store_true", help="run against a built-in fake controller")
    ap.add_argument("--tune-csv", default=None, help="also append every sent tuning frame to this CSV")
    ap.add_argument("--trace-dir", default=None,
                    help="folder for CAN trace logs (default: the last folder used, else the working directory)")
    ap.add_argument("--autoclose", type=float, default=0, help="close after N s (testing)")
    a = ap.parse_args()

    stop_sim = threading.Event()
    if a.sim:
        kw = dict(interface="virtual", channel="pegasus_sim", receive_own_messages=False)
        threading.Thread(target=sim_controller, args=(kw, stop_sim), daemon=True).start()
        time.sleep(0.2)
        bus = can.Bus(**kw)
    else:
        bus = can.Bus(interface=a.interface, channel=a.channel, bitrate=a.bitrate)

    root = tk.Tk()
    app = App(root, bus, a.max_rpm, a.tune_csv, a.trace_dir)
    if a.autoclose:
        def drive():       # exercise the TX path in test runs
            app.dir.set("F")
            app.speed.set(1200 if a.sim else 300)    # sim: above the Vq FF clamp speed
            app.toggle_tx()
        root.after(300, drive)
        if a.sim:          # tuning path, only against the fake controller
            root.after(100, app.start_trace)
            root.after(400, lambda: app.send_fast_tlm(True))
            root.after(1500, lambda: (app.set_fields(ID_TAU, tau_us=150),
                                      app.tune_send(TUNE_FRAMES[ID_TAU])))
            if a.autoclose >= 6:       # trace -> Parser2 round trip, result window left open until close
                root.after(int((a.autoclose - 2.5) * 1000), app.stop_trace)
                root.after(int((a.autoclose - 2.4) * 1000), app.parse_last)
        root.after(int(a.autoclose * 1000), app.close)
    root.mainloop()
    stop_sim.set()
    bus.shutdown()


if __name__ == "__main__":
    main()
