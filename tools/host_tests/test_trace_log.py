#!/usr/bin/env python3
"""
Host test for the CAN trace log in tools/pegasus_gui.py and its round trip through
tools/Parser2.py: a synthetic bench trace (0x420/0x421/0x422 at 200 Hz in different
arrival orders, queue-delay bunching, real lost windows, 10 Hz slow frames with 0x40A,
TX commands) is written with the GUI's TraceWriter and decoded back by Parser2.
No CAN hardware and no display needed (python-can and tkinter must import).

    python tools/host_tests/test_trace_log.py        # or: python -m pytest tools/host_tests
"""
import csv
import os
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest

TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, TOOLS)
import can  # noqa: E402
import Parser  # noqa: E402
import Parser2 as P2  # noqa: E402
import pegasus_gui as g  # noqa: E402

T0 = 1_000_000.0            # arbitrary epoch for the synthetic clock, s
N_WIN = 400                 # 2 s of 5 ms windows
LOST_WINDOWS = (250, 251)   # never arrive
NO_422 = 300                # only this window's 0x422 is lost
NO_420 = 310                # only this window's 0x420 is lost
BUNCHED = (150, 151, 152, 153)   # held ~13 ms behind a slow burst, then delivered back to back


def raw16(v):
    return int(round(v / 0.01))


def window_values(i):
    """Engineering values of window i (distinct per window, 0.01 resolution)."""
    return {
        "Id_ref": -53.30 + 0.01 * i, "Iq_ref": 12.00 + 0.02 * i, "Id": -53.10 + 0.01 * i, "Iq": 11.90 + 0.02 * i,
        "Vd": -6.00 + 0.01 * i, "Vq": 21.00 + 0.01 * i, "RPM": 10000 + i, "Vdc": 48.50 + 0.001 * i,
        "Vd_FF": -9.10 - 0.01 * i, "Vq_FF": 29.40, "Vq_FF_unsat": 29.40 + 0.05 * i, "FF_lim": 29.40,
    }


def window_frames(i):
    v = window_values(i)
    return {
        0x420: struct.pack("<hhhh", *(raw16(v[k]) for k in ("Id_ref", "Iq_ref", "Id", "Iq"))),
        0x421: struct.pack("<hhhh", raw16(v["Vd"]), raw16(v["Vq"]), v["RPM"], raw16(v["Vdc"])),
        0x422: struct.pack("<hhhh", *(raw16(v[k]) for k in ("Vd_FF", "Vq_FF", "Vq_FF_unsat", "FF_lim"))),
    }


def build_events(with_422=True):
    """[(time_ms, rx|tx, id, extended, data)] for the whole synthetic run, in arrival order."""
    ev = [(500.0, "tx", 0x116, False, bytes([1, 0, 0, 0, 0, 0, 0, 0])),
          (520.0, "tx", 0x102, False, struct.pack("<H", 10000))]
    for i in range(N_WIN):
        if i in LOST_WINDOWS:
            continue
        t = 1000.0 + 5.0 * i
        order = [0x420, 0x421, 0x422]
        if i < 100:                                   # either arrival order has been seen in real logs
            order = ([0x420, 0x421, 0x422], [0x421, 0x420, 0x422], [0x422, 0x421, 0x420])[i % 3]
        if i in BUNCHED:                              # queued behind the slow burst: late, then back to back
            t = 1000.0 + 5.0 * BUNCHED[0] + 13.0 + 1.8 * (i - BUNCHED[0])
        frames = window_frames(i)
        for k, cid in enumerate(order):
            if (i == NO_422 and cid == 0x422) or (i == NO_420 and cid == 0x420) or (cid == 0x422 and not with_422):
                continue
            ev.append((t + 0.6 * k, "rx", cid, True, frames[cid]))
    for j in range(20):                               # 10 Hz slow cycle
        t = 1003.0 + 100.0 * j
        ev.append((t, "rx", 0x401, True, struct.pack("<HHHH", 10000, 10000 + j, 512 - 53, 512 + 12)))
        ev.append((t + 0.6, "rx", 0x40A, True, struct.pack("<HHHH", 1000 + j, (65530 + j) & 0xFFFF, 3000, 0)))
    ev.sort(key=lambda e: e[0])
    return ev


def write_trace(path, events):
    tw = g.TraceWriter(path, start=T0).open()
    for t, kind, cid, ext, data in events:
        ts = T0 + t / 1000.0
        if kind == "tx":
            tw.log_tx(cid, data, ext, now=ts)
        else:
            tw.log_rx(cid, data, ext, hw_ts=ts, now=ts)     # driver clock == PC clock here
    n = tw.close()
    assert n == len(events) and tw.dropped == 0 and tw.error is None, (n, len(events), tw.dropped, tw.error)


def fast_cfg():
    return P2.load_fw_config("worktree")


class RowFormat(unittest.TestCase):
    def test_rows_match_pcan_view(self):
        # lines copied from real PCAN-View 1.1 traces (Bringup_gen4/29_10/10500_unstable_2.trc)
        self.assertEqual(g.TraceWriter.format_row(1, 23.7, 0x102, False, bytes([0x10, 0x27])),
                         "     1)        23.7  Rx         0102  2  10 27 \r\n")
        self.assertEqual(g.TraceWriter.format_row(4, 44.1, 0x401, True, bytes.fromhex("6E1A6F1AD6010902")),
                         "     4)        44.1  Rx     00000401  8  6E 1A 6F 1A D6 01 09 02 \r\n")
        self.assertEqual(g.TraceWriter.format_row(13, 866937.3, 0x421, True, bytes(8), tx=True),
                         "    13)    866937.3  Tx     00000421  8  00 00 00 00 00 00 00 00 \r\n")

    def test_header_is_read_as_v1_1(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "h.trc")
            write_trace(p, [(1.0, "rx", 0x401, True, bytes(8))])
            self.assertEqual(Parser._trc_layout(p), ("1.1", None))
            with open(p, "rb") as fh:
                head = fh.read()
            self.assertTrue(head.startswith(b";$FILEVERSION=1.1\r\n;$STARTTIME="))
            self.assertIn(b"Start time: ", head)


class ParserRoundTrip(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.trc = os.path.join(cls.tmp.name, "synthetic.trc")
        write_trace(cls.trc, build_events())
        cls.cfg = fast_cfg()
        cls.frames = P2.read_frames(cls.trc)
        cls.slow, cls.bursts, cls.fast, cls.events = P2.parse_log(cls.frames, cls.cfg)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_frames_survive_the_trace(self):
        self.assertEqual(len(self.frames), len(build_events()))
        self.assertEqual({0x116, 0x102} & {c for _, c, _ in self.frames}, {0x116, 0x102})   # TX rows read back

    def test_every_window_grouped_in_any_arrival_order(self):
        expected = [i for i in range(N_WIN) if i not in LOST_WINDOWS]
        self.assertEqual(len(self.fast), len(expected))
        for row, i in zip(self.fast, expected):
            v = window_values(i)
            want = {k: v[k] for k in ("Id_ref", "Iq_ref", "Id", "Iq", "Vd", "Vq", "RPM", "Vdc",
                                      "Vd_FF", "Vq_FF", "Vq_FF_unsat", "FF_lim")}
            if i == NO_420:
                want = {k: x for k, x in want.items() if k not in ("Id_ref", "Iq_ref", "Id", "Iq")}
            if i == NO_422:
                want = {k: x for k, x in want.items() if k not in ("Vd_FF", "Vq_FF", "Vq_FF_unsat", "FF_lim")}
            for k, x in want.items():
                self.assertAlmostEqual(row[k], x, delta=0.0051 if k != "RPM" else 0.5, msg=f"window {i} {k}")
            self.assertEqual(row["pair_ok"], 0 if i == NO_420 else 1, f"window {i}")
            for k in ("Vd_FF", "Vq_FF", "Vq_FF_unsat", "FF_lim"):      # lost 0x422: columns absent
                self.assertEqual(k in row, i != NO_422, f"window {i} {k}")

    def test_unpaired_count_is_only_the_lost_frame(self):
        self.assertEqual(sum(1 for r in self.fast if not r["pair_ok"]), 1)

    def test_missed_frames_counts_lost_windows_not_jitter(self):
        # two windows never arrived; the 13 ms queue delay at BUNCHED must not count
        self.assertEqual(sum(r["missed_frames"] for r in self.fast), len(LOST_WINDOWS))
        first_after_loss = LOST_WINDOWS[-1] + 1 - len(LOST_WINDOWS)       # index in self.fast
        self.assertEqual(self.fast[first_after_loss]["missed_frames"], len(LOST_WINDOWS))
        self.assertEqual(self.cfg["fast_telemetry"]["missed_frames"], len(LOST_WINDOWS))
        self.assertEqual(self.cfg["fast_telemetry"]["frames"]["0x422"], N_WIN - len(LOST_WINDOWS) - 1)

    def test_slow_frame_0x40A(self):
        # slow rows forward-fill, so pick the row each 0x40A frame produced by its timestamp
        z = [next(r for r in self.slow if abs(r["timestamp"] - (1003.6 + 100.0 * j)) < 0.05) for j in range(20)]
        for j, r in enumerate(z):
            self.assertEqual((r["Count_At_Z"], r["Z_Count"], r["Count_From_Duty"]),
                             (1000 + j, (65530 + j) & 0xFFFF, 3000))
        self.assertFalse(any("Z_Count" in r for r in self.slow if r["timestamp"] < 1003.0))
        self.assertEqual(z[6]["Z_Count"], 0)            # u16 wrap is passed through, not unwrapped

    def test_tx_commands_reach_the_tuning_state(self):
        self.assertTrue(any(e[1] == "0x116" and e[2] == {"Tune_Fast_Telem": 1} for e in self.events))
        self.assertTrue(self.fast[-1]["Tune_Source"].startswith("0x116@"))

    def test_log_without_0x422_parses_as_before(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "old.trc")
            write_trace(p, build_events(with_422=False))
            _, _, fast, _ = P2.parse_log(P2.read_frames(p), fast_cfg())
            self.assertEqual(len(fast), N_WIN - len(LOST_WINDOWS))
            self.assertTrue(all("Vd_FF" not in r for r in fast))
            self.assertEqual(sum(1 for r in fast if not r["pair_ok"]), 1)

    def test_cli_csv_columns(self):
        out = os.path.join(self.tmp.name, "out")
        r = subprocess.run([sys.executable, P2.__file__, self.trc, "--fw-rev", "worktree", "--scorecard",
                            "--out", out], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        with open(os.path.join(out, "synthetic_fast.csv"), newline="") as fh:
            rows = list(csv.DictReader(fh))
        self.assertEqual(list(rows[0])[-4:], ["Vd_FF", "Vq_FF", "Vq_FF_unsat", "FF_lim"])
        self.assertEqual(list(rows[0])[:12], ["timestamp", "Id_ref", "Iq_ref", "Id", "Iq", "Vd", "Vq", "RPM",
                                              "Vdc", "pair_ok", "dt_ms", "missed_frames"])
        v = window_values(7)
        self.assertAlmostEqual(float(rows[7]["Vq_FF_unsat"]), v["Vq_FF_unsat"], places=2)
        self.assertEqual(rows[NO_422 - 2]["Vq_FF"], "")                   # lost 0x422 -> empty cell
        with open(os.path.join(out, "synthetic_slow.csv"), newline="") as fh:
            header = next(csv.reader(fh))
        self.assertEqual(header[-3:], ["Count_At_Z", "Z_Count", "Count_From_Duty"])
        self.assertIn("missed frame(s)", r.stdout)


class WriterBehaviour(unittest.TestCase):
    def test_no_frame_lost_at_telemetry_rate_and_producer_never_blocks(self):
        n = 3 * 200 * 150 + 1500                 # 150 s of fast telemetry plus slow frames
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "load.trc")
            tw = g.TraceWriter(p, start=T0).open()
            t_start = time.perf_counter()
            for k in range(n):
                tw.log_rx(0x420 + k % 3, bytes(8), True, hw_ts=T0 + k * 0.0016, now=T0 + k * 0.0016)
                if k % 4000 == 0:
                    tw.log_tx(0x102, bytes(2), False, now=T0 + k * 0.0016)
            produce_s = time.perf_counter() - t_start
            rows = tw.close()
            self.assertEqual(rows, n + len(range(0, n, 4000)))
            self.assertEqual((tw.dropped, tw.error), (0, None))
            self.assertLess(produce_s, 5.0, "log_rx must only enqueue")
            with open(p, "rb") as fh:
                lines = [ln for ln in fh.read().split(b"\r\n") if ln and not ln.startswith(b";")]
            self.assertEqual(len(lines), rows)
            self.assertEqual(int(lines[-1].split(b")")[0]), rows)         # numbering is gapless

    def test_flushed_while_running(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "live.trc")
            tw = g.TraceWriter(p, start=T0).open()
            for k in range(100):
                tw.log_rx(0x401, bytes(8), True, hw_ts=T0 + k * 0.01, now=T0 + k * 0.01)
            time.sleep(g.TraceWriter.FLUSH_S + 0.8)          # still open: a crash now must keep the rows
            with open(p, "rb") as fh:
                rows = [ln for ln in fh.read().split(b"\r\n") if ln and not ln.startswith(b";")]
            tw.close()
            self.assertEqual(len(rows), 100)

    def test_offsets_are_monotonic_and_0_1_ms(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "t.trc")
            tw = g.TraceWriter(p, start=T0).open()
            now = T0 + 5.0
            # first frame: driver clock 3 ms behind the PC clock. Second frame, 1 ms later on the
            # driver clock, arrives with only 0.5 ms latency: the clock mapping tightens, which would
            # put it before the first frame; it must not go backwards.
            tw.log_rx(0x420, bytes(8), True, hw_ts=now - 0.003, now=now)
            tw.log_rx(0x421, bytes(8), True, hw_ts=now - 0.002, now=now - 0.0015)
            tw.close()
            with open(p, "rb") as fh:
                rows = [ln.split() for ln in fh.read().split(b"\r\n") if ln and not ln.startswith(b";")]
            t = [float(r[1]) for r in rows]
            self.assertEqual(t, [5000.0, 5000.0])
            self.assertTrue(all(len(r[1].split(b".")[1]) == 1 for r in rows))

    def test_write_failure_is_reported_not_raised(self):
        with tempfile.TemporaryDirectory() as d:
            tw = g.TraceWriter(os.path.join(d, "f.trc"), start=T0).open()
            tw._fh.close()                                   # simulate the disk going away
            tw.log_rx(0x401, bytes(8), True, hw_ts=T0 + 1, now=T0 + 1)
            tw.close()
            self.assertIsNotNone(tw.error)
            self.assertEqual(tw.frames, 0)


class SimController(unittest.TestCase):
    def test_sim_emits_0x422_and_0x40A(self):
        kw = dict(interface="virtual", channel="pegasus_trace_test", receive_own_messages=False)
        stop = threading.Event()
        th = threading.Thread(target=g.sim_controller, args=(kw, stop), daemon=True)
        th.start()
        bus = can.Bus(**kw)
        try:
            time.sleep(0.2)
            bus.send(can.Message(arbitration_id=g.ID_SPEED, is_extended_id=False, data=struct.pack("<H", 3000)))
            bus.send(can.Message(arbitration_id=g.ID_FAST_TLM, is_extended_id=False, data=bytes([1])))
            seen = {}
            end = time.time() + 2.2
            while time.time() < end:
                m = bus.recv(timeout=0.1)
                if m is not None and m.is_extended_id:
                    seen.setdefault(m.arbitration_id, []).append(bytes(m.data))
        finally:
            stop.set()
            th.join(2)
            bus.shutdown()
        self.assertTrue({0x420, 0x421, 0x422, 0x40A} <= set(seen), sorted(seen))
        self.assertGreater(len(seen[0x422]), 100)                         # ~200/s while fast tlm is on
        d = g.decode_fast_tlm(0x422, seen[0x422][-1])
        self.assertEqual(d["Vq_FF"], min(d["Vq_FF_unsat"], d["FF_lim"]))
        self.assertAlmostEqual(d["FF_lim"], 0.57735 * 51.0, places=1)
        cz, zc, cfd, rsv = struct.unpack("<HHHH", seen[0x40A][-1])
        self.assertEqual((cz, cfd, rsv), (1234, 3000, 0))
        self.assertGreaterEqual(zc, 1)

    def test_decode_fast_tlm_0x422(self):
        d = g.decode_fast_tlm(0x422, struct.pack("<hhhh", -912, 2940, 3105, 2940))
        self.assertEqual(d, {"Vd_FF": -9.12, "Vq_FF": 29.4, "Vq_FF_unsat": 31.05, "FF_lim": 29.4})


if __name__ == "__main__":
    unittest.main(verbosity=2)
