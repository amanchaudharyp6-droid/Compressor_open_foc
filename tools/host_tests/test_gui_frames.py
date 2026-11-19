#!/usr/bin/env python3
"""
Host test for the live-tuning frame encoders in tools/pegasus_gui.py.

Each encoded frame is decoded here with an independent copy of the firmware's
formula (Core/Src/can_rx_handlers.c, float32 arithmetic) and compared with what
the GUI says it will apply. No CAN hardware needed (python-can must import).

    python tools/host_tests/test_gui_frames.py        # or: python -m pytest tools/host_tests
"""
import math
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import pegasus_gui as g  # noqa: E402


def f32(x):
    return struct.unpack("<f", struct.pack("<f", float(x)))[0]


def u16(m, i):
    return (m[i + 1] << 8) | m[i]


# ---- firmware decode, transcribed from can_rx_handlers.c ----
def fw_pi(m):           # 0x105 / 0x106 / 0x107
    kp, ki, kd = f32(f32(u16(m, 0)) * f32(1.0e-2)), f32(f32(u16(m, 2)) * f32(1.0e-2)), \
        f32(f32(u16(m, 4)) * f32(1.0e-5))
    low = [f32(x * f32(0.8)) for x in (kp, ki, kd)]          # * LOW_SPEED_SCALING
    return {"Kp": kp, "Ki": ki, "Kd": kd}, low


def fw_0x103(m):
    return {"Lambda": f32(f32(u16(m, 0)) * f32(1.0e-5)), "Ld": f32(f32(u16(m, 2)) * f32(1.0e-8)),
            "Lq": f32(f32(u16(m, 4)) * f32(1.0e-7)), "Rs": f32(f32(u16(m, 6)) * f32(1.0e-6))}


def fw_0x104(m):
    return {"Kfw": f32(f32(u16(m, 0)) * f32(1.0e-3)), "Kaw": f32(f32(u16(m, 2)) * f32(1.0e-3)),
            "M_to_F": f32(m[4] * f32(0.01)), "F_to_M": f32(m[5] * f32(0.01)),
            "Rst_Factor": f32(m[6] * f32(0.01))}


def fw_0x115(m):
    tau_s = min(f32(f32(u16(m, 0)) * f32(1.0e-6)), f32(400.0e-6))   # fminf(.., MAX)
    readback_us = int(f32(f32(tau_s * f32(1.0e6)) + f32(0.5)))       # 0x406 bytes 0-1
    return tau_s, readback_us


def fw_0x116(m):
    return 1 if m[0] == 1 else 0


FW = {0x103: fw_0x103, 0x104: fw_0x104}

# raw integers the boot values must encode to (firmware boot value / LSB)
BOOT_RAW = {
    0x105: {"Kp": 400, "Ki": 800, "Kd": 1},
    0x106: {"Kp": 17, "Ki": 350, "Kd": 1},
    0x107: {"Kp": 15, "Ki": 400, "Kd": 1},
    0x103: {"Lambda": 1121, "Ld": 6350, "Lq": 879, "Rs": 6160},
    0x104: {"Kfw": 2, "Kaw": 12000, "M_to_F": 85, "F_to_M": 55, "Rst_Factor": 45},
    0x115: {"tau_us": 0},
    0x116: {"on": 0},
}


class FrameTests(unittest.TestCase):
    def roundtrip(self, arb_id, values):
        fr = g.TUNE_FRAMES[arb_id]
        data, raw = g.encode_frame(fr, values)
        self.assertEqual(len(data), 8)
        gui = g.decode_frame(fr, data)
        if arb_id in (0x105, 0x106, 0x107):
            fw, low = fw_pi(data)
            self.assertEqual([g.low_speed(gui[k]) for k in ("Kp", "Ki", "Kd")], low)
        elif arb_id in FW:
            fw = FW[arb_id](data)
        elif arb_id == 0x115:
            tau_s, rb = fw_0x115(data)
            fw = {"tau_us": float(rb)}
            self.assertAlmostEqual(tau_s * 1e6, gui["tau_us"], places=3)
        else:
            fw = {"on": float(fw_0x116(data))}
        self.assertEqual(gui, fw, f"0x{arb_id:03X} GUI preview != firmware decode")
        for f in fr.fields:   # what the preview shows is what the firmware decodes
            self.assertEqual(g.applied(f, raw[f.key]), gui[f.key])
        return data, raw, gui

    def test_boot_values_encode_exactly(self):
        for arb_id, vals in g.BOOT.items():
            data, raw, gui = self.roundtrip(arb_id, vals)
            self.assertEqual(raw, BOOT_RAW[arb_id], f"0x{arb_id:03X}")
            for k, v in vals.items():   # boot values survive the round trip (to float32)
                self.assertTrue(math.isclose(gui[k], v, rel_tol=1e-6), f"0x{arb_id:03X} {k}")

    def test_layout_bytes(self):
        d, _ = g.encode_frame(g.TUNE_FRAMES[0x105], {"Kp": 4, "Ki": 8, "Kd": 1e-5})
        self.assertEqual(d, bytes([0x90, 0x01, 0x20, 0x03, 0x01, 0x00, 0, 0]))
        d, _ = g.encode_frame(g.TUNE_FRAMES[0x104], g.BOOT[0x104])
        self.assertEqual(d, bytes([2, 0, 0xE0, 0x2E, 85, 55, 45, 0]))
        d, _ = g.encode_frame(g.TUNE_FRAMES[0x115], {"tau_us": 210})
        self.assertEqual(d, bytes([210, 0, 0, 0, 0, 0, 0, 0]))
        d, _ = g.encode_frame(g.TUNE_FRAMES[0x116], {"on": 1})
        self.assertEqual(d, bytes([1, 0, 0, 0, 0, 0, 0, 0]))

    def test_quantisation_truncates(self):
        _, raw, gui = self.roundtrip(0x106, {"Kp": "0.136", "Ki": 3.5, "Kd": 1e-5})
        self.assertEqual(raw["Kp"], 13)
        self.assertAlmostEqual(gui["Kp"], 0.13, places=6)
        # float traps: x / lsb lands just below an integer
        for v, want in ((0.29, 29), (0.57, 57), (1.15, 115), (0.07, 7)):
            self.assertEqual(g.quantise(g.PI_FIELDS[0], v), want, v)
        self.assertEqual(g.quantise(g.TUNE_FRAMES[0x104].fields[2], 0.55), 55)
        self.assertEqual(g.quantise(g.PI_FIELDS[2], 4e-6), 0)          # below 1 LSB -> 0

    def test_displayed_value_requantises_to_same_raw(self):
        # what the preview shows (fmt of the float32 decode) must send the same raw back
        seen = set()
        for fr in g.TUNE_FRAMES.values():
            for f in fr.fields:
                if (f.lsb, f.nbytes) in seen:
                    continue
                seen.add((f.lsb, f.nbytes))
                top = (1 << (8 * f.nbytes)) - 1
                if f.max_value is not None:
                    top = min(top, int(f.max_value / f.lsb))
                for raw in range(top + 1):
                    self.assertEqual(g.quantise(f, g.fmt(g.applied(f, raw))), raw, (f.key, raw))

    def test_presets(self):
        for kp, ki in ((4, 8), (2, 4)):
            _, raw, _ = self.roundtrip(0x105, {"Kp": kp, "Ki": ki, "Kd": 1e-5})
            self.assertEqual((raw["Kp"], raw["Ki"]), (kp * 100, ki * 100))
        for us in (0, 150, 210, 250, 400):
            _, raw, gui = self.roundtrip(0x115, {"tau_us": us})
            self.assertEqual(raw["tau_us"], us)
            self.assertEqual(gui["tau_us"], us)

    def test_range_limits(self):
        pi = g.TUNE_FRAMES[0x105]
        bad = [{"Kp": -1, "Ki": 8, "Kd": 0}, {"Kp": 655.36, "Ki": 8, "Kd": 0},
               {"Kp": "nan", "Ki": 8, "Kd": 0}, {"Kp": "inf", "Ki": 8, "Kd": 0},
               {"Kp": "", "Ki": 8, "Kd": 0}, {"Kp": "abc", "Ki": 8, "Kd": 0},
               {"Kp": 4, "Ki": 8, "Kd": 0.65536}]
        for vals in bad:
            with self.assertRaises(ValueError, msg=vals):
                g.encode_frame(pi, vals)
        self.roundtrip(0x105, {"Kp": 655.35, "Ki": 0, "Kd": 0.65535})   # top of u16
        with self.assertRaises(ValueError):
            g.encode_frame(g.TUNE_FRAMES[0x115], {"tau_us": 401})       # firmware clamps at 400
        with self.assertRaises(ValueError):
            g.encode_frame(g.TUNE_FRAMES[0x115], {"tau_us": -5})
        fw = dict(g.BOOT[0x104])
        for k, v in (("M_to_F", 2.56), ("Kaw", 65.536)):
            with self.assertRaises(ValueError, msg=k):
                g.encode_frame(g.TUNE_FRAMES[0x104], dict(fw, **{k: v}))
        with self.assertRaises(ValueError):
            g.encode_frame(g.TUNE_FRAMES[0x116], {"on": 2})

    def test_fw_invariants(self):
        fw = g.TUNE_FRAMES[0x104]
        with self.assertRaises(ValueError):
            g.encode_frame(fw, dict(g.BOOT[0x104], F_to_M=0.9))          # F2M >= M2F
        with self.assertRaises(ValueError):
            g.encode_frame(fw, dict(g.BOOT[0x104], Rst_Factor=0.6))      # RST > F2M

    def test_full_frame_required(self):
        with self.assertRaises(ValueError):
            g.encode_frame(g.TUNE_FRAMES[0x105], {"Kp": 4, "Ki": 8})     # Kd missing

    def test_fast_tlm_decode(self):
        d = g.decode_fast_tlm(0x420, struct.pack("<hhhh", -12345, 2500, -32767, 1))
        self.assertEqual(d, {"Id_ref": -123.45, "Iq_ref": 25.0, "Id": -327.67, "Iq": 0.01})
        d = g.decode_fast_tlm(0x421, struct.pack("<hhhh", -4100, 2950, -8000, 5123))
        self.assertEqual(d, {"Vd": -41.0, "Vq": 29.5, "rpm": -8000, "Vdc": 51.23})
        self.assertEqual(g.decode_fast_tlm(0x401, bytes(8)), {})

    def test_constants_match_firmware_source(self):
        root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")

        def read(rel):
            with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as fh:
                return fh.read()
        prof = read("Core/Inc/motor_profiles/motor_hirange_tr.h")
        vcfg = read("Core/Inc/vehicle_config.h")

        def define(text, nm):
            import re
            m = re.search(rf"#define\s+{nm}\s+([-0-9.eE]+)f?\b", text)
            self.assertIsNotNone(m, nm)
            return float(m.group(1).rstrip("f"))
        self.assertEqual(define(prof, "LOW_SPEED_SCALING"), g.LOW_SPEED_SCALING)
        self.assertEqual(define(vcfg, "FOC_ISENSE_DELAY_MAX_S") * 1e6, g.TAU_MAX_US)
        self.assertEqual(define(vcfg, "FOC_ISENSE_DELAY_S"), g.BOOT[0x115]["tau_us"])
        want = {0x105: ("SPEED_KP", "SPEED_KI", "SPEED_KD"), 0x106: ("ID_KP", "ID_KI", "ID_KD"),
                0x107: ("IQ_KP", "IQ_KI", "IQ_KD")}
        for arb_id, names in want.items():
            self.assertEqual([define(prof, n) for n in names],
                             [g.BOOT[arb_id][k] for k in ("Kp", "Ki", "Kd")], f"0x{arb_id:03X}")
        self.assertEqual([define(prof, n) for n in ("LAMBDA", "LD", "LQ", "RS")],
                         [g.BOOT[0x103][k] for k in ("Lambda", "Ld", "Lq", "Rs")])
        self.assertEqual([define(prof, n) for n in ("FW_KFW", "FW_KAW")],
                         [g.BOOT[0x104]["Kfw"], g.BOOT[0x104]["Kaw"]])
        self.assertEqual([define(prof, n) * 0.01 for n in
                          ("FW_M2F_PCT", "FW_F2M_PCT", "FW_RST_FACTOR_PCT")],
                         [g.BOOT[0x104][k] for k in ("M_to_F", "F_to_M", "Rst_Factor")])


if __name__ == "__main__":
    unittest.main(verbosity=2)
