"""Parser2 - Parser.py plus everything a tuning session needs to be self-describing.

    python tools/Parser2.py <log.trc> [--fw-rev REV] [--scorecard] [--burst] [--out DIR]

Outputs (next to the log, or in --out DIR):
  <log>_slow.csv       Parser.py's rows/columns, plus forward-filled tuning state
                       taken from the command frames in the trace (0x102-0x116),
                       the derived current-PI gain set, and constant firmware-config
                       columns. Parser.py emits one row per decoded frame (~28 per
                       100 ms telemetry cycle); --burst writes one row per cycle.
                       Bench encoder diagnostics (ext 0x40A: Count_At_Z, Z_Count,
                       Count_From_Duty) are appended after the cfg_* columns.
  <log>_config.json    compile-time config of the firmware rev (git show REV:...),
                       with file:line for every value, plus sanity checks against
                       the telemetry echoes (0x404/0x406-0x409, 0x7B0 git SHA).
  <log>_fast.csv       200 Hz bench telemetry (ext 0x420/0x421/0x422), if the log has it.
                       One row per 5 ms window; the frames of a window are grouped by
                       arrival time, in any order. 0x422 (feed-forward) is empty in
                       logs from firmware that does not send it.
  <log>_scorecard.csv  per-hold scorecard (--scorecard), also printed.

Decoders for the controller's telemetry come from Parser.py (not modified here).
"""
import argparse
import collections
import csv
import json
import math
import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import Parser
from Parser import DECODERS, u16

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def s16(lo, hi):
    v = u16(lo, hi)
    return v - 0x10000 if v & 0x8000 else v


# ----------------------------
# COMMAND (controller RX) DECODERS
# Byte layouts from Core/Src/can_rx_handlers.c (dispatch table) and the
# acceptance filters in Core/Src/fdcan_AL.c FDCAN_SETUP(). All standard IDs.
# ----------------------------

def decode_cmd_102(d):
    # speed ref (rpm) when Speed_1_Torque_0, else torque ref
    return {"Cmd_Ref_0x102": u16(d[0], d[1])}

def decode_cmd_103(d):
    # motor model into Current_Ref_Estimator (MTPA/FW references)
    return {
        "Tune_Lambda": u16(d[0], d[1]) * 1.0E-5,
        "Tune_Ld": u16(d[2], d[3]) * 1.0E-8,
        "Tune_Lq": u16(d[4], d[5]) * 1.0E-7,
        "Tune_Rs": u16(d[6], d[7]) * 1.0E-6
    }

def decode_cmd_104(d):
    # generated FW chart params (dead while FW_CLOSED_LOOP)
    return {
        "Tune_Kfw": u16(d[0], d[1]) * 1.0E-3,
        "Tune_Kaw": u16(d[2], d[3]) * 1.0E-3,
        "Tune_M2F": d[4] * 0.01,
        "Tune_F2M": d[5] * 0.01,
        "Tune_FW_Rst": d[6] * 0.01
    }

def decode_cmd_105(d):
    return {
        "Tune_Speed_Kp": u16(d[0], d[1]) * 1.0E-2,
        "Tune_Speed_Ki": u16(d[2], d[3]) * 1.0E-2,
        "Tune_Speed_Kd": u16(d[4], d[5]) * 1.0E-5
    }

def decode_cmd_106(d):
    return {
        "Tune_Id_Kp": u16(d[0], d[1]) * 1.0E-2,
        "Tune_Id_Ki": u16(d[2], d[3]) * 1.0E-2,
        "Tune_Id_Kd": u16(d[4], d[5]) * 1.0E-5
    }

def decode_cmd_107(d):
    return {
        "Tune_Iq_Kp": u16(d[0], d[1]) * 1.0E-2,
        "Tune_Iq_Ki": u16(d[2], d[3]) * 1.0E-2,
        "Tune_Iq_Kd": u16(d[4], d[5]) * 1.0E-5
    }

def decode_cmd_108(d):
    # ignored by firmware when CAN_SPEED_ONLY
    return {"Cmd_Direction": d[0]}

def decode_cmd_111(d):
    return {"Cmd_Reset": d[0]}

def decode_cmd_113(d):
    # power mode (0x109 is the BMS frame in VH mode)
    return {"Cmd_Power_Mode": d[0]}

def decode_cmd_114(d):
    # throttle voltage, mV (ignored when CAN_SPEED_ONLY)
    return {"Cmd_Throttle_V": u16(d[0], d[1]) * 0.001}

def decode_cmd_115(d):
    # current-sense delay tau, us; firmware clamps to FOC_ISENSE_DELAY_MAX_S
    return {"Tune_Isense_Tau_us": u16(d[0], d[1])}

def decode_cmd_116(d):
    # bench fast telemetry: byte0 == 1 on, anything else off
    return {"Tune_Fast_Telem": 1 if d[0] == 1 else 0}

def decode_cmd_729(d):
    return {"Cmd_Autotune": d[0]}

CMD_DECODERS = {
    0x102: decode_cmd_102,
    0x103: decode_cmd_103,
    0x104: decode_cmd_104,
    0x105: decode_cmd_105,
    0x106: decode_cmd_106,
    0x107: decode_cmd_107,
    0x108: decode_cmd_108,
    0x111: decode_cmd_111,
    0x113: decode_cmd_113,
    0x114: decode_cmd_114,
    0x115: decode_cmd_115,
    0x116: decode_cmd_116,
    0x729: decode_cmd_729,
}

# Commands that change tuning (logged in Tune_Source); the rest are operator
# commands kept as Cmd_* columns.
TUNING_IDS = {0x103, 0x104, 0x105, 0x106, 0x107, 0x115, 0x116}

TUNE_COLS = [
    "Tune_Speed_Kp", "Tune_Speed_Ki", "Tune_Speed_Kd",
    "Tune_Id_Kp", "Tune_Id_Ki", "Tune_Id_Kd",
    "Tune_Iq_Kp", "Tune_Iq_Ki", "Tune_Iq_Kd",
    "Tune_Isense_Tau_us", "Tune_Fast_Telem",
    "Tune_Lambda", "Tune_Ld", "Tune_Lq", "Tune_Rs",
    "Tune_Kfw", "Tune_Kaw", "Tune_M2F", "Tune_F2M", "Tune_FW_Rst",
    "Tune_Source",
    "Cmd_Ref_0x102", "Cmd_Power_Mode", "Cmd_Direction", "Cmd_Throttle_V",
    "Cmd_Reset", "Cmd_Autotune",
]

DERIVED_COLS = [
    "Gain_Set",
    "Eff_Speed_Kp", "Eff_Speed_Ki",
    "Eff_Id_Kp", "Eff_Id_Ki",
    "Eff_Iq_Kp", "Eff_Iq_Ki",
]


# ----------------------------
# FAST TELEMETRY DECODERS (bench, FOC_BENCH_FAST_TLM)
# fdcan_AL.c Fast_Tlm_Tick_1ms(): int16 LE, means over the FAST_TLM_PERIOD_MS
# window except rpm (instantaneous d.RPM, signed).
# ----------------------------

def decode_420(d):
    return {
        "Id_ref": s16(d[0], d[1]) * 0.01,
        "Iq_ref": s16(d[2], d[3]) * 0.01,
        "Id": s16(d[4], d[5]) * 0.01,
        "Iq": s16(d[6], d[7]) * 0.01
    }

def decode_421(d):
    return {
        "Vd": s16(d[0], d[1]) * 0.01,
        "Vq": s16(d[2], d[3]) * 0.01,
        "RPM": s16(d[4], d[5]),
        "Vdc": s16(d[6], d[7]) * 0.01
    }

def decode_422(d):
    # Vd/Vq feed-forward as FOC_Loop.c applies it (clamped), Vq FF before the
    # clamp, and the clamp limit 0.577*Vbus. Window means, 0.01 V.
    return {
        "Vd_FF": s16(d[0], d[1]) * 0.01,
        "Vq_FF": s16(d[2], d[3]) * 0.01,
        "Vq_FF_unsat": s16(d[4], d[5]) * 0.01,
        "FF_lim": s16(d[6], d[7]) * 0.01
    }

FAST_DECODERS = {
    0x420: decode_420,
    0x421: decode_421,
    0x422: decode_422,
}


# ----------------------------
# BENCH SLOW FRAMES Parser.py does not know (10 Hz, with the 0x40x cycle)
# ----------------------------

def decode_40A(d):
    # fdcan_AL.c Send_Data_On_CAN_40A: u16 LE; bytes 6-7 reserved. Count_At_Z is
    # the TIM2 count latched at the last Z pulse (only updated in FOC_START),
    # Z_Count the low 16 bits of the Z pulse counter, Count_From_Duty the count
    # the encoder was seeded with at boot from the PWM duty.
    return {
        "Count_At_Z": u16(d[0], d[1]),
        "Z_Count": u16(d[2], d[3]),
        "Count_From_Duty": u16(d[4], d[5])
    }

SLOW_DECODERS = dict(DECODERS)      # Parser.py's table, left untouched
SLOW_DECODERS[0x40A] = decode_40A
ENC_DIAG_COLS = ["Count_At_Z", "Z_Count", "Count_From_Duty"]    # last columns of the slow CSV


# ----------------------------
# FIRMWARE CONFIG (git show REV:path + a small C preprocessor)
# ----------------------------

# main.h include order: board_config.h, motor_profiles/motor_select.h
# (which includes the selected motor profile), vehicle_config.h.
CONFIG_HEADERS = [
    "Core/Inc/board_config.h",
    "Core/Inc/motor_profiles/motor_select.h",
    "Core/Inc/vehicle_config.h",
]

# Values written to <log>_config.json (in this order). Missing ones are listed
# under "missing" in the JSON.
CONFIG_KEYS = [
    "MOTOR_PROFILE", "HIRANGE_XR", "HILOAD_XR", "HIRANGE_TR",
    "OPERATION_MODE", "CAN_SPEED_ONLY",
    # motor model
    "POLEPAIRS", "RS", "LD", "LQ", "LAMBDA",
    "LD_PERCENTAGE_CHANGE", "LQ_PERCENTAGE_CHANGE", "FLUX_PERCENTAGE_CHANGE",
    "MOTOR_PEAK_ARMS", "MOTOR_PEAK_AC", "IQ_UP_LIMIT",
    "OP_VOLTAGE", "SVM_VOLTAGE_LIMIT",
    # field weakening
    "FW_CLOSED_LOOP", "FW_CL_KI", "FW_CL_VREF_PCT", "FW_CL_ID_FLOOR_K",
    "FW_CL_ACTIVE_A", "FW_CL_ID_ERR_A", "FW_CL_IQ_KI", "FW_CL_IQ_RELEASE_A_S",
    "FW_KFW", "FW_KAW", "FW_M2F", "FW_F2M", "FW_RST_FACTOR",
    # speed PI
    "SPEED_KP", "SPEED_KI", "SPEED_KD", "SPEED_KD_FILTER",
    "SPEED_PID_OUT_UPL", "SPEED_PID_OUT_LOWL", "SPEED_PID_BACK_KAW",
    # current PIs (one set; the low-speed set is this x LOW_SPEED_SCALING)
    "ID_KP", "ID_KI", "ID_KD", "ID_KD_FILTER",
    "ID_PID_OUT_UPL", "ID_PID_OUT_LOWL", "ID_PID_BACK_KAW",
    "IQ_KP", "IQ_KI", "IQ_KD", "IQ_KD_FILTER",
    "IQ_PID_OUT_UPL", "IQ_PID_OUT_LOWL", "IQ_PID_BACK_KAW",
    # gain-set switch
    "LOW_SPEED_TO_HIGH_SPEED_SWITCH_RPM", "LOW_SPEED_I_LIMIT",
    "LOW_SPEED_T_LIMIT", "LOW_SPEED_I_T_SWITCH", "LOW_SPEED_SCALING",
    # torque / speed envelope
    "FOC_TRQ_RAMP_UP_NM_S", "FOC_TRQ_RAMP_DOWN_NM_S",
    "TRQ_REQ_SLEW_UP_NM_S", "TRQ_REQ_SLEW_DOWN_NM_S",
    "SPORTS_MAX_SPEED", "ECO_MAX_SPEED", "OVERSPEED_TRIP_RPM", "OVERSPEED_COUNT",
    # FOC hand-edits
    "FOC_VDQ_CIRCLE_LIMIT", "FOC_VDQ_LIMIT_VD_MAX_FRAC",
    "FOC_ISENSE_DELAY_COMP", "FOC_ISENSE_DELAY_S", "FOC_ISENSE_DELAY_MAX_S",
    "FOC_VOUT_DELAY_COMP", "FOC_VOUT_DELAY_S", "FOC_ANGLE_ADVANCE",
    "FOC_VBUS_FROM_VDC", "FOC_VDC_LPF_TAU_S", "DEAD_TIME_COMP_ENABLE",
    "FOC_BENCH_FAST_TLM", "FAST_TLM_PERIOD_MS",
    # sensing / CAN / protection
    "ENCODER_PPR", "T_S", "T_F_ROTOR_SPEED", "CAN_BUS_CYCLE",
    "OLD_VOLT_DIV", "NEW_VOLT_DIV", "BUS_VDC_SCALE",
    "BUS_DC_OV_LIMIT", "BUS_DC_UV_LIMIT", "BUS_DC_OV_COUNT", "BUS_DC_UV_COUNT",
]

# Constant columns added to every slow-CSV row (cfg_<NAME>).
CONFIG_COLUMNS = [
    "RS", "LD", "LQ", "LAMBDA", "LD_PERCENTAGE_CHANGE", "LQ_PERCENTAGE_CHANGE",
    "POLEPAIRS", "OP_VOLTAGE", "FW_CL_KI", "FW_CL_VREF_PCT", "FW_CL_ID_FLOOR_K",
    "LOW_SPEED_SCALING", "LOW_SPEED_T_LIMIT", "FOC_VDQ_CIRCLE_LIMIT",
    "FOC_ISENSE_DELAY_COMP", "FOC_VOUT_DELAY_COMP", "FOC_VOUT_DELAY_S",
    "FOC_VBUS_FROM_VDC",
]

_C_TYPES = {"float", "double", "int", "unsigned", "signed", "long", "short",
            "char", "bool", "uint8_t", "uint16_t", "uint32_t", "uint64_t",
            "int8_t", "int16_t", "int32_t", "int64_t", "volatile"}

_PP_TOKEN = re.compile(
    r"\s*(0[xX][0-9a-fA-F]+[uUlL]*"
    r"|(?:\d+\.\d*|\.\d+|\d+)(?:[eE][+-]?\d+)?[fFuUlL]*"
    r"|[A-Za-z_]\w*"
    r"|&&|\|\||==|!=|<=|>=|<<|>>"
    r"|\"[^\"]*\"|'[^']*'"
    r"|\S)")
_PP_NUMBER = re.compile(r"^(0[xX][0-9a-fA-F]+|(?:\d+\.\d*|\.\d+|\d+)(?:[eE][+-]?\d+)?)[fFuUlL]*$")
_PP_IDENT = re.compile(r"^[A-Za-z_]\w*$")


def _pp_tokens(text):
    return _PP_TOKEN.findall(text)


def _pp_strip_comments(text):
    # keep newlines so line numbers survive
    text = re.sub(r"/\*.*?\*/", lambda m: "\n" * m.group(0).count("\n"), text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def _pp_logical_lines(text):
    """(line number, text) with backslash continuations joined."""
    out = []
    buf, start = "", None
    for n, line in enumerate(_pp_strip_comments(text).split("\n"), 1):
        if start is None:
            start = n
        if line.rstrip().endswith("\\"):
            buf += line.rstrip()[:-1] + " "
            continue
        out.append((start, buf + line))
        buf, start = "", None
    return out


def _pp_expand(tokens, defines, unknown_zero, depth=0):
    """Macro-expand object-like macros. Returns a token list, or None if an
    identifier cannot be resolved and unknown_zero is False."""
    out = []
    i = 0
    while i < len(tokens):
        t = tokens[i]
        if t == "defined":
            if i + 1 < len(tokens) and tokens[i + 1] == "(":
                name = tokens[i + 2]
                i += 4
            else:
                name = tokens[i + 1]
                i += 2
            out.append("1" if name in defines else "0")
            continue
        # drop C casts: ( type )
        if t == "(" and i + 2 < len(tokens) and tokens[i + 1] in _C_TYPES and tokens[i + 2] == ")":
            i += 3
            continue
        if t in ("true", "false"):
            out.append("1" if t == "true" else "0")
        elif _PP_IDENT.match(t):
            if t in defines and depth < 32:
                sub = _pp_expand(_pp_tokens(defines[t][0]), defines, unknown_zero, depth + 1)
                if sub is None:
                    return None
                out.append("(")
                out.extend(sub)
                out.append(")")
            elif unknown_zero:
                out.append("0")
            else:
                return None
        else:
            m = _PP_NUMBER.match(t)
            out.append(m.group(1) if m else t)
        i += 1
    return out


def _pp_pyexpr(tokens):
    """C expression tokens -> Python expression string."""
    out = []
    i = 0
    while i < len(tokens):
        t = tokens[i]
        if t == "&&":
            out.append(" and ")
        elif t == "||":
            out.append(" or ")
        elif t == "!":
            # bind ! to the next atom (C precedence), not the whole comparison
            if i + 1 < len(tokens) and tokens[i + 1] == "(":
                depth, j = 0, i + 1
                while j < len(tokens):
                    depth += tokens[j] == "("
                    depth -= tokens[j] == ")"
                    if depth == 0:
                        break
                    j += 1
                out.append("(not " + _pp_pyexpr(tokens[i + 1:j + 1]) + ")")
                i = j
            elif i + 1 < len(tokens):
                out.append("(not " + _pp_pyexpr([tokens[i + 1]]) + ")")
                i += 1
        elif t.startswith('"') or t.startswith("'"):
            raise ValueError("string literal")
        else:
            out.append(" " + t + " ")
        i += 1
    return "".join(out)


def _pp_eval(text, defines, unknown_zero=True):
    tokens = _pp_expand(_pp_tokens(text), defines, unknown_zero)
    if tokens is None:
        raise ValueError("unresolved identifier")
    v = eval(_pp_pyexpr(tokens), {"__builtins__": {}}, {})
    if isinstance(v, bool):
        v = int(v)
    return v


def _pp_run(path, text, defines, loader=None, active_out=None, errors=None):
    """Run the conditional / #define / #include logic over one file.
    defines: name -> (body, "path:line"). active_out collects (line, text)
    of non-directive lines that survive the conditionals."""
    def cond_of(expr, n):
        try:
            return bool(_pp_eval(expr, defines))
        except Exception as e:
            if errors is not None:
                errors.append(f"{path}:{n}: cannot evaluate #if {expr!r} ({e}); taken as false")
            return False

    stack = []          # [parent_active, branch_taken]
    active = True
    for n, line in _pp_logical_lines(text):
        m = re.match(r"^\s*#\s*(\w+)\s*(.*)$", line)
        if not m:
            if active and active_out is not None:
                active_out.append((n, line))
            continue
        kw, rest = m.group(1), m.group(2).strip()
        if kw in ("if", "ifdef", "ifndef"):
            if not active:
                cond = False
            elif kw == "if":
                cond = cond_of(rest, n)
            elif kw == "ifdef":
                cond = rest.split()[0] in defines
            else:
                cond = rest.split()[0] not in defines
            stack.append([active, cond])
            active = cond
        elif kw == "elif":
            parent, taken = stack[-1]
            cond = parent and not taken and cond_of(rest, n)
            stack[-1][1] = taken or cond
            active = cond
        elif kw == "else":
            parent, taken = stack[-1]
            active = parent and not taken
            stack[-1][1] = True
        elif kw == "endif":
            active = stack.pop()[0]
        elif not active:
            continue
        elif kw == "define":
            dm = re.match(r"^([A-Za-z_]\w*)(\(?)(.*)$", rest)
            if dm and not dm.group(2):          # object-like macros only
                defines[dm.group(1)] = (dm.group(3).strip(), f"{path}:{n}")
        elif kw == "undef":
            defines.pop(rest.split()[0], None)
        elif kw == "include" and loader is not None:
            im = re.match(r'^"([^"]+)"', rest)
            if im:
                loader(im.group(1), path)
        elif kw == "error" and errors is not None:
            errors.append(f"{path}:{n}: #error {rest}")


def _git_read(rev, path):
    """File text at a git rev, or from the working tree for rev 'worktree'.
    None if it does not exist."""
    if rev.lower() == "worktree":
        full = os.path.join(REPO_ROOT, path)
        if not os.path.isfile(full):
            return None
        with open(full, encoding="utf-8", errors="replace") as f:
            return f.read()
    r = subprocess.run(["git", "-C", REPO_ROOT, "show", f"{rev}:{path}"],
                       capture_output=True)
    if r.returncode != 0:
        return None
    return r.stdout.decode("utf-8", errors="replace")


def _git(*args):
    r = subprocess.run(["git", "-C", REPO_ROOT] + list(args), capture_output=True)
    return r.stdout.decode("utf-8", errors="replace").strip() if r.returncode == 0 else None


def _pp_value(name, defines):
    if name not in defines:
        return None
    try:
        return _pp_eval(defines[name][0], defines, unknown_zero=False)
    except Exception:
        return defines[name][0]


def _active_source(rev, path, defines):
    """Non-directive lines of a .c file that survive the config #ifs."""
    text = _git_read(rev, path)
    if text is None:
        return None
    lines = []
    _pp_run(path, text, dict(defines), active_out=lines)
    return lines


def _source_expr(lines, path, pattern, defines):
    """First active line matching pattern; group 1 is evaluated."""
    if not lines:
        return None
    for n, line in lines:
        m = re.search(pattern, line)
        if m:
            expr = m.group(1).strip()
            try:
                value = _pp_eval(expr, defines, unknown_zero=False)
            except Exception:
                value = expr
            return {"value": value, "expr": expr, "src": f"{path}:{n}"}
    return None


def load_fw_config(rev):
    defines = {}
    errors = []
    sha = describe = None
    if rev.lower() != "worktree":
        sha = _git("rev-parse", f"{rev}^{{commit}}")
        if sha is None:
            raise SystemExit(f"git rev '{rev}' not found in {REPO_ROOT}")
        describe = _git("describe", "--tags", "--always", sha)

    def loader(name, from_path):
        for cand in (os.path.dirname(from_path) + "/" + name, "Core/Inc/" + name):
            cand = os.path.normpath(cand).replace("\\", "/")
            text = _git_read(rev, cand)
            if text is not None:
                _pp_run(cand, text, defines, loader, errors=errors)
                return

    for path in CONFIG_HEADERS:
        text = _git_read(rev, path)
        if text is None:
            errors.append(f"{path}: not found at {rev}")
            continue
        _pp_run(path, text, defines, loader, errors=errors)

    params, missing = {}, []
    for k in CONFIG_KEYS:
        if k in defines:
            params[k] = {"value": _pp_value(k, defines), "expr": defines[k][0], "src": defines[k][1]}
        else:
            missing.append(k)

    # Parameters that live in .c files, not #defines
    src = {}
    main_c = _active_source(rev, "Core/Src/main.c", defines)
    foc_c = _active_source(rev, "Core/Src/FOC_Loop.c", defines)
    rx_c = _active_source(rev, "Core/Src/can_rx_handlers.c", defines)
    al_c = _active_source(rev, "Core/Src/fdcan_AL.c", defines)

    m = re.compile(r"RateLimiter_Init\(\s*&limiter\s*,\s*([^,]+),")
    src["SPEED_REF_RATE_UP_RPM_S"] = _source_expr(main_c, "Core/Src/main.c", m, defines)
    m = re.compile(r"RateLimiter_Init\(\s*&limiter\s*,[^,]+,\s*([^,]+),")
    src["SPEED_REF_RATE_DOWN_RPM_S"] = _source_expr(main_c, "Core/Src/main.c", m, defines)
    for key, field in (("MODEL_REF_SPEED_RATE_UP", "Ref_Speed_rate_up"),
                       ("MODEL_REF_SPEED_RATE_DOWN", "Ref_Speed_rate_down"),
                       ("MODEL_REF_SPEED_RATE_DOWN_NEUTRAL", "Ref_Speed_rate_down_neutral"),
                       ("ID_FILTER_COEFF", "Id_Filter_coefficient"),
                       ("IQ_FILTER_COEFF", "Iq_Filter_coefficient"),
                       ("ID_FILTER_SWITCH", "Id_Filter_switch"),
                       ("IQ_FILTER_SWITCH", "Iq_Filter_switch")):
        src[key] = _source_expr(foc_c, "Core/Src/FOC_Loop.c",
                                r"FOC_Loop_U\.\w+\." + field + r"\s*=\s*([^;]+);", defines)

    # Gain-set switch (FOC_Loop.c hand-edit ~l.334): high set needs
    # rpm >= LOW_SPEED_TO_HIGH_SPEED_SWITCH_RPM AND (measured torque >= T_limit
    # if I_0_T_1_switch else hypot(Id,Iq) >= I_limit). The generated code had
    # T_limit <= 0.0F (constant false) -> the high set was unreachable.
    switch = {"torque_branch": "unknown", "src": None}
    for n, line in (foc_c or []):
        if "T_limit <=" in line and "Actual_torque_from_meas" in line:
            switch = {"torque_branch": "measured_torque", "src": f"Core/Src/FOC_Loop.c:{n}"}
            break
        if re.search(r"T_limit\s*<=\s*0\.0F", line):
            switch = {"torque_branch": "constant_false", "src": f"Core/Src/FOC_Loop.c:{n}"}
            break
    for n, line in (foc_c or []):
        if "FOC_Loop_Y.PID_Switch =" in line:
            switch["speed_src"] = f"Core/Src/FOC_Loop.c:{n}"
            break

    rx_ids = {}
    for n, line in (rx_c or []):
        mm = re.search(r"\{\s*(0x[0-9A-Fa-f]+)\s*,\s*(can_rx_\w+)\s*\}", line)
        if mm:
            rx_ids[int(mm.group(1), 16)] = {"handler": mm.group(2), "src": f"Core/Src/can_rx_handlers.c:{n}"}
    filt_ids = {}
    for n, line in (al_c or []):
        mm = re.search(r"_fdcan_filter_IDList\(\s*(0x[0-9A-Fa-f]+)\s*,\s*(\d)", line)
        if mm:
            filt_ids[int(mm.group(1), 16)] = f"Core/Src/fdcan_AL.c:{n}"
    # 0x729 is dispatched directly from main.c's RX callback
    accepted = sorted(i for i in CMD_DECODERS if i in filt_ids and (i in rx_ids or i == 0x729))

    return {
        "rev": rev,
        "sha": sha,
        "describe": describe,
        "repo": REPO_ROOT,
        "params": params,
        "source_params": {k: v for k, v in src.items() if v is not None},
        "gain_switch": switch,
        "rx_dispatch": {f"0x{k:03X}": v for k, v in sorted(rx_ids.items())},
        "rx_filters": {f"0x{k:03X}": v for k, v in sorted(filt_ids.items())},
        "rx_accepted_cmd_ids": [f"0x{i:03X}" for i in accepted],
        "missing": missing + [k for k, v in src.items() if v is None],
        "preprocessor_errors": errors,
        "_accepted": accepted,
    }


def cfg(config, name, default=None):
    p = config["params"].get(name) or config["source_params"].get(name)
    if p is None or isinstance(p["value"], str):
        return default
    return p["value"]


def default_tuning(config):
    """Firmware boot values (main.c can_d initialiser) of every Tune_* column."""
    tau = cfg(config, "FOC_ISENSE_DELAY_S", 0.0) if cfg(config, "FOC_ISENSE_DELAY_COMP", 0) else None
    return {
        "Tune_Speed_Kp": cfg(config, "SPEED_KP"),
        "Tune_Speed_Ki": cfg(config, "SPEED_KI"),
        "Tune_Speed_Kd": cfg(config, "SPEED_KD"),
        "Tune_Id_Kp": cfg(config, "ID_KP"),
        "Tune_Id_Ki": cfg(config, "ID_KI"),
        "Tune_Id_Kd": cfg(config, "ID_KD"),
        "Tune_Iq_Kp": cfg(config, "IQ_KP"),
        "Tune_Iq_Ki": cfg(config, "IQ_KI"),
        "Tune_Iq_Kd": cfg(config, "IQ_KD"),
        "Tune_Isense_Tau_us": None if tau is None else round(tau * 1e6, 3),
        "Tune_Fast_Telem": 0 if cfg(config, "FOC_BENCH_FAST_TLM", 0) else None,
        "Tune_Lambda": cfg(config, "LAMBDA"),
        "Tune_Ld": cfg(config, "LD"),
        "Tune_Lq": cfg(config, "LQ"),
        "Tune_Rs": cfg(config, "RS"),
        "Tune_Kfw": cfg(config, "FW_KFW"),
        "Tune_Kaw": cfg(config, "FW_KAW"),
        "Tune_M2F": cfg(config, "FW_M2F"),
        "Tune_F2M": cfg(config, "FW_F2M"),
        "Tune_FW_Rst": cfg(config, "FW_RST_FACTOR"),
        "Tune_Source": "fw_default",
    }


def gain_set(sig, tune, config):
    """Derived current/speed PI set (FOC_Loop.c PID_Switch) from one row's
    telemetry. Telemetry is a 10 Hz snapshot with 1 Nm / 1 A resolution, the
    switch runs every FOC step, so treat this as an estimate near the edges."""
    scale = cfg(config, "LOW_SPEED_SCALING", 1.0)
    rpm = sig.get("RPM_Fdbk")
    high = False
    if config["gain_switch"]["torque_branch"] != "constant_false" and rpm is not None:
        if rpm >= cfg(config, "LOW_SPEED_TO_HIGH_SPEED_SWITCH_RPM", 0.0):
            if cfg(config, "LOW_SPEED_I_T_SWITCH", 1):
                trq = sig.get("Actual_Torque")
                high = trq is not None and trq >= cfg(config, "LOW_SPEED_T_LIMIT", 0.0)
            else:
                i_d, i_q = sig.get("Id"), sig.get("Iq")
                high = i_d is not None and math.hypot(i_d, i_q) >= cfg(config, "LOW_SPEED_I_LIMIT", 0.0)
    k = 1.0 if high else scale
    out = {"Gain_Set": "HIGH" if high else f"LOW(x{scale:g})"}
    for loop in ("Speed", "Id", "Iq"):
        for g in ("Kp", "Ki"):
            v = tune.get(f"Tune_{loop}_{g}")
            out[f"Eff_{loop}_{g}"] = None if v is None else round(v * k, 6)
    return out


# ----------------------------
# TRC PARSER (Parser.py layout handling, plus command and fast frames)
# ----------------------------

def read_frames(filename):
    """[(timestamp_ms, can_id, data)] for every data frame, both directions."""
    version, columns = Parser._trc_layout(filename)
    v2 = version.startswith("2")
    if v2 and not columns:
        columns = ["N", "O", "T", "I", "d", "l", "D"]   # PCAN-View 2.0 default
    frames = []
    with open(filename) as f:
        for line in f:
            if line.startswith(";") or line.strip() == "":
                continue
            p = line.split()
            try:
                if v2:
                    timestamp, can_id, data = Parser._trc_row_v2(p, columns)
                else:
                    timestamp, can_id, data = Parser._trc_row_v1(p)
            except (ValueError, IndexError, KeyError):
                continue
            frames.append((timestamp, can_id, data))
    return frames


def parse_log(frames, config):
    """Walk the trace once. Returns (slow_rows, bursts, fast_rows, events).

    slow_rows: exactly Parser.parse_trc's rows (one per DECODERS frame,
               forward-filled) + tuning, derived and cfg columns.
    bursts:    one dict per telemetry cycle (starts at 0x401) with per-ID
               decodes (no cross-ID name collisions) and the tuning state.
    fast_rows: one row per 5 ms window of 0x420/0x421/0x422 frames, grouped
               by arrival time in any order (see FAST_GROUP_FRAC).
    events:    tuning/command changes and error-code changes."""
    accepted = set(config["_accepted"])
    tune = default_tuning(config)
    tau_max_us = (cfg(config, "FOC_ISENSE_DELAY_MAX_S", 400e-6) or 400e-6) * 1e6
    cfg_cols = {f"cfg_{k}": cfg(config, k) for k in CONFIG_COLUMNS}
    cfg_cols["cfg_fw_rev"] = config["describe"] or config["rev"]

    current_signals = {}
    slow_rows, bursts, fast_rows, events = [], [], [], []
    burst = None
    last_err = None
    ignored = {}

    period_ms = cfg(config, "FAST_TLM_PERIOD_MS", 5) or 5
    group, group_t0, group_tune = {}, None, None     # fast window being collected
    fast_n, fast_first, fast_last = {}, {}, {}        # per-ID frame count and span

    for timestamp, can_id, data in frames:

        if can_id in CMD_DECODERS:
            if len(data) < 1 or (can_id in (0x102, 0x114, 0x115) and len(data) < 2) \
                    or (can_id in (0x103, 0x104, 0x105, 0x106, 0x107) and len(data) < 6):
                continue
            decoded = CMD_DECODERS[can_id](data)
            if accepted and can_id not in accepted:
                # the firmware rev has no filter/handler for this ID
                ignored[can_id] = ignored.get(can_id, 0) + 1
                continue
            if can_id == 0x115:
                decoded["Tune_Isense_Tau_us"] = min(decoded["Tune_Isense_Tau_us"], tau_max_us)
            changed = {k: v for k, v in decoded.items() if tune.get(k) != v}
            if changed and can_id in TUNING_IDS:
                tune["Tune_Source"] = f"0x{can_id:03X}@{timestamp / 1000:.3f}s"
                events.append((timestamp, f"0x{can_id:03X}", changed))
            elif changed and can_id not in (0x102, 0x113):
                events.append((timestamp, f"0x{can_id:03X}", changed))
            tune.update(decoded)
            continue

        if can_id in FAST_DECODERS and len(data) >= 8:
            decoded = FAST_DECODERS[can_id](data)
            fast_n[can_id] = fast_n.get(can_id, 0) + 1
            fast_first.setdefault(can_id, timestamp)
            fast_last[can_id] = timestamp
            # The frames of one window leave the controller back to back but
            # arrive in either order (0x421 first in some logs). A window ends
            # when an ID repeats or the next frame is more than half a period
            # after its first frame.
            if group and (can_id in group or timestamp - group_t0 > period_ms * FAST_GROUP_FRAC):
                fast_rows.append(_fast_row(group, group_tune))
                group = {}
            if not group:
                group_t0 = timestamp
            group[can_id] = (timestamp, decoded)
            group_tune = {k: tune.get(k) for k in FAST_TUNE_KEYS}   # state as of the window's last frame
            continue

        if can_id not in SLOW_DECODERS or (can_id == 0x40A and len(data) < 6):
            continue

        decoded = SLOW_DECODERS[can_id](data)
        current_signals.update(decoded)

        row = {"timestamp": timestamp}
        row.update(current_signals)
        row.update({k: tune.get(k) for k in TUNE_COLS})
        row.update(gain_set(current_signals, tune, config))
        row.update(cfg_cols)
        slow_rows.append(row)

        if can_id == 0x401:
            if burst is not None:
                bursts.append(burst)
            burst = {"t": timestamp / 1000.0, "row": None, "tune": dict(tune)}
        if burst is not None and can_id not in burst:
            burst[can_id] = decoded
            burst["row"] = row
            if can_id == 0x403:
                burst["gain"] = gain_set(current_signals, tune, config)
        if can_id == 0x403 and decoded["Error_Code"] != last_err:
            if last_err is not None:
                events.append((timestamp, "0x403", {"Error_Name": decoded["Error_Name"]}))
            last_err = decoded["Error_Code"]

    if burst is not None:
        bursts.append(burst)
    if group:
        fast_rows.append(_fast_row(group, group_tune))

    if fast_rows:
        _fast_gaps(fast_rows, period_ms)
        span_s = (max(fast_last.values()) - min(fast_first.values())) / 1000.0
        config["fast_telemetry"] = {
            "frames": {f"0x{i:03X}": fast_n[i] for i in sorted(fast_n)},
            "span_s": round(span_s, 3),
            "rate_per_s": {f"0x{i:03X}": round((fast_n[i] - 1) * 1000.0 / (fast_last[i] - fast_first[i]), 2)
                           for i in sorted(fast_n) if fast_last[i] > fast_first[i]},
            "nominal_per_s": 1000.0 / period_ms,
            "rows": len(fast_rows),
            "unpaired_rows": sum(1 for r in fast_rows if not r["pair_ok"]),
            "missed_frames": sum(r["missed_frames"] for r in fast_rows),
        }

    for can_id, n in ignored.items():
        config.setdefault("warnings", []).append(
            f"0x{can_id:03X}: {n} frame(s) in the log, but rev {config['rev']} has no RX filter/handler for it "
            "-> ignored (tuning state unchanged). Wrong --fw-rev?")
    return slow_rows, bursts, fast_rows, events


FAST_GROUP_FRAC = 0.5       # a window's frames all arrive within this x FAST_TLM_PERIOD_MS of its first
FAST_TUNE_KEYS = ("Tune_Isense_Tau_us", "Tune_Id_Kp", "Tune_Id_Ki", "Tune_Iq_Kp", "Tune_Iq_Ki",
                  "Tune_Speed_Kp", "Tune_Speed_Ki", "Tune_Source")
FAST_OFF_GAP_MS = 1000.0    # a longer silence is telemetry switched off, not loss
FAST_CATCHUP_ROWS = 40      # frames queued behind the 100 ms slow-frame burst (up to ~13 ms late)
                            # are back in step within this many rows


def _fast_row(group, tune):
    """One CSV row from {id: (timestamp, decoded)}. Timestamp is the 0x420
    arrival (as before), else 0x421, else 0x422. pair_ok = both 0x420 and
    0x421 present; 0x422 only fills its own columns."""
    row = {"timestamp": None}
    for cid in sorted(group):
        if row["timestamp"] is None:
            row["timestamp"] = group[cid][0]
        row.update(group[cid][1])
    row["pair_ok"] = int(0x420 in group and 0x421 in group)
    row.update(tune)
    return row


def _fast_gaps(rows, period_ms):
    """dt_ms (row to row, as before) and missed_frames: 5 ms windows that never
    arrived, judged from frame counts against elapsed time, not from the
    row-to-row gap.

    A row-to-row gap over 1.5 periods used to count as loss, but the 100 ms slow
    burst delays the fast frames by up to ~13 ms and they then arrive back to
    back, so jitter read as hundreds of lost frames in a log with none. Instead,
    S_j = t_j - j*period is the arrival delay plus a slow ramp from the
    controller's clock error. Queue delay is positive and short-lived; a lost
    window steps S up by one period for good. So the baseline is the minimum of
    S over the next FAST_CATCHUP_ROWS rows, and loss is a step in that baseline
    of half a period or more. Comparing consecutive baselines (not absolute
    time) makes the result independent of the clock error. The last
    FAST_CATCHUP_ROWS rows of a run cannot be verified and are not counted.
    Silence longer than FAST_OFF_GAP_MS ends a run (telemetry was switched off)."""
    last_t = None
    for r in rows:
        t = r["timestamp"]
        r["dt_ms"] = None if last_t is None else round(t - last_t, 2)
        r["missed_frames"] = 0
        last_t = t

    start = 0
    for i in range(1, len(rows) + 1):
        if i < len(rows) and rows[i]["timestamp"] - rows[i - 1]["timestamp"] <= FAST_OFF_GAP_MS:
            continue
        run = rows[start:i]
        start = i
        w = FAST_CATCHUP_ROWS
        if len(run) <= w:
            continue
        s = [r["timestamp"] - j * period_ms for j, r in enumerate(run)]
        base, dq = [], collections.deque()          # sliding minimum over s[j - w + 1 : j + 1]
        for j, v in enumerate(s):
            while dq and s[dq[-1]] >= v:
                dq.pop()
            dq.append(j)
            if dq[0] <= j - w:
                dq.popleft()
            if j >= w - 1:
                base.append(s[dq[0]])
        # base[k] is the minimum over rows k .. k+w-1
        for k in range(1, len(base)):
            lost = int((base[k] - base[k - 1]) / period_ms + 0.5)
            if lost > 0:
                run[k]["missed_frames"] = lost


# ----------------------------
# SANITY CHECKS (telemetry echoes vs the config)
# ----------------------------

def check_config(config, frames, bursts):
    warn = config.setdefault("warnings", [])
    notes = config.setdefault("checks", [])

    # 0x7B0: low 32 bits of the build's `git rev-parse --short HEAD`
    for _, can_id, data in frames:
        if can_id == 0x7B0 and len(data) >= 4:
            log_sha = Parser.decode_7B0(data)["Git_SHA"].lower()
            config["log_git_sha"] = log_sha
            sha = config["sha"]
            if sha is None:
                notes.append(f"0x7B0 build SHA {log_sha} (rev is the working tree, not checked)")
            elif sha.startswith(log_sha) or sha[:8] == log_sha:
                notes.append(f"0x7B0 build SHA {log_sha} matches {config['rev']}")
            else:
                full = _git("rev-parse", "--verify", "--quiet", f"{log_sha}^{{commit}}")
                rel = ""
                if full and subprocess.run(["git", "-C", REPO_ROOT, "merge-base", "--is-ancestor", full, sha]).returncode == 0:
                    n = _git("rev-list", "--count", f"{full}..{sha}")
                    rel = f" ({full[:8]} is {n} commit(s) before {config['rev']}: built from a working tree on top of it - " \
                          f"fine if that tree was then committed as {config['rev']})"
                warn.append(f"0x7B0: firmware was built at {log_sha}, --fw-rev {config['rev']} is {sha[:8]}{rel}")
            break

    # first telemetry echoes, before any tuning command
    first = next((b for b in bursts if b["tune"]["Tune_Source"] == "fw_default"
                  and all(k in b for k in (0x404, 0x406, 0x407, 0x408, 0x409))), None)
    if first is None:
        return
    checks = [
        ("0x407 Speed_Kp", first[0x407]["Speed_Kp"], cfg(config, "SPEED_KP"), 0.011),
        ("0x407 Speed_Ki", first[0x407]["Speed_Ki"], cfg(config, "SPEED_KI"), 0.011),
        ("0x408 Id(Flux)_Kp", first[0x408]["Flux_Kp"], cfg(config, "ID_KP"), 0.011),
        ("0x408 Id(Flux)_Ki", first[0x408]["Flux_Ki"], cfg(config, "ID_KI"), 0.011),
        ("0x409 Iq(Torque)_Kp", first[0x409]["Torque_Kp"], cfg(config, "IQ_KP"), 0.011),
        ("0x409 Iq(Torque)_Ki", first[0x409]["Torque_Ki"], cfg(config, "IQ_KI"), 0.011),
        ("0x404 Rs", first[0x404]["Rs"], cfg(config, "RS"), 1.5e-6),
    ]
    if not cfg(config, "FLUX_PERCENTAGE_CHANGE", 0):
        checks.append(("0x404 Lambda", first[0x404]["Lambda"], cfg(config, "LAMBDA"), 1.5e-5))
    if cfg(config, "FOC_ISENSE_DELAY_COMP", 0):
        checks.append(("0x406 Isense_Delay_us", first[0x406]["Isense_Delay_us"],
                       (cfg(config, "FOC_ISENSE_DELAY_S", 0.0) or 0.0) * 1e6, 1.0))
    for name, seen, want, tol in checks:
        if want is None:
            continue
        if abs(seen - want) <= tol:
            notes.append(f"{name} echo {seen:g} == config {want:g}")
        else:
            warn.append(f"{name} echo {seen:g} != config {want:g} at t={first['t']:.1f}s - config does not match the flashed build")


# ----------------------------
# SCORECARD
# ----------------------------

HOLD_MIN_S = 5.0
HOLD_SKIP_S = 2.0
ALT_FLOOR_SLOW_A = 1.5      # > one 1 A LSB of 0x402
ALT_FLOOR_FAST_A = 0.5
TRUNC = 0.5                 # (uint)(x + offset) floors -> true mean ~ decoded + 0.5


def _mean(x):
    x = [v for v in x if v is not None]
    return sum(x) / len(x) if x else None


def _std(x):
    x = [v for v in x if v is not None]
    if len(x) < 2:
        return None
    m = sum(x) / len(x)
    return math.sqrt(sum((v - m) ** 2 for v in x) / (len(x) - 1))


def _rms(x):
    x = [v for v in x if v is not None]
    return math.sqrt(sum(v * v for v in x) / len(x)) if x else None


def find_holds(bursts):
    """[(ref, t_first, t_last)] for runs of constant RPM_Ref >= HOLD_MIN_S."""
    holds = []
    run = []
    for b in bursts + [None]:
        ref = b[0x401]["RPM_Ref"] if b is not None else None
        if run and (b is None or ref != run[0][0x401]["RPM_Ref"]):
            r0 = run[0][0x401]["RPM_Ref"]
            if r0 > 0 and run[-1]["t"] - run[0]["t"] >= HOLD_MIN_S:
                holds.append((r0, run[0]["t"], run[-1]["t"]))
            run = []
        if b is not None:
            run.append(b)
    return holds


def implied_tau(rpm, vd, vq, i_d, i_q, rs, ld, lq, lam, pp, vout_rot_s):
    """Solve the steady-state dq voltage equations for the current the
    voltages imply, and compare its angle with the measured current.
        Vd = Rs*Id - we*Lq*Iq
        Vq = Rs*Iq + we*(Ld*Id + lam)
    Vd/Vq telemetry is in the output frame (advanced by we*FOC_VOUT_DELAY_S
    when FOC_VOUT_DELAY_COMP, i.e. the rotor frame at PWM time). Without
    that comp the applied voltage is the commanded one rotated by
    -we*vout_rot_s. A measured current sampled tau late reads rotated by
    -we*tau, so tau = -(angle(I_meas) - angle(I_true)) / we."""
    we = rpm * 2.0 * math.pi / 60.0 * pp
    if abs(we) < 1.0:
        return None
    if vout_rot_s:
        a = -we * vout_rot_s
        vd, vq = vd * math.cos(a) - vq * math.sin(a), vd * math.sin(a) + vq * math.cos(a)
    b = vq - we * lam
    det = rs * rs + we * we * ld * lq
    id_t = (rs * vd + we * lq * b) / det
    iq_t = (rs * b - we * ld * vd) / det
    delta = math.atan2(i_q, i_d) - math.atan2(iq_t, id_t)
    delta = (delta + math.pi) % (2.0 * math.pi) - math.pi
    return {
        "Id_true": id_t,
        "Iq_true": iq_t,
        "I_ratio": math.hypot(id_t, iq_t) / max(math.hypot(i_d, i_q), 1e-9),
        "delta_deg": math.degrees(delta),
        "tau_us": -delta / we * 1e6,
    }


def _alternation(idr, iqr, floor):
    """Fraction of consecutive frame pairs where Id_ref and Iq_ref step in
    opposite directions, both by more than floor; plus RMS first differences."""
    d_id = [b - a for a, b in zip(idr, idr[1:])]
    d_iq = [b - a for a, b in zip(iqr, iqr[1:])]
    if not d_id:
        return None, None, None
    opp = sum(1 for x, y in zip(d_id, d_iq) if abs(x) > floor and abs(y) > floor and x * y < 0)
    return opp / len(d_id), _rms(d_id), _rms(d_iq)


def scorecard(bursts, fast_rows, config):
    pp = cfg(config, "POLEPAIRS", 5.0)
    rs, ld, lq, lam = cfg(config, "RS"), cfg(config, "LD"), cfg(config, "LQ"), cfg(config, "LAMBDA")
    vout_rot = 0.0 if cfg(config, "FOC_VOUT_DELAY_COMP", 0) else (cfg(config, "FOC_VOUT_DELAY_S", 85e-6) or 85e-6)
    circle = bool(cfg(config, "FOC_VDQ_CIRCLE_LIMIT", 0))
    period_s = (cfg(config, "FAST_TLM_PERIOD_MS", 5) or 5) / 1000.0

    out = []
    for ref, t0, t1 in find_holds(bursts):
        a = t0 + HOLD_SKIP_S
        win = [b for b in bursts if a <= b["t"] <= t1 and 0x402 in b and 0x403 in b]
        if len(win) < 5:
            continue
        fast = [r for r in fast_rows if a * 1000 <= r["timestamp"] <= t1 * 1000 and r["pair_ok"]]
        use_fast = len(fast) >= 0.8 * (t1 - a) / period_s

        g = lambda cid, k: [b[cid][k] if cid in b else None for b in win]
        if use_fast:
            rpm = [abs(r["RPM"]) for r in fast]
            i_d = [r["Id"] for r in fast]
            i_q = [r["Iq"] for r in fast]
            idr = [r["Id_ref"] for r in fast]
            iqr = [r["Iq_ref"] for r in fast]
            vd = [r["Vd"] for r in fast]
            vq = [r["Vq"] for r in fast]
            vdc = [r["Vdc"] for r in fast]
            corr, floor = 0.0, ALT_FLOOR_FAST_A
        else:
            rpm = g(0x401, "RPM_Fdbk")
            i_d = g(0x401, "Id")
            i_q = g(0x401, "Iq")
            idr = g(0x402, "Id_ref")
            iqr = g(0x402, "Iq_ref")
            vd = g(0x402, "Vd")
            vq = g(0x402, "Vq")
            vdc = g(0x403, "Bus_Voltage")
            corr, floor = TRUNC, ALT_FLOOR_SLOW_A

        m = lambda x: None if _mean(x) is None else _mean(x) + corr
        rpm_m = m(rpm)
        row = {
            "ref_rpm": ref,
            "t_start_s": round(t0, 2),
            "t_end_s": round(t1, 2),
            "duration_s": round(t1 - t0, 2),
            "win_s": round(t1 - a, 2),
            "src": "fast" if use_fast else "slow",
            "n": len(fast) if use_fast else len(win),
            "rpm_mean": rpm_m,
            "rpm_sd": _std(rpm),
            "rpm_pp": max(rpm) - min(rpm),
            "Id_mean": m(i_d), "Id_sd": _std(i_d),
            "Iq_mean": m(i_q), "Iq_sd": _std(i_q),
            "Id_ref_mean": m(idr), "Id_ref_sd": _std(idr),
            "Iq_ref_mean": m(iqr), "Iq_ref_sd": _std(iqr),
            "Vd_mean": m(vd), "Vq_mean": m(vq),
        }
        e_d = [x - y for x, y in zip(i_d, idr)]
        e_q = [x - y for x, y in zip(i_q, iqr)]
        row.update({"eId_mean": _mean(e_d), "eId_sd": _std(e_d),
                    "eIq_mean": _mean(e_q), "eIq_sd": _std(e_q)})
        alt, d_idr, d_iqr = _alternation(idr, iqr, floor)
        row.update({"alt_frac": alt, "alt_floor_A": floor, "dIdref_rms": d_idr, "dIqref_rms": d_iqr})
        row.update({"Vdc_min": min(vdc), "Vdc_max": max(vdc)})

        mi = [v for v in g(0x715, "Mod_Index_Peak") if v is not None]
        row["MI_demand_peak"] = max(mi) if mi else None
        row["MI_demand_median"] = sorted(mi)[len(mi) // 2] if mi else None
        clip = [v for v in g(0x715, "VLim_Clip_pct") if v is not None] if circle else []
        row["clip_pct_mean"] = _mean(clip) if clip else None
        row["clip_pct_max"] = max(clip) if clip else None
        trq = g(0x403, "Actual_Torque")
        row["Trq_meas_mean"] = _mean(trq)
        row["Trq_ref_mean"] = _mean(g(0x724, "Torque_ref"))

        errs = sorted({b[0x403]["Error_Name"] for b in win if b[0x403]["Error_Code"]})
        e706 = 0
        e708a = e708b = 0
        for b in win:
            e706 |= b[0x706]["Error_Code"] if 0x706 in b else 0
            e708a |= b[0x708]["Error_Code_1"] if 0x708 in b else 0
            e708b |= b[0x708]["Error_Code_2"] if 0x708 in b else 0
        states = sorted({b[0x405]["State"] for b in win if 0x405 in b})
        overrun = sum(1 for b in win if 0x405 in b and b[0x405]["Overrun_Flag"])
        row.update({"errors": ";".join(errs), "err_706": e706, "err_708_c1": e708a, "err_708_c2": e708b,
                    "cS": ";".join(str(s) for s in states), "overrun_frames": overrun})

        tunes = [b["tune"] for b in win]
        t_last = tunes[-1]
        for k in ("Tune_Speed_Kp", "Tune_Speed_Ki", "Tune_Id_Kp", "Tune_Id_Ki",
                  "Tune_Iq_Kp", "Tune_Iq_Ki", "Tune_Isense_Tau_us", "Tune_Fast_Telem"):
            row[k] = t_last.get(k)
        row["Tune_changed_in_window"] = int(any(
            any(t.get(k) != t_last.get(k) for k in TUNE_COLS if k.startswith("Tune_")) for t in tunes))
        sets = sorted({b["gain"]["Gain_Set"] for b in win if "gain" in b})
        row["Gain_Set"] = ";".join(sets)
        gl = win[-1].get("gain", {})
        for k in ("Eff_Id_Kp", "Eff_Id_Ki", "Eff_Iq_Kp", "Eff_Iq_Ki", "Eff_Speed_Kp", "Eff_Speed_Ki"):
            row[k] = gl.get(k)

        # implied current-sense lag
        tau_set = (t_last.get("Tune_Isense_Tau_us") or 0.0)
        res = None
        if None not in (rs, ld, lq, lam) and None not in (row["Vd_mean"], row["Vq_mean"], row["Id_mean"], row["Iq_mean"]):
            res = implied_tau(rpm_m, row["Vd_mean"], row["Vq_mean"], row["Id_mean"], row["Iq_mean"],
                              rs, ld, lq, lam, pp, vout_rot)
        if res:
            row.update({"Id_true": res["Id_true"], "Iq_true": res["Iq_true"], "I_true_over_meas": res["I_ratio"],
                        "delta_deg": res["delta_deg"], "tau_resid_us": res["tau_us"],
                        "tau_total_us": res["tau_us"] + tau_set})
        # same, with the firmware's live LUT model (0x404: derated Ld/Lq)
        lut = [b[0x404] for b in win if 0x404 in b]
        if lut and None not in (row["Vd_mean"], row["Id_mean"]):
            r2 = implied_tau(rpm_m, row["Vd_mean"], row["Vq_mean"], row["Id_mean"], row["Iq_mean"],
                             _mean([x["Rs"] for x in lut]), _mean([x["Ld"] for x in lut]),
                             _mean([x["Lq"] for x in lut]), _mean([x["Lambda"] for x in lut]), pp, vout_rot)
            if r2:
                row["tau_resid_us_lut"] = r2["tau_us"]
                row["Ld_lut_uH"] = _mean([x["Ld"] for x in lut]) * 1e6
                row["Lq_lut_uH"] = _mean([x["Lq"] for x in lut]) * 1e6
        # uncertainty from telemetry resolution: +-half an LSB on the voltage
        # means plus +-half an LSB on the current means (slow: 1 V / 1 A,
        # the +0.5 truncation correction is only right on average)
        if res:
            hv, hi = (0.005, 0.005) if use_fast else (0.5, 0.5)
            dv = dc = 0.0
            for sv, si in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
                rv = implied_tau(rpm_m, row["Vd_mean"] + sv * hv, row["Vq_mean"] + si * hv,
                                 row["Id_mean"], row["Iq_mean"], rs, ld, lq, lam, pp, vout_rot)
                ri = implied_tau(rpm_m, row["Vd_mean"], row["Vq_mean"],
                                 row["Id_mean"] + sv * hi, row["Iq_mean"] + si * hi, rs, ld, lq, lam, pp, vout_rot)
                dv = max(dv, abs(rv["tau_us"] - res["tau_us"]))
                dc = max(dc, abs(ri["tau_us"] - res["tau_us"]))
            row["tau_unc_us"] = dv + dc
            row["tau_confidence"] = "ok" if row["tau_unc_us"] <= min(75.0, 0.25 * abs(res["tau_us"])) else "low"
        out.append(row)
    return out


SCORE_PRINT = [
    ("ref", "ref_rpm", "{:.0f}"), ("dur", "duration_s", "{:.1f}"), ("src", "src", "{}"),
    ("rpm", "rpm_mean", "{:.1f}"), ("sd", "rpm_sd", "{:.2f}"), ("pp", "rpm_pp", "{:.0f}"),
    ("Id", "Id_mean", "{:.1f}"), ("Iq", "Iq_mean", "{:.1f}"),
    ("Idr", "Id_ref_mean", "{:.1f}"), ("Iqr", "Iq_ref_mean", "{:.1f}"),
    ("eIdsd", "eId_sd", "{:.2f}"), ("eIqsd", "eIq_sd", "{:.2f}"),
    ("alt%", "alt_frac", "{:.0%}"), ("dIdr", "dIdref_rms", "{:.2f}"), ("dIqr", "dIqref_rms", "{:.2f}"),
    ("Vd", "Vd_mean", "{:.2f}"), ("Vq", "Vq_mean", "{:.2f}"),
    ("Vdc", None, None), ("MIpk", "MI_demand_peak", "{:.2f}"), ("clip", "clip_pct_max", "{:.0f}"),
    ("set", "Gain_Set", "{}"), ("err", "errors", "{}"),
    ("Iq_v", "Iq_true", "{:.1f}"), ("dlt", "delta_deg", "{:.1f}"),
    ("tau_us", "tau_resid_us", "{:.0f}"), ("+-", "tau_unc_us", "{:.0f}"), ("tauLUT", "tau_resid_us_lut", "{:.0f}"), ("conf", "tau_confidence", "{}"),
]


def print_scorecard(rows):
    head = [h for h, _, _ in SCORE_PRINT]
    table = []
    for r in rows:
        line = []
        for h, k, f in SCORE_PRINT:
            if h == "Vdc":
                line.append(f"{r['Vdc_min']:.1f}-{r['Vdc_max']:.1f}")
            elif r.get(k) is None or r.get(k) == "":
                line.append("-")
            else:
                line.append(f.format(r[k]))
        table.append(line)
    w = [max(len(h), *(len(t[i]) for t in table)) if table else len(h) for i, h in enumerate(head)]
    print("  ".join(h.rjust(w[i]) for i, h in enumerate(head)))
    for t in table:
        print("  ".join(c.rjust(w[i]) for i, c in enumerate(t)))


# ----------------------------
# CSV WRITERS
# ----------------------------

def write_slow_csv(rows, filename, burst_rows=None):
    src = burst_rows if burst_rows is not None else rows
    extra = set(TUNE_COLS) | set(DERIVED_COLS)
    keys = set()
    for r in src:
        keys.update(r.keys())
    cfg_keys = sorted(k for k in keys if k.startswith("cfg_"))
    enc_keys = [k for k in ENC_DIAG_COLS if k in keys]      # last, so older logs' columns do not move
    base = ["timestamp"] + sorted(k for k in keys if k != "timestamp" and k not in extra
                                  and not k.startswith("cfg_") and k not in enc_keys)
    keys = base + [k for k in TUNE_COLS + DERIVED_COLS if k in keys] + cfg_keys + enc_keys
    with open(filename, "w", newline="") as f:
        # plain writer: DictWriter's per-row field check dominates at ~170 columns
        writer = csv.writer(f)
        writer.writerow(keys)
        for r in src:
            writer.writerow([r.get(k) for k in keys])


def write_rows(rows, filename, keys=None):
    if keys is None:
        keys = []
        for r in rows:
            for k in r:
                if k not in keys:
                    keys.append(k)
    with open(filename, "w", newline="") as f:
        writer = csv.DictWriter(f, keys, extrasaction="ignore")
        writer.writeheader()
        for r in rows:
            writer.writerow({k: (round(v, 6) if isinstance(v, float) else v) for k, v in r.items()})


FAST_KEYS = ["timestamp", "Id_ref", "Iq_ref", "Id", "Iq", "Vd", "Vq", "RPM", "Vdc",
             "pair_ok", "dt_ms", "missed_frames",
             "Tune_Isense_Tau_us", "Tune_Id_Kp", "Tune_Id_Ki", "Tune_Iq_Kp", "Tune_Iq_Ki",
             "Tune_Speed_Kp", "Tune_Speed_Ki", "Tune_Source",
             "Vd_FF", "Vq_FF", "Vq_FF_unsat", "FF_lim"]       # 0x422, empty for logs without it


# ----------------------------
# MAIN
# ----------------------------

def main():
    ap = argparse.ArgumentParser(description="Pegasus PCAN .trc -> telemetry + tuning context + scorecard")
    ap.add_argument("log", nargs="?", help=".trc file (file dialog if omitted)")
    ap.add_argument("--fw-rev", default="HEAD",
                    help="git rev of the flashed firmware (default HEAD; 'worktree' reads the files on disk)")
    ap.add_argument("--scorecard", action="store_true", help="detect speed holds and score them")
    ap.add_argument("--burst", action="store_true",
                    help="slow CSV with one row per telemetry cycle instead of Parser.py's one row per frame")
    ap.add_argument("--out", help="output directory (default: next to the log)")
    args = ap.parse_args()

    file_path = args.log
    if not file_path:
        Parser.Tk().withdraw()
        file_path = Parser.filedialog.askopenfilename(title="Select TRC log file",
                                                      filetypes=[("TRC files", "*.trc")])
        if not file_path:
            print("No file selected.")
            return

    out_dir = args.out or os.path.dirname(os.path.abspath(file_path))
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.join(out_dir, os.path.splitext(os.path.basename(file_path))[0])

    config = load_fw_config(args.fw_rev)
    frames = read_frames(file_path)
    slow_rows, bursts, fast_rows, events = parse_log(frames, config)
    check_config(config, frames, bursts)

    write_slow_csv(slow_rows, stem + "_slow.csv", [b["row"] for b in bursts if b["row"]] if args.burst else None)
    print("Slow CSV written to:", stem + "_slow.csv",
          f"({len(bursts) if args.burst else len(slow_rows)} rows, {len(bursts)} telemetry cycles)")

    if fast_rows:
        write_rows(fast_rows, stem + "_fast.csv", FAST_KEYS)
        ft = config["fast_telemetry"]
        print("Fast CSV written to:", stem + "_fast.csv",
              f"({ft['rows']} rows, {ft['missed_frames']} missed frame(s), {ft['unpaired_rows']} unpaired)")
        print("  fast frames over {:.1f} s: ".format(ft["span_s"]) +
              ", ".join(f"{i} {n}" + (f" ({ft['rate_per_s'][i]:.1f}/s)" if i in ft["rate_per_s"] else "")
                        for i, n in ft["frames"].items()) +
              f"  (nominal {ft['nominal_per_s']:.0f}/s per ID)")
    else:
        print("No 0x420/0x421/0x422 fast telemetry in this log.")

    config["log"] = os.path.abspath(file_path)
    config["events"] = [{"t_s": round(t / 1000.0, 3), "id": i, "changed": c} for t, i, c in events]
    dump = {k: v for k, v in config.items() if not k.startswith("_")}
    with open(stem + "_config.json", "w") as f:
        json.dump(dump, f, indent=1, default=str)
    print("Config JSON written to:", stem + "_config.json",
          f"(rev {config['describe'] or config['rev']}, {len(config['params'])} params)")
    for w in config.get("warnings", []):
        print("WARNING:", w)
    for e in config["preprocessor_errors"]:
        print("WARNING: config", e)
    if config["missing"]:
        print("Not found at this rev:", ", ".join(config["missing"]))

    if args.scorecard:
        rows = scorecard(bursts, fast_rows, config)
        write_rows(rows, stem + "_scorecard.csv")
        print()
        print_scorecard(rows)
        print()
        for t, i, c in events:
            print(f"  event {t / 1000.0:8.2f}s {i} {c}")
        print("Scorecard written to:", stem + "_scorecard.csv")


if __name__ == "__main__":
    main()
