import csv
import os
from tkinter import Tk, filedialog

def u16(lo, hi):
    return lo | (hi << 8)

# ----------------------------
# CAN DECODERS
# Matches the CAN matrices in Core/Src/fdcan_AL.c:
#   0x705-0x6F2  vehicle matrix   (VH_CAN_ID, standard ID)
#   0x401-0x410  debug/tuning matrix (DEBUG_CAN_ID, extended ID)
# ----------------------------

# errors_nums enum, Core/Inc/app_types.h — plain sequential C enum (NO_ERROR=0).
# Feeds 0x403 byte6 ("err") directly; this is a single error CODE, not a
# bitfield, unlike the error_c1/c2/c3 bitfields decoded below for 0x706/0x708.
_ERR_ENUM = {
    0:  "NO_ERROR",
    1:  "CURR_SENS_ERROR",
    2:  "BUS_VOLTAGE_OV_ERROR",
    3:  "BUS_VOLTAGE_UV_ERROR",
    4:  "AUX_VOLTAGE_OV_ERROR",
    5:  "AUX_VOLTAGE_UV_ERROR",
    6:  "ID_IQ_OC_ERROR",
    7:  "PHASE_CURR_ERROR",
    8:  "MTR_TEMP_OT_ERROR",
    9:  "MTC_TEMP_OT_ERROR",
    10: "MTR_TEMP_DISCONNECTION_ERROR",
    11: "MTC_TEMP_DISCONNECTION_ERROR",
    12: "EEPROM_WRITE_ERROR",
    13: "EEPROM_READ_ERROR",
    14: "ENCODER_A_B_ERROR",
    15: "ENCODER_Z_ERROR",
    16: "ENCODER_PWM_ERROR",
    17: "ENCODER_5V_ERROR",
    18: "STALL_PROTECTION",
    19: "ROTOR_MOTION_AT_INIT_ERROR",   # fault-handling branches only
    20: "OVERSPEED_ERROR",
}


def _bit(byte, n):
    return (byte >> n) & 1


# 0x706 byte3 ("error_c" = er.error_c1, Core/Inc/main.h). Bit positions
# confirmed against where each is actually set:
#   bit0 Limit_Errors? no -> Sensor_Disconnection_Checks.c (encoder Z/A-B/5V)
#   bit1 Limit_Errors.c Overspeed_Error() (profiles with OVERSPEED_TRIP_RPM)
#   bit2 Limit_Errors.c Stall_Protection()
#   bit3 Limit_Errors.c Mtr_OT_Error()
#   bit4/5 Limit_Errors.c / Sensor_Disconnection_Checks.c Bus_Voltage_Error() OV/UV
# bit6/7 are in the vehicle CAN-matrix spec but nothing in this firmware
# sets them yet — flagged "_Spec" so a decoded trace can't be mistaken for a
# confirmed-live fault.
def _decode_706_errors(byte):
    return {
        "Encoder_Fault":          _bit(byte, 0),
        "Motor_Overspeed":        _bit(byte, 1),
        "Motor_Overload_Stall":   _bit(byte, 2),
        "Motor_Overheat":         _bit(byte, 3),
        "SW_Overvoltage":         _bit(byte, 4),
        "SW_Undervoltage":        _bit(byte, 5),
        "Temp_Low_Spec":          _bit(byte, 6),
        "Speed_Limit_Spec":       _bit(byte, 7),
    }


# 0x708 byte0 ("error_c_1" = er.error_c2). bit1/3/4 confirmed in firmware
# (temp-sensor disconnect, phase SW-OC, Id/Iq OC — the matrix image labels
# bit4 "Hardware over Current" but Limit_Errors.c's Over_Current_Id_Iq_Error()
# is a software Id/Iq-limit check, not a HW comparator trip; kept the spec
# name since it's the same bit). bit0/2/5/6/7 are spec-only, unset by any
# source file today.
def _decode_708_error_c1(byte):
    return {
        "Spare_Spec":                      _bit(byte, 0),
        "Mtr_Temp_Sensor_Disconnected":    _bit(byte, 1),
        "Stall_Protection_Spec":           _bit(byte, 2),
        "SW_Overcurrent_Phase":            _bit(byte, 3),
        "HW_Overcurrent_IdIq":             _bit(byte, 4),
        "Motor_Output_Short_GND_Spec":     _bit(byte, 5),
        "Motor_Output_Loss_Of_Phase_Spec": _bit(byte, 6),
        "Gear_Direction_Reverse_Spec":     _bit(byte, 7),
    }


# 0x708 byte1 ("error_c_2" = er.error_c3, with Ctlr/Motor derate ORed in
# directly by Send_on_CAN_708() in fdcan_AL.c). bit1/2 confirmed (Aux UV,
# EEPROM read/CRC — see main.c:248 and Sensor_Disconnection_Checks.c:375).
# bit0 "Initial Angle Fault" is named in the vehicle CAN-matrix image but the
# scan was cropped before confirming which byte it lives in — treat as
# unconfirmed until checked against the full matrix doc.
def _decode_708_error_c2(byte):
    return {
        "Initial_Angle_Fault_Spec": _bit(byte, 0),
        "Aux_Voltage_UV":           _bit(byte, 1),
        "EEPROM_Read_CRC_Error":    _bit(byte, 2),
    }

def decode_401(d):
    return {
        "RPM_Ref": u16(d[0], d[1]),
        "RPM_Fdbk": u16(d[2], d[3]),
        "Id": u16(d[4], d[5]) - 512,
        "Iq": u16(d[6], d[7]) - 512
    }

def decode_402(d):
    return {
        "Iq_ref": u16(d[0], d[1]) - 512,
        "Id_ref": u16(d[2], d[3]) - 512,
        "Vd": d[4] - 159,
        "Vq": d[5] - 159,
        "Encoder_Count": u16(d[6], d[7])
    }

def decode_403(d):
    return {
        "Bus_Voltage": u16(d[0], d[1]) / 100,
        "Aux_Voltage": u16(d[2], d[3]) / 100,
        "Motor_Temp": d[4] - 50,
        "Controller_Temp": d[5] - 50,
        "Error_Code": d[6],
        "Error_Name": _ERR_ENUM.get(d[6], f"UNKNOWN({d[6]})"),
        "Actual_Torque": d[7] - 80
    }

def decode_404(d):
    return {
        "Ld": u16(d[0], d[1]) / 1.0E8,
        "Lq": u16(d[2], d[3]) / 1.0E7,
        "Lambda": u16(d[4], d[5]) / 1.0E5,
        "Rs": u16(d[6], d[7]) / 1.0E6
    }

def decode_405(d):
    return {
        "Throttle_State": d[0],
        "State": d[1],
        "Overrun_Flag": d[2],
        "PW_State": d[3],
        "Offset_Angle_Avg": u16(d[4], d[5]) / 1000,
        "Throttle_Voltage": u16(d[6], d[7]) / 1000
    }

# 0x406 bytes 0-1: Kfw x1000 normally; on FOC_ISENSE_DELAY_COMP builds (TW-7
# bench) the current-sense delay tau in us. Both keys are emitted.
def decode_406(d):
    return {
        "Kfw": u16(d[0], d[1]) / 1000,
        "Isense_Delay_us": u16(d[0], d[1]),
        "Kaw": u16(d[2], d[3]) / 1000,
        "V_s": u16(d[4], d[5]) / 1000,
        "V_max": u16(d[6], d[7]) / 1000
    }

def decode_407(d):
    return {
        "Speed_Kp": u16(d[0], d[1]) / 100,
        "Speed_Ki": u16(d[2], d[3]) / 100,
        "Speed_Kd": u16(d[4], d[5]) / 1.0E5,
        "M2F_Factor": d[6] / 100,
        "F2M_Factor": d[7] / 100
    }

def decode_408(d):
    return {
        "Flux_Kp": u16(d[0], d[1]) / 100,
        "Flux_Ki": u16(d[2], d[3]) / 100,
        "Flux_Kd": u16(d[4], d[5]) / 1.0E5,
        "FW_Int_Reset_Factor": d[6] / 100,
        "Speed_1_Torque_0": d[7]
    }

def decode_409(d):
    return {
        "Torque_Kp": u16(d[0], d[1]) / 100,
        "Torque_Ki": u16(d[2], d[3]) / 100,
        "Torque_Kd": u16(d[4], d[5]) / 1.0E5,
        "V_s": d[6],
        "V_max": d[7]
    }

def decode_410(d):
    return {
        "Config_Ver_Major": d[0],
        "Config_Ver_Minor": d[1],
        "Config_Ver_Subminor": d[2],
        "FW_Ver_Major": d[3],
        "FW_Ver_Minor": d[4],
        "FW_Ver_Subminor": d[5],
        "MTPA_FWC_Condition": d[6],
        "ISR_Time_us_LowByte": d[7]
    }

def decode_411(d):
    return {
        "Vd(V)": d[0] - 96,
        "Vq(V)": d[1] - 96,
        "Vd_PID(V)": d[2] - 96,
        "Vq_PID(V)": d[3] - 96,
        "Vd_FF(V)": d[4] - 96,
        "Vq_FF(V)": d[5] - 96,
        "P_lim": u16(d[6], d[7])
    }

def decode_705(d):
    return {
        "Motor_Temp": d[0] - 50,
        "Controller_Temp": d[1] - 50,
        "Iq": u16(d[2], d[3]) - 512,
        "Id": u16(d[4], d[5]) - 512,
        "Offset_Angle_CW": u16(d[6], d[7]) / 100
    }

_CAN706_STATE = {
    0x01: "NEUTRAL",
    0x02: "FWD_ECO",
    0x04: "REVERSE",
    0x08: "FWD_SPORTS"
}

def decode_706(d):
    flags = d[1]
    row = {
        "Drive_State": _CAN706_STATE.get(d[0], f"UNKNOWN(0x{d[0]:02X})"),
        "Regen_Active": (flags >> 2) & 1,
        "Low_Vdc": (flags >> 3) & 1,
        "Mtc_OverTemp": (flags >> 4) & 1,
        "Drive_On": (flags >> 5) & 1,
        "HillHold_Active": d[2],
        "Error_Code": d[3],
        "Speed_KMPH": u16(d[4], d[5]) / 10,
        "Offset_Angle_CCW": u16(d[6], d[7]) / 100
    }
    row.update(_decode_706_errors(d[3]))
    return row

def decode_708(d):
    row = {
        "Error_Code_1": d[0],
        "Error_Code_2": d[1],
        "Ctlr_Derate": (d[1] >> 3) & 1,
        "Motor_Derate": (d[1] >> 4) & 1,
        "Theta_elec": u16(d[2], d[3]) / 100,
        "Encoder_Count": u16(d[4], d[5]),
        "Still_Dev_Counts": d[6]
    }
    row.update(_decode_708_error_c1(d[0]))
    row.update(_decode_708_error_c2(d[1]))
    return row

def decode_709(d):
    return {
        "FOC_Loop_RefTrq": d[0],
        "torque_req": d[1], 
        "max_ph": u16(d[2], d[3]),
        "Angle_Cal_Count": d[6],
        "Angle_Cal_Stage": d[7]
    }

_HILLHOLD_STATE = {
    0: "INACTIVE",
    1: "ACTIVE",
    2: "RELEASING",
    3: "COOLDOWN",
    4: "STANDBY"
}

def decode_710(d):
    return {
        "Throttle_percent": d[0],
        "Brake_Command_Reply": d[2],
        "Throttle_voltage": d[3] / 10,
        "HillHold_Active": d[4],
        "HillHold_State": _HILLHOLD_STATE.get(d[5], d[5]),
        "HillHold_Torque_Nm": u16(d[6], d[7]) / 10
    }

# 0x715 bytes 0-1 (fdcan_AL.c Send_on_CAN_715): commanded |Id| at 2 A/bit,
# and the PEAK |Vd,Vq|/Vmax_SVM since the previous frame (x100, read-and-
# clear). Mod_Index_Peak > 1.0 means the commanded vector left the SVM circle
# and the per-phase clamp was cutting it — the real voltage-headroom signal.
# Byte 3 depends on the build: pack voltage normally, but with
# FOC_VDQ_CIRCLE_LIMIT (TW-7 bench) it is the circle-limit clip ratio, % of
# FOC steps in the window where the limit acted, and Mod_Index_Peak is the
# DEMANDED (pre-limit) index. Both keys are emitted; read the one that
# matches the firmware.
def decode_715(d):
    return {
        "Id_cmd": -2 * d[0],
        "Mod_Index_Peak": d[1] / 100,
        "MTPA_FWC_Condition": d[2],
        "Pack_voltage": d[3],
        "VLim_Clip_pct": d[3],
        "Bus_voltage": d[4],
        "LV_12V_voltage": d[5],
        "RPM": u16(d[6], d[7])
    }

# 0x716 byte4 is V_s / d.Vdc: the model's PRE-field-weakening voltage estimate
# (from the MTPA references) over the raw sensed bus. It is NOT the modulation
# index — use Mod_Index_Peak from 0x715 for that.
def decode_716(d):
    return {
        "Pack_current": u16(d[2], d[3]) - 500,
        "Vs_MTPA_over_Vdc": d[4] / 100,
        "Vrms": d[5],
        "Irms": u16(d[6], d[7])
    }

def decode_717(d):
    return {
        "Electrical_Hz": u16(d[0], d[1])
    }

def decode_724(d):
    return {
        "Torque_ref": u16(d[0], d[1]) / 10,
        "Vq": u16(d[2], d[3]) - 62,
        "Vd": u16(d[4], d[5]) - 62,
        "App_ID": d[6],
        "State": d[7]
    }

def decode_726(d):
    return {
        "Config_Ver_Major": d[0],
        "Config_Ver_Minor": d[1],
        "Config_Ver_Subminor": d[2],
        "Board_ID_Byte3": d[3],
        "Board_ID_Byte4": d[4],
        "FW_Ver_Major": d[5],
        "FW_Ver_Minor": d[6],
        "FW_Ver_Subminor": d[7]
    }

def decode_72A(d):
    return {
        "Autotune_Status": d[0],
        "Offset_CW_x1000": u16(d[1], d[2]),
        "Offset_CCW_x1000": u16(d[3], d[4]),
        "Device_ID": d[5],
        "Angle_Cal_Count": d[6],
        "Cal_Stage": d[7]
    }

def decode_7A1(d):
    return {
        "Device_ID": d[0],
        "FW_Ver_Major": d[1],
        "FW_Ver_Minor": d[2],
        "FW_Ver_Subminor": d[3],
        "Debug_Enabled": d[4],
        "Config_Ver_Major": d[5],
        "App_ID": d[7]
    }

def decode_7A2(d):
    return {
        "Device_ID": d[0],
        "HW_Ver_Major": d[1],
        "HW_Ver_Minor": d[2],
        "HW_Ver_Subminor": d[3]
    }

def decode_7A3(d):
    return {
        "Device_ID": d[0],
        "Config_Ver_Major": d[1],
        "Config_Ver_Minor": d[2],
        "Config_Ver_Subminor": d[3]
    }

def decode_7B0(d):
    git_sha = d[0] | (d[1] << 8) | (d[2] << 16) | (d[3] << 24)
    return {
        "Git_SHA": f"{git_sha:08X}",
        "Device_ID": d[4]
    }

def decode_6F0(d):
    return {
        "Device_ID": d[0],
        "HW_Ver_Major": d[1],
        "HW_Ver_Minor": d[2],
        "HW_Ver_Subminor": d[3],
        "FW_Ver_Major": d[4],
        "FW_Ver_Minor": d[5],
        "FW_Ver_Subminor": d[6],
        "UID_0": d[7]
    }

def decode_6F1(d):
    return {
        "Device_ID": d[0],
        "UID_1": d[1],
        "UID_2": d[2],
        "UID_3": d[3],
        "UID_4": d[4],
        "UID_5": d[5],
        "UID_6": d[6],
        "UID_7": d[7]
    }

def decode_6F2(d):
    return {
        "Device_ID": d[0],
        "UID_8": d[1],
        "UID_9": d[2],
        "UID_10": d[3],
        "UID_11": d[4]
    }

DECODERS = {
    0x401: decode_401,
    0x402: decode_402,
    0x403: decode_403,
    0x404: decode_404,
    0x405: decode_405,
    0x406: decode_406,
    0x407: decode_407,
    0x408: decode_408,
    0x409: decode_409,
    0x410: decode_410,  
    0x411: decode_411,
    0x705: decode_705,
    0x706: decode_706,
    0x708: decode_708,
    0x709: decode_709,
    0x710: decode_710,
    0x715: decode_715,
    0x716: decode_716,
    0x717: decode_717,
    0x724: decode_724,
    0x726: decode_726,
    0x72A: decode_72A,
    0x7A1: decode_7A1,
    0x7A2: decode_7A2,
    0x7A3: decode_7A3,
    0x7B0: decode_7B0,
    0x6F0: decode_6F0,
    # 0x6F1: decode_6F1,
    # 0x6F2: decode_6F2
}

# ----------------------------
# TRC PARSER
# ----------------------------

# PCAN-View writes two different row layouts, chosen by the save dialog:
#   $FILEVERSION=1.1:  "1)  0.4  Rx  00000403  8  AC 26 ..."
#                      number) offset Rx/Tx id dlc data
#   $FILEVERSION=2.x:  "1  0.400  DT  00000403  Rx  8  AC 26 ..."
#                      laid out by the ;$COLUMNS= header (N,O,T,I,d,l,D here):
#                      an extra Rx/Tx column sits before the length.
# Reading 2.x rows with the 1.1 indices takes the length byte as data[0] and
# shifts every byte by one — the "random values" symptom.
_TRC_COL_FIELDS = {
    "N": "number", "O": "offset", "T": "type", "B": "bus", "I": "id",
    "d": "dir", "R": "reserved", "L": "dlc", "l": "len", "D": "data",
}


def _trc_layout(filename):
    """Return (version, column list) from the TRC header."""
    version = "1.1"
    columns = None
    with open(filename) as f:
        for line in f:
            if not line.startswith(";"):
                break
            if line.startswith(";$FILEVERSION="):
                version = line.split("=", 1)[1].strip()
            elif line.startswith(";$COLUMNS="):
                columns = [c.strip() for c in line.split("=", 1)[1].split(",")]
    return version, columns


def _trc_row_v1(p):
    # number) offset Rx/Tx id dlc data
    timestamp = float(p[1])
    can_id = int(p[3], 16)
    dlc = int(p[4])
    data = [int(x, 16) for x in p[5:5 + dlc]]
    return timestamp, can_id, data


def _trc_row_v2(p, columns):
    f = {}
    for i, col in enumerate(columns):
        if col == "D":
            f["data"] = p[i:]
            break
        f[_TRC_COL_FIELDS.get(col, col)] = p[i]
    # Only data frames: DT (CAN) / FD (CAN FD). Skip status, error, RTR rows.
    if f.get("type", "DT") not in ("DT", "FD", "FB"):
        raise ValueError("not a data frame")
    timestamp = float(f["offset"])
    can_id = int(f["id"], 16)
    dlc = int(f.get("len", f.get("dlc", len(f["data"]))))
    data = [int(x, 16) for x in f["data"][:dlc]]
    return timestamp, can_id, data


def parse_trc(filename):

    current_signals = {}
    rows = []

    version, columns = _trc_layout(filename)
    v2 = version.startswith("2")
    if v2 and not columns:
        columns = ["N", "O", "T", "I", "d", "l", "D"]   # PCAN-View 2.0 default

    with open(filename) as f:

        for line in f:

            if line.startswith(";") or line.strip()=="":
                continue

            p = line.split()

            try:
                if v2:
                    timestamp, can_id, data = _trc_row_v2(p, columns)
                else:
                    timestamp, can_id, data = _trc_row_v1(p)
            except (ValueError, IndexError, KeyError):
                continue

            if can_id in DECODERS:

                decoded = DECODERS[can_id](data)

                current_signals.update(decoded)

                row = {"timestamp":timestamp}
                row.update(current_signals)

                rows.append(row)

    return rows


# ----------------------------
# CSV WRITER
# ----------------------------

def write_csv(rows, filename):

    keys=set()

    for r in rows:
        keys.update(r.keys())

    keys = ["timestamp"] + sorted(k for k in keys if k!="timestamp")

    with open(filename,"w",newline="") as f:

        writer = csv.DictWriter(f,keys)
        writer.writeheader()

        for r in rows:
            writer.writerow(r)


# ----------------------------
# MAIN
# ----------------------------

def main():

    Tk().withdraw()   # hide root window

    file_path = filedialog.askopenfilename(
        title="Select TRC log file",
        filetypes=[("TRC files", "*.trc")]
    )

    if not file_path:
        print("No file selected.")
        return

    output_file = os.path.splitext(file_path)[0] + "_decoded.csv"

    rows = parse_trc(file_path)
    write_csv(rows, output_file)

    print("Decoded CSV written to:", output_file)


if __name__ == "__main__":
    main()
