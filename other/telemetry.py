import serial
import struct
import time

# ------------------------ MSP helpers ------------------------

def make_msp(command, payload=b""):
    """Build MSP v1 frame."""
    if isinstance(payload, list):
        payload = bytes(payload)
    length = len(payload)
    checksum = 0
    frame = b"$M<" + bytes([length]) + bytes([command]) + payload
    checksum ^= length
    checksum ^= command
    for b_ in payload:
        checksum ^= b_
    frame += bytes([checksum])
    return frame

def read_msp_response(ser):
    """Read one MSP response frame (v1). Returns (cmd, data) or (None, None)."""
    start = ser.read_until(b"$M>")
    if not start.endswith(b"$M>"):
        return None, None
    lb = ser.read(1)
    if not lb:
        return None, None
    length = lb[0]
    cmd_b = ser.read(1)
    if not cmd_b:
        return None, None
    cmd = cmd_b[0]
    data = ser.read(length)
    _ = ser.read(1)  # checksum – skipping validation for brevity
    return cmd, data

# ---------------------- Parsers (robust) ----------------------

def parse_status(data: bytes):
    """
    Robust parser for MSP_STATUS (101).
    Known layouts:
      - 12 bytes: <IHHI>  -> cycleTime, i2cErr, sensors, flags
      - 24 bytes: <IHHI III> -> base(12) + 3x uint32 extended flags (e.g., armingDisableFlags, flightModeFlags, extra)
      - 26 bytes: <IHHI III H> (rare) -> same + trailing H
    We DO NOT assume profile indices here to avoid mislabeling.
    """
    l = len(data)
    result = {}
    if l >= 12:
        cycleTime, i2cErr, sensors, flags = struct.unpack("<IHHI", data[:12])
        result.update({
            "cycleTime": cycleTime,
            "i2cError": i2cErr,
            "sensorsMask": sensors,
            "flags": flags
        })
    if l == 24:
        ext1, ext2, ext3 = struct.unpack("<III", data[12:24])
        result.update({
            "ext1": ext1,           # often armingDisableFlags (bitfield)
            "ext2": ext2,           # often flightModeFlags (bitfield)
            "ext3": ext3            # extra/implementation-defined
        })
    elif l == 26:
        ext1, ext2, ext3, tail = struct.unpack("<IIIH", data[12:26])
        result.update({"ext1": ext1, "ext2": ext2, "ext3": ext3, "tail": tail})
    elif l not in (12, 24, 26):
        result["raw"] = data.hex()
        result["len"] = l
    return result

def parse_analog(data: bytes):
    """MSP_ANALOG (110) – returns VBAT/Amperage/mAh if available."""
    out = {}
    if len(data) >= 8:
        vbat, mahdrawn, rssi, amperage = struct.unpack("<HHHH", data[:8])
        out["vbat_V"] = vbat / 10.0
        out["amperage_A"] = amperage / 100.0
        out["mah"] = mahdrawn
    else:
        out["raw"] = data.hex()
        out["len"] = len(data)
    return out

# ---------------------- Optional decoders ----------------------

def is_armed(flags: int) -> bool:
    """In classic BF, bit0 of flags indicates armed."""
    return bool(flags & 1)

def bits_to_hex(v):
    return f"0x{v:08X}"

# -------------------------- Main loop --------------------------

if __name__ == "__main__":
    # Update port as needed
    ser = serial.Serial("COM19", 115200, timeout=1)

    # Query API & FC info (nice to have)
    ser.write(make_msp(1))  # MSP_API_VERSION
    cmd, data = read_msp_response(ser)
    if cmd == 1 and data and len(data) >= 3:
        api_major, api_minor, api_patch = struct.unpack("<BBB", data[:3])
        print(f"API version: {api_major}.{api_minor}.{api_patch}")

    ser.write(make_msp(106))  # MSP_FC_VARIANT
    cmd, data = read_msp_response(ser)
    if cmd == 106 and data and len(data) >= 4:
        try:
            variant = data[:4].decode("ascii", errors="ignore")
        except Exception:
            variant = data[:4]
        print(f"FC variant: {variant}")

    ser.write(make_msp(107))  # MSP_FC_VERSION
    cmd, data = read_msp_response(ser)
    if cmd == 107 and data and len(data) >= 3:
        vmaj, vmin, vpatch = struct.unpack("<BBB", data[:3])
        print(f"FC version: {vmaj}.{vmin}.{vpatch}")

    print("-" * 60)

    while True:
        # --- IMU ---
        ser.write(make_msp(102))  # MSP_RAW_IMU
        cmd, data = read_msp_response(ser)
        if cmd == 102 and data and len(data) >= 18:
            try:
                ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack("<hhhhhhhhh", data[:18])
                print(f"Acc: {ax:6d} {ay:6d} {az:6d} | Gyro: {gx:6d} {gy:6d} {gz:6d}")
            except struct.error:
                print(f"IMU parse error (len={len(data)}): {data.hex()}")

        # --- STATUS (robust) ---
        ser.write(make_msp(101))  # MSP_STATUS
        cmd, data = read_msp_response(ser)
        if cmd == 101 and data:
            st = parse_status(data)
            if "cycleTime" in st:
                armed = is_armed(st["flags"])
                line = (f"Armed: {armed}, CycleTime: {st['cycleTime']}, "
                        f"i2cErr: {st['i2cError']}, SensorsMask: {st['sensorsMask']}")
                # Extended flags if present
                if "ext1" in st:
                    line += (f", ext1: {bits_to_hex(st['ext1'])}, "
                             f"ext2: {bits_to_hex(st['ext2'])}, ext3: {bits_to_hex(st['ext3'])}")
                if "tail" in st:
                    line += f", tail: 0x{st['tail']:04X}"
                print(line)
            else:
                print(f"Unknown STATUS len={st.get('len')} raw={st.get('raw')}")

        # --- ANALOG (VBAT/current) ---
        ser.write(make_msp(110))
        cmd, data = read_msp_response(ser)
        if cmd == 110 and data:
            an = parse_analog(data)
            if "vbat_V" in an:
                print(f"VBAT: {an['vbat_V']:.1f} V | Amperage: {an['amperage_A']:.2f} A | mAh: {an['mah']}")
            else:
                print(f"Analog raw len={an['len']} hex={an['raw']}")

        # (Optional) Uncomment to see motors/servos:
        # ser.write(make_msp(104))  # MSP_MOTOR
        # cmd, data = read_msp_response(ser)
        # if cmd == 104 and data:
        #     motors = struct.unpack("<" + "H"*(len(data)//2), data)
        #     print("Motors:", motors)
        #
        # ser.write(make_msp(103))  # MSP_SERVO
        # cmd, data = read_msp_response(ser)
        # if cmd == 103 and data:
        #     servos = struct.unpack("<" + "H"*(len(data)//2), data)
        #     print("Servos:", servos)

        print("-" * 60)
        time.sleep(1)
