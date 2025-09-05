import serial
import struct
import time

# פונקציה ליצירת פריימים של MSP
def make_msp(command, payload=[]):
    length = len(payload)
    checksum = 0
    frame = b"$M<" + bytes([length]) + bytes([command]) + bytes(payload)
    checksum ^= length
    checksum ^= command
    for b in payload:
        checksum ^= b
    frame += bytes([checksum])
    return frame

# פונקציה לקריאת תשובה
def read_msp_response(ser):
    start = ser.read_until(b"$M>")
    if not start.endswith(b"$M>"):
        return None, None

    length_bytes = ser.read(1)
    if not length_bytes:
        return None, None
    length = length_bytes[0]

    cmd = ser.read(1)[0]
    data = ser.read(length)
    checksum = ser.read(1)  # לא נבדוק כרגע
    return cmd, data

if __name__ == "__main__":
    # תעדכן ל-COM המתאים
    ser = serial.Serial("COM19", 115200, timeout=1)

    while True:
        # בקשת חיישני IMU
        ser.write(make_msp(102))  # 102 = MSP_RAW_IMU
        cmd, data = read_msp_response(ser)
        if cmd == 102 and data:
            try:
                ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack("<hhhhhhhhh", data)
                print(f"Acc: {ax:6d} {ay:6d} {az:6d} | Gyro: {gx:6d} {gy:6d} {gz:6d}")
            except struct.error:
                print("IMU data parse error")

        # בקשת סטטוס
        ser.write(make_msp(101))  # 101 = MSP_STATUS
        cmd, data = read_msp_response(ser)
        if cmd == 101 and data:
            print(f"MSP_STATUS len={len(data)} raw={data.hex()}")
            # נסה לפרש לפי אורך
            if len(data) >= 12:
                cycleTime, i2cError, sensor, flag = struct.unpack("<IHHI", data[:12])
                armed = bool(flag & 1)
                print(f"Armed: {armed}, CycleTime: {cycleTime}, i2cError: {i2cError}, SensorsMask: {sensor}")
            else:
                print("STATUS payload shorter than expected")

        print("-" * 60)
        time.sleep(1)
