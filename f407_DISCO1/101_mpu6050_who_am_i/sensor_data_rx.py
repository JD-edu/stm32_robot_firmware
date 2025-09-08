import serial
import struct

# 시리얼 포트 설정 (Windows: "COMx", Linux: "/dev/ttyUSBx")
ser = serial.Serial(port="COM62", baudrate=9600, timeout=1)

while True:
    data = ser.read(12)  # 10바이트 수신 (16비트 데이터 5개)

    if len(data) == 12:
        # Little Endian 형식으로 변환
        ax, ay, az, gx, gy, gz = struct.unpack('<hhhhhh', data)
        print(f"Accel X: {ax}, Y: {ay}, Z: {az}, Gyro X: {gx}, Y: {gy}  z: {gz}")
