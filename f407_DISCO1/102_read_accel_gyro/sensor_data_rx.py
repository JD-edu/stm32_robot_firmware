import serial
import struct

# 시리얼 포트 설정 (Windows: "COMx", Linux: "/dev/ttyUSBx")
ser = serial.Serial(port="COM63", baudrate=115200, timeout=1)

while True:
    data = ser.read(18)  # 10바이트 수신 (16비트 데이터 5개)

    if len(data) == 18:
        # Little Endian 형식으로 변환
        ax, ay, az, gx, gy, gz, pitch, roll, yaw = struct.unpack('<hhhhhhhhh', data)
        #print(f"Accel X: {ax}, Y: {ay}, Z: {az}, Gyro X: {gx}, Y: {gy}  z: {gz}")
        print(f"pitch: {pitch}, roll: {roll}, yaw: {yaw}")
