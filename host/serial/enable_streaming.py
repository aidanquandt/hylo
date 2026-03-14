import argparse
import serial
import time

parser = argparse.ArgumentParser(description="Enable IMU streaming on device")
parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (e.g. /dev/ttyUSB0, COM10)")
parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
args = parser.parse_args()

ser = serial.Serial(args.port, args.baud, timeout=1)
time.sleep(0.5)  # Wait for connection to establish

# Send command to enable IMU streaming (legacy text protocol; use protocol_tool for protobuf)
command = "imu.set.stream on\r\n"
ser.write(command.encode('utf-8'))
print(f"Sent: {command.strip()}")

# Read response
time.sleep(0.2)
while ser.in_waiting > 0:
    response = ser.readline().decode('utf-8', errors='ignore').strip()
    if response:
        print(f"Response: {response}")

ser.close()
print("\nIMU streaming should now be enabled at 10Hz")
