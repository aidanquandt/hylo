import argparse
import serial
import time

parser = argparse.ArgumentParser(description="Raw serial monitor")
parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (e.g. /dev/ttyUSB0, COM10)")
parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
args = parser.parse_args()

ser = serial.Serial(args.port, args.baud, timeout=1)
print(f"Monitoring {args.port} at {args.baud} baud...")
print("Press Ctrl+C to stop\n")

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(line)
        time.sleep(0.01)
except KeyboardInterrupt:
    print("\nStopped monitoring")
finally:
    ser.close()
