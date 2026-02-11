import serial
import time

# Open serial port
ser = serial.Serial('COM10', 115200, timeout=1)
print("Monitoring COM10 at 115200 baud...")
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
