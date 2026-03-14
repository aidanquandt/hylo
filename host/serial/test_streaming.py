import serial
import time

# Open serial port
ser = serial.Serial('COM10', 115200, timeout=1)
print("Connected to COM10 at 115200 baud")
time.sleep(0.5)

# Clear any pending data
ser.reset_input_buffer()

# Send enable command (legacy text protocol; use protocol_tool for protobuf)
command = "imu.set.stream on\r\n"
print(f"\nSending: {command.strip()}")
ser.write(command.encode('utf-8'))
time.sleep(0.5)

# Read responses for a few seconds
print("\nReceiving data for 5 seconds...\n")
start_time = time.time()
line_count = 0

try:
    while (time.time() - start_time) < 5:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(line)
                line_count += 1
        time.sleep(0.01)
except KeyboardInterrupt:
    pass

print(f"\n\nReceived {line_count} lines")
ser.close()

if line_count == 0:
    print("\nNo data received. Possible issues:")
    print("1. Device not connected or powered")
    print("2. Wrong COM port")
    print("3. Device firmware not running")
    print("4. Check your connections")
elif line_count < 10:
    print("\nReceived acknowledgment but no streaming data.")
    print("The IMU module might not be in ACTIVE state.")
else:
    print("\nIMU streaming is working!")
