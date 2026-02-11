import serial
import time

# Open serial port
ser = serial.Serial('COM10', 115200, timeout=1)
time.sleep(0.5)  # Wait for connection to establish

# Send command to enable IMU streaming (correct format: module.action.target args)
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
