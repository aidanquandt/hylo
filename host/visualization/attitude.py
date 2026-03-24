import argparse
import pygame
import math
from math import atan2, asin, pi, fmod, degrees, sin, cos, acos, sqrt
import serial  # Import serial library for UART communication (pyserial)
import sys
import re
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *

class Display:
    useQuat = False  # using Euler angles computed from accel/gyro

    def __init__(self, port="/dev/ttyUSB0", baud=115200):
        video_flags = OPENGL | DOUBLEBUF
        pygame.init()
        screen = pygame.display.set_mode((640, 480), video_flags)
        pygame.display.set_caption("Hylo IMU Visualization")
        self.resizewin(640, 480)
        glShadeModel(GL_SMOOTH)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
        
        # Open the serial port for receiving IMU data
        self.ser = serial.Serial(port, baud, timeout=1)
        
        # IMU data storage
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.temp = 0.0

    def resizewin(self, width, height):
        if height == 0:
            height = 1
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, 1.0*width/height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def draw(self):
        # Read IMU data from UART
        data = self.ser.readline()  # Read a line from UART
        if data:
            try:
                # Parse the IMU data format: [counter] A: ax ay az | G: gx gy gz | T: temp
                line = data.decode('utf-8').strip()
                
                # Use regex to extract the values
                # Pattern: [number] A: float float float | G: float float float | T: float
                pattern = r'\[\d+\]\s+A:\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s+G:\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s+T:\s+([-\d.]+)'
                match = re.match(pattern, line)
                
                if match:
                    ax, ay, az = map(float, match.groups()[0:3])
                    gx, gy, gz = map(float, match.groups()[3:6])
                    temp = float(match.groups()[6])
                    
                    # Compute orientation from accelerometer (pitch and roll)
                    self.pitch = math.atan2(ax, math.sqrt(ay*ay + az*az)) * 180.0 / math.pi
                    self.roll = math.atan2(ay, az) * 180.0 / math.pi
                    # Yaw can't be determined from accel alone, use gyro rate instead
                    self.yaw += gz * 0.1  # Approximate integration (10Hz update)
                    self.temp = temp
                    
                    self.visualize_orientation()
                else:
                    # If line doesn't match, it might be a status message - just skip it
                    pass
                    
            except ValueError as e:
                print(f"Error parsing data: {e}")
            except Exception as e:
                print(f"Unexpected error: {e}")
                
    def visualize_orientation(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0, 0.0, -7.0)

        self.drawText((-2.6, 1.8, 2), "Hylo IMU Visualization", 18)
        self.drawText((-2.6, 1.6, 2), "Real-time orientation from accelerometer/gyroscope", 14)
        self.drawText((-2.6, -1.8, 2), "Roll: %.1f, Pitch: %.1f, Yaw: %.1f" % (self.roll, self.pitch, self.yaw), 16)
        self.drawText((-2.6, -2.0, 2), "Temperature: %.1f C" % self.temp, 16)

        # Apply rotations in the order: yaw, pitch, roll
        glRotatef(-self.roll, 0.00, 0.00, 1.00)
        glRotatef(self.pitch, 1.00, 0.00, 0.00)
        glRotatef(self.yaw, 0.00, 1.00, 0.00)

        # Draw the cube representing the IMU orientation
        glBegin(GL_QUADS)
        
        # Top face (green)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(1.0, 0.2, 1.0)
        
        # Bottom face (red)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(1.0, -0.2, -1.0)
        
        # Front face (blue)
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)
        
        # Back face (yellow)
        glColor3f(1.0, 1.0, 0.0)
        glVertex3f(1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, -1.0)
        
        # Right face (magenta)
        glColor3f(1.0, 0.0, 1.0)
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, -1.0)
        
        # Left face (cyan)
        glColor3f(0.0, 1.0, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        
        glEnd()

        pygame.display.flip()

    def drawText(self, position, textString, size):
        font = pygame.font.SysFont("Courier", size, True)
        textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
        textData = pygame.image.tostring(textSurface, "RGBA", True)
        glRasterPos3d(*position)
        glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

# Main loop
if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Hylo IMU 3D attitude visualization")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (e.g. /dev/ttyUSB0, COM10)")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = ap.parse_args()
    display = Display(port=args.port, baud=args.baud)
    print("Hylo IMU Visualization Started")
    print(f"Waiting for IMU data on {args.port}...")
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                display.ser.close()
                pygame.quit()
                exit()
        display.draw()
