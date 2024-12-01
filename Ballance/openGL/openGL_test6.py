#!/usr/bin/env python

from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import board
import busio
import adafruit_mpu6050
import math
import time
from touchScreenBasicCoordOutput import read_touch_coordinates
import threading

# Initialize I2C and MPU6050
i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((1600, 1200), DOUBLEBUF | OPENGL | RESIZABLE)
pygame.display.set_caption('MPU6050 Orientation')

# Variables to store orientation
pitch = roll = yaw = 0.0
zoom_level = 3.0  # Initial zoom level

# Camera position variables
camera_x = 0.0
camera_y = 0.5
camera_z = zoom_level

points = []
current_position = (0, 0)
font = pygame.font.SysFont('arial', 24)

def resize(width, height):
    if height == 0:
        height = 1
    aspect_ratio = width / height
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, aspect_ratio, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    # Set the initial camera position and orientation
    gluLookAt(camera_x, camera_y, camera_z,  # Camera position (adjusted by zoom_level)
              0, 0, 0,    # Look at the origin
              0, 1, 0)    # Up direction

def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
    glEnable(GL_POINT_SMOOTH)
    glPointSize(5.0)

def draw_rect():
    glBegin(GL_QUADS)    
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f( 1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)        
    glVertex3f(-1.0, 0.2,  1.0)        
    glVertex3f( 1.0, 0.2,  1.0)        

    glColor3f(1.0, 0.5, 0.0)    
    glVertex3f( 1.0, -0.2,  1.0)
    glVertex3f(-1.0, -0.2,  1.0)        
    glVertex3f(-1.0, -0.2, -1.0)        
    glVertex3f( 1.0, -0.2, -1.0)        

    glColor3f(1.0, 0.0, 0.0)        
    glVertex3f( 1.0, 0.2,  1.0)
    glVertex3f(-1.0, 0.2,  1.0)        
    glVertex3f(-1.0, -0.2,  1.0)        
    glVertex3f( 1.0, -0.2,  1.0)        

    glColor3f(1.0, 1.0, 0.0)    
    glVertex3f( 1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)        
    glVertex3f( 1.0, 0.2, -1.0)        

    glColor3f(0.0, 0.0, 1.0)    
    glVertex3f(-1.0, 0.2,  1.0)
    glVertex3f(-1.0, 0.2, -1.0)        
    glVertex3f(-1.0, -0.2, -1.0)        
    glVertex3f(-1.0, -0.2,  1.0)        

    glColor3f(1.0, 0.0, 1.0)    
    glVertex3f( 1.0, 0.2, -1.0)
    glVertex3f( 1.0, 0.2,  1.0)
    glVertex3f( 1.0, -0.2,  1.0)        
    glVertex3f( 1.0, -0.2, -1.0)        
    glEnd()

def draw_circle(x, y, z, radius, num_segments):
    glBegin(GL_TRIANGLE_FAN)
    glVertex3f(x, y, z)  # Center of the circle
    for i in range(num_segments + 1):
        angle = 2.0 * math.pi * i / num_segments
        dx = radius * math.cos(angle)
        dy = radius * math.sin(angle)
        glVertex3f(x + dx, y + dy, z)
    glEnd()

def draw_points():
    current_time = time.time()
    glColor3f(0.0, 0.0, 1.0)
    for point, timestamp in points:
        if current_time - timestamp < 2:
            print(f"currently drawing point at {point[0]}, {point[1]}, {point[2]}")
            # Save the current model view matrix
            glPushMatrix()
            # Reset the model view matrix to ensure the point faces the camera
            glTranslatef(point[0], point[1], point[2])
            glRotatef(-yaw, 0.0, 1.0, 0.0)
            glRotatef(-pitch, 1.0, 0.0, 0.0)
            glRotatef(-roll, 0.0, 0.0, 1.0)
            draw_circle(0, 0, 0, radius=0.05, num_segments=7)
            # Restore the model view matrix
            glPopMatrix()

def update_points():
    global points, current_position
    for x, y in read_touch_coordinates():
        print(f"reading touch coordinates: {x}, {y}")
        print("converting to gl coordinates...")
        gl_x = ((y - 150) / (3940 - 150)) * 2 - 1
        gl_y = 0.3  
        gl_z = -(((x - 250) / (3800 - 250)) * 2 - 1)
        print(f"corresponding gl coordinates: {gl_x}, {gl_y}, {gl_z}")
        if len(points) == 100:
            points.pop(0)
        points.append(((gl_x, gl_y, gl_z), time.time()))

        current_position = (gl_x, gl_z)

def get_orientation(dt):
    global pitch, roll, yaw
    accel_data = mpu.acceleration
    gyro_data = mpu.gyro

    # Calculate pitch and roll from accelerometer data
    accel_x, accel_y, accel_z = accel_data
    accel_pitch = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180 / math.pi
    accel_roll = math.atan2(-accel_x, accel_z) * 180 / math.pi

    # Integrate gyroscope data to get yaw
    gyro_x, gyro_y, gyro_z = gyro_data
    pitch += gyro_x * dt
    roll += gyro_y * dt
    yaw += gyro_z * dt

    alpha = 0.9
    pitch = alpha * pitch + (1 - alpha) * accel_pitch
    roll = alpha * roll + (1 - alpha) * accel_roll

def draw_text(x, y, text):
    text_surface = font.render(text, True, (255, 255, 255, 255), (0, 66, 0, 255))
    text_data = pygame.image.tostring(text_surface, "RGBA", True)
    glWindowPos2d(x, y)
    glDrawPixels(text_surface.get_width(), text_surface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, text_data)

def user_input_thread():
    global camera_x, camera_y, camera_z
    while True:
        try:
            camera_x = float(input("Enter camera x position: "))
            camera_y = float(input("Enter camera y position: "))
            camera_z = float(input("Enter camera z position: "))
        except ValueError:
            print("Invalid input. Please enter numeric values.")

def main():
    global zoom_level, yaw, pitch, roll

    # Set initial window size
    initial_width, initial_height = 800, 600
    resize(initial_width, initial_height)
    init()

    threading.Thread(target=update_points, daemon=True).start()
    threading.Thread(target=user_input_thread, daemon=True).start()

    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                return
            elif event.type == VIDEORESIZE:
                # Update the OpenGL viewport and perspective on window resize
                resize(event.w, event.h)
                screen = pygame.display.set_mode((event.w, event.h), DOUBLEBUF | OPENGL | RESIZABLE)
            elif event.type == KEYDOWN:
                if event.key == K_PLUS or event.key == K_EQUALS:  # '+' key
                    zoom_level -= 0.1
                elif event.key == K_MINUS:  # '-' key
                    zoom_level += 0.1

        dt = clock.tick(60) / 1000.0
        get_orientation(dt)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        # Set the camera position and orientation
        gluLookAt(camera_x, camera_y, camera_z,  # Camera position (adjusted by user input)
                  0, 0, 0,    # Look at the origin
                  0, 1, 0)    # Up direction

        glTranslatef(0, 0, -5.0)
        glRotatef(pitch, -1, 0.0, 0.0)
        glRotatef(yaw + 3, 0.0, 1, 0.0)
        glRotatef(roll, 0.0, 0.0, 1)
        
        draw_rect()
        draw_points()

        # Display the current position at the bottom of the screen
        screen_width, screen_height = pygame.display.get_surface().get_size()
        draw_text(10, screen_height - 30, f"Current Position: {current_position}")

        pygame.display.flip()

if __name__ == "__main__":
    main()