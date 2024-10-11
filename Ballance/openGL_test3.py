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

# Initialize I2C and MPU6050
i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((800, 600), DOUBLEBUF | OPENGL)
pygame.display.set_caption('MPU6050 Orientation')

# Variables to store orientation
pitch = roll = yaw = 0.0

def resize(width, height):
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0 * width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

def draw_cube():
    glBegin(GL_QUADS)
    scale_x = 2.0  # Scale factor along X axis
    scale_y = 1.0  # Scale factor along Y axis
    scale_z = 0.5  # Scale factor along Z axis
    
    glBegin(GL_QUADS)
    # Front face
    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(-scale_x, -scale_y, scale_z)
    glVertex3f(scale_x, -scale_y, scale_z)
    glVertex3f(scale_x, scale_y, scale_z)
    glVertex3f(-scale_x, scale_y, scale_z)
    
    # Back face
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(-scale_x, -scale_y, -scale_z)
    glVertex3f(-scale_x, scale_y, -scale_z)
    glVertex3f(scale_x, scale_y, -scale_z)
    glVertex3f(scale_x, -scale_y, -scale_z)
    
    # Top face
    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-scale_x, scale_y, -scale_z)
    glVertex3f(-scale_x, scale_y, scale_z)
    glVertex3f(scale_x, scale_y, scale_z)
    glVertex3f(scale_x, scale_y, -scale_z)
    
    # Bottom face
    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(-scale_x, -scale_y, -scale_z)
    glVertex3f(scale_x, -scale_y, -scale_z)
    glVertex3f(scale_x, -scale_y, scale_z)
    glVertex3f(-scale_x, -scale_y, scale_z)
    
    # Right face
    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(scale_x, -scale_y, -scale_z)
    glVertex3f(scale_x, scale_y, -scale_z)
    glVertex3f(scale_x, scale_y, scale_z)
    glVertex3f(scale_x, -scale_y, scale_z)
    
    # Left face
    glColor3f(0.0, 1.0, 1.0)
    glVertex3f(-scale_x, -scale_y, -scale_z)
    glVertex3f(-scale_x, -scale_y, scale_z)
    glVertex3f(-scale_x, scale_y, scale_z)
    glVertex3f(-scale_x, scale_y, -scale_z)
    glEnd()

def get_orientation():
    global pitch, roll, yaw
    accel_data = mpu.acceleration
    gyro_data = mpu.gyro

    # Calculate pitch and roll from accelerometer data
    accel_x, accel_y, accel_z = accel_data
    pitch = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180 / math.pi
    roll = math.atan2(-accel_x, accel_z) * 180 / math.pi

    # Integrate gyroscope data to get yaw
    gyro_x, gyro_y, gyro_z = gyro_data
    yaw += gyro_z * 0.01  # Assuming a loop time of 10ms

def main():
    resize(800, 600)
    init()

    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                return

        get_orientation()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -5.0)
        glRotatef(pitch, 1.0, 0.0, 0.0)
        glRotatef(roll, 0.0, 1.0, 0.0)
        glRotatef(yaw, 0.0, 0.0, 1.0)
        draw_cube()
        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    main()
