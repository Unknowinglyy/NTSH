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

# Initialize I2C and MPU6050
i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((800, 600), DOUBLEBUF | OPENGL)
pygame.display.set_caption('MPU6050 Orientation')


# Variables to store orientation
pitch = roll = yaw = 0.0

points = []

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

def draw_rect():
    glBegin(GL_QUADS)	
    glColor3f(0.0,1.0,0.0)
    glVertex3f( 1.0, 0.2,-1.0)
    glVertex3f(-1.0, 0.2,-1.0)		
    glVertex3f(-1.0, 0.2, 1.0)		
    glVertex3f( 1.0, 0.2, 1.0)		

    glColor3f(1.0,0.5,0.0)	
    glVertex3f( 1.0,-0.2, 1.0)
    glVertex3f(-1.0,-0.2, 1.0)		
    glVertex3f(-1.0,-0.2,-1.0)		
    glVertex3f( 1.0,-0.2,-1.0)		

    glColor3f(1.0,0.0,0.0)		
    glVertex3f( 1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)		
    glVertex3f(-1.0,-0.2, 1.0)		
    glVertex3f( 1.0,-0.2, 1.0)		

    glColor3f(1.0,1.0,0.0)	
    glVertex3f( 1.0,-0.2,-1.0)
    glVertex3f(-1.0,-0.2,-1.0)
    glVertex3f(-1.0, 0.2,-1.0)		
    glVertex3f( 1.0, 0.2,-1.0)		

    glColor3f(0.0,0.0,1.0)	
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2,-1.0)		
    glVertex3f(-1.0,-0.2,-1.0)		
    glVertex3f(-1.0,-0.2, 1.0)		

    glColor3f(1.0,0.0,1.0)	
    glVertex3f( 1.0, 0.2,-1.0)
    glVertex3f( 1.0, 0.2, 1.0)
    glVertex3f( 1.0,-0.2, 1.0)		
    glVertex3f( 1.0,-0.2,-1.0)		
    glEnd()	

def display():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    draw_rect()
    current_time = time.time()
    glBegin(GL_POINTS)
    for point, timestamp in points:
        if current_time - timestamp < 2:
            glVertex3f(point[0], point[1], point[2])
    glEnd()

    #pygame.display.flip()

def update_points():
    global points
    for x,y in read_touch_coordinates():
        gl_x = ((x-250) / (3800 - 250)) * 2 - 1
        gl_y = ((y-150) / (3940 - 150)) * 0.4 - 0.2
        gl_z = 0.2

        points.append(((gl_x, gl_y, gl_z), time.time()))

        pygame.event.post(pygame.event.Event(pygame.USEREVENT))


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

def main():
    resize(800, 600)
    init()

    import threading
    threading.Thread(target=update_points, daemon=True).start()

    clock = pygame.time.Clock()

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                return
            elif event.type == VIDEORESIZE:
                resize(event.w, event.h)
            elif event.type == pygame.USEREVENT:
                display()
        
        display()
        time.sleep(0.01)
        

        dt = clock.tick(120) / 1000.0
        get_orientation(dt)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -5.0)
        glRotatef(pitch, 1.0, 0.0, 0.0)
        glRotatef(yaw, 0.0, -1.0, 0.0)
        glRotatef(roll, 0.0, 0.0, -1.0)
        draw_rect()
        pygame.display.flip()
        #clock.tick(60)

if __name__ == "__main__":
    main()