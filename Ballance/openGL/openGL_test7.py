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
current_position = (0, 0)
font = pygame.font.SysFont('arial', 24)

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
    glEnable(GL_DEPTH_TEST, GL_BLEND)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
    glEnable(GL_POINT_SMOOTH)
    glPointSize(5.0)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)


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

    #display()

def draw_points():
    current_time = time.time()
    glEnable(GL_POINT_SMOOTH)  
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST)
    glColor3f(0.0, 0.0, 1.0)
    glBegin(GL_POINTS)
    for point, timestamp in points:
        if current_time - timestamp < 2:
            print(f"currently drawing point at {point[0]}, {point[1]}, {point[2]}")
            glVertex3f(point[0], point[1], point[2])
    glEnd()

    # Draw three static points for testing
    # glBegin(GL_POINTS)
    # glVertex3f(0.0, 0.3, 0.0)  # Point 1
    # glVertex3f(0.5, 0.3, 0.0)  # Point 2
    # glVertex3f(-0.5, 0.3, 0.0) # Point 3
    # glEnd()


    #pygame.display.flip()

def update_points():
    global points, current_position
    for x, y in read_touch_coordinates():
        print(f"reading touch coordinates: {x}, {y}")
        print("converting to gl coordinates...")
        gl_x = ((y-150) / (3940 - 150)) * 2 - 1
        gl_y = 0.3
        gl_z = -(((x-250) / (3800 - 250)) * 2 - 1)
        print(f"corresponding gl coordinates: {gl_x}, {gl_y}, {gl_z}")
        #if the length of the points list is 100, remove the oldest point
        #else just append the new point
        if len(points) == 100:
            points.pop(0)
        points.append(((gl_x, gl_y, gl_z), time.time()))

        current_position = (gl_x, gl_z)

        # pygame.event.post(pygame.event.Event(pygame.USEREVENT))


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
    text_surface = font.render(text, True, (255, 255, 255, 255), (0, 0, 0, 255))
    text_data = pygame.image.tostring(text_surface, "RGBA", True)
    glWindowPos2d(x, y)
    glDrawPixels(text_surface.get_width(), text_surface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, text_data)

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
            # elif event.type == pygame.USEREVENT:
            #     display()
        
        #display()
        time.sleep(0.01)
        

        dt = clock.tick(60) / 1000.0
        get_orientation(dt)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        # prism_top_center = (0.0, 0.2, 0.0)
        # Set the camera position and orientation
        # gluLookAt(0, 0.5, 5,  # Camera position
        #           prism_top_center[0], prism_top_center[1], prism_top_center[2],  # Look at point
        #           0, 1, 0)  # Up direction

        glTranslatef(0.0, 0.0, -5.0)  
        # glRotatef(90, 1.0, 0.0, 0.0) # hard coded pitch tilt
        # glRotatef(10, -1.0, 0.0, 0.0) # hard coded pitch tilt
        glRotatef(10, 0.0, 0.0, -1.0) # hard coded roll tilt
        
        

        glRotatef(pitch, 1, 0.0, 0.0) # up down
        glRotatef(yaw, 0.0, -1, 0.0) # side to side
        glRotatef(roll, 0.0, 0.0, -1) # tilt
        draw_rect()
        draw_points()
        draw_text(-0.95, 0.9, f"Current Position: {current_position[0]}, {current_position[1]}")
        pygame.display.flip()
        #clock.tick(60)

if __name__ == "__main__":
    main()