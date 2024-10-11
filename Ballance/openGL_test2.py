from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import board
import busio
import adafruit_mpu6050

i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

# Add these global variables to track the accumulated angles
pitch = 0.0
roll = 0.0
yaw = 0.0

last_time = 0

def read_data(delta_time):
    global ax, ay, az
    ax, ay, az = mpu.gyro

    # Calculate angular displacement based on angular velocity (gyro data)
    global pitch, roll, yaw

    # Update orientation angles based on gyro data (angular velocity)
    pitch += ay * delta_time
    roll += ax * delta_time
    if yaw_mode:
        yaw += az * delta_time
    else:
        yaw = 0  # Only apply yaw when yaw_mode is enabled

def main():
    global yaw_mode, last_time

    video_flags = OPENGL | DOUBLEBUF
    
    pygame.init()
    screen = pygame.display.set_mode((640,480), video_flags)
    pygame.display.set_caption("Press Esc to quit, z toggles yaw mode")
    resize(640,480)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()
    last_time = pygame.time.get_ticks()  # Initialize last_time
    
    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()  # Quit pygame properly
            break       
        if event.type == KEYDOWN and event.key == K_z:
            yaw_mode = not yaw_mode
            #ser.write(b"z")

        # Calculate the time delta since the last frame
        current_time = pygame.time.get_ticks()
        delta_time = (current_time - last_time) / 1000.0  # Convert to seconds
        last_time = current_time
        
        read_data(delta_time)
        draw()
      
        pygame.display.flip()
        frames = frames + 1

    print("fps:  %d" % ((frames * 1000) / (pygame.time.get_ticks() - ticks)))
    #ser.close()

def draw():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    osd_text = "pitch: " + str("{0:.2f}".format(pitch)) + ", roll: " + str("{0:.2f}".format(roll))

    if yaw_mode:
        osd_line = osd_text + ", yaw: " + str("{0:.2f}".format(yaw))
    else:
        osd_line = osd_text

    drawText((-2, -2, 2), osd_line)

    # Apply accumulated rotations
    if yaw_mode:  # Experimental yaw rotation
        glRotatef(yaw, 0.0, 1.0, 0.0)  # Yaw, rotate around y-axis

    glRotatef(pitch, 1.0, 0.0, 0.0)  # Pitch, rotate around x-axis
    glRotatef(-1 * roll, 0.0, 0.0, 1.0)  # Roll, rotate around z-axis

    # Draw the cube as before
    glBegin(GL_QUADS)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)        
    glVertex3f(-1.0, 0.2, 1.0)        
    glVertex3f(1.0, 0.2, 1.0)        

    glColor3f(1.0, 0.5, 0.0)    
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)        
    glVertex3f(-1.0, -0.2, -1.0)        
    glVertex3f(1.0, -0.2, -1.0)        

    glColor3f(1.0, 0.0, 0.0)        
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)        
    glVertex3f(-1.0, -0.2, 1.0)        
    glVertex3f(1.0, -0.2, 1.0)        

    glColor3f(1.0, 1.0, 0.0)    
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)        
    glVertex3f(1.0, 0.2, -1.0)        

    glColor3f(0.0, 0.0, 1.0)    
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)        
    glVertex3f(-1.0, -0.2, -1.0)        
    glVertex3f(-1.0, -0.2, 1.0)        

    glColor3f(1.0, 0.0, 1.0)    
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)        
    glVertex3f(1.0, -0.2, -1.0)        
    glEnd()    
