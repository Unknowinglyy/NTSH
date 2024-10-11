from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import board
import busio
import adafruit_mpu6050

# Initialize I2C and the MPU6050 sensor
i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

# Global variables to store the rotation angles
pitch = 0.0
roll = 0.0
yaw = 0.0
yaw_mode = False
last_time = 0

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

def drawText(position, textString):
    font = pygame.font.SysFont("Courier", 18, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def draw():
    global pitch, roll, yaw
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    # Display the current pitch, roll, and yaw values as text
    osd_text = "pitch: " + str("{0:.2f}".format(pitch)) + ", roll: " + str("{0:.2f}".format(roll))
    if yaw_mode:
        osd_line = osd_text + ", yaw: " + str("{0:.2f}".format(yaw))
    else:
        osd_line = osd_text
    drawText((-2, -2, 2), osd_line)

    # Apply the accumulated rotations based on the gyro data
    if yaw_mode:
        glRotatef(yaw, 0.0, 1.0, 0.0)  # Yaw (rotate around y-axis)

    glRotatef(pitch, 1.0, 0.0, 0.0)    # Pitch (rotate around x-axis)
    glRotatef(-1 * roll, 0.0, 0.0, 1.0)  # Roll (rotate around z-axis)

    # Draw a simple 3D cube
    glBegin(GL_QUADS)
    
    # Top face (green)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    # Bottom face (orange)
    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    # Front face (red)
    glColor3f(1.0, 0.0, 0.0)
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

    # Left face (blue)
    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    # Right face (purple)
    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glEnd()

def read_data(delta_time):
    global pitch, roll, yaw

    # Read gyro data (angular velocity)
    ax, ay, az = mpu.gyro

    # Integrate the angular velocity to update the orientation angles
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
    screen = pygame.display.set_mode((640, 480), video_flags)
    pygame.display.set_caption("Press Esc to quit, z toggles yaw mode")
    resize(640, 480)
    init()

    last_time = pygame.time.get_ticks()  # Initialize last_time
    frames = 0
    ticks = pygame.time.get_ticks()

    while True:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()
            break
        if event.type == KEYDOWN and event.key == K_z:
            yaw_mode = not yaw_mode

        # Calculate the time delta since the last frame
        current_time = pygame.time.get_ticks()
        delta_time = (current_time - last_time) / 1000.0  # Convert to seconds
        last_time = current_time

        read_data(delta_time)  # Update the pitch, roll, and yaw
        draw()  # Draw the cube with the updated orientation

        pygame.display.flip()
        frames += 1

    print("fps: %d" % ((frames * 1000) / (pygame.time.get_ticks() - ticks)))

if __name__ == '__main__':
    main()
