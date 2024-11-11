from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import board
import busio
import adafruit_mpu6050

# Initialize I2C and MPU6050 sensor
i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

# Global variables to track orientation
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

    # Display current pitch, roll, and yaw values
    osd_text = f"pitch: {pitch:.2f}, roll: {roll:.2f}"
    if yaw_mode:
        osd_text += f", yaw: {yaw:.2f}"
    drawText((-2, -2, 2), osd_text)

    # Apply rotations based on the accumulated angles
    if yaw_mode:
        glRotatef(yaw, 0.0, 1.0, 0.0)  # Yaw around y-axis
    glRotatef(pitch, 1.0, 0.0, 0.0)   # Pitch around x-axis
    glRotatef(-1 * roll, 0.0, 0.0, 1.0)  # Roll around z-axis

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

    # Read gyroscope data from MPU6050
    gyro_x, gyro_y, gyro_z = mpu.gyro
    print(f"Gyro Data -> X: {gyro_x:.2f}, Y: {gyro_y:.2f}, Z: {gyro_z:.2f}")

    # Integrate gyroscope data to update pitch, roll, and yaw
    pitch += gyro_y * delta_time
    roll += gyro_x * delta_time
    if yaw_mode:
        yaw += gyro_z * delta_time

def main():
    global yaw_mode, last_time

    # Pygame setup for OpenGL window
    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((640, 480), video_flags)
    pygame.display.set_caption("Press Esc to quit, z toggles yaw mode")
    resize(640, 480)
    init()

    # Initialize time tracking
    last_time = pygame.time.get_ticks()
    frames = 0
    ticks = pygame.time.get_ticks()

    while True:
        # Event handling
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()
            break
        if event.type == KEYDOWN and event.key == K_z:
            yaw_mode = not yaw_mode

        # Calculate time delta between frames
        current_time = pygame.time.get_ticks()
        delta_time = (current_time - last_time) / 1000.0  # Convert to seconds
        last_time = current_time

        read_data(delta_time)  # Update orientation based on sensor data
        draw()  # Render the cube with updated orientation

        pygame.display.flip()
        frames += 1

    print(f"fps: {frames * 1000 // (pygame.time.get_ticks() - ticks)}")

if __name__ == '__main__':
    main()
