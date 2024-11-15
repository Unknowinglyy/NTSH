import RPi.GPIO as GPIO
import time
import math
from touchScreenBasicCoordOutput import read_touch_coordinates, Point
import threading
# --------------------------------------------------------------------------------------------
# GPIO setup for stepper motors
MOTOR_PINS = {
    'motor1': {'step': 23, 'dir': 24},
    'motor2': {'step': 20, 'dir': 21},
    'motor3': {'step': 5, 'dir': 6}
}

# Motor corner positions
MOTOR_POSITIONS = {
    'motor1': (3670, 210),
    'motor2': (397, 2033.5),
    'motor3': (3670, 3800)
}

# Center position of the touchscreen
CENTER_X, CENTER_Y = 2005, 2033.5

# Set up GPIO
GPIO.setmode(GPIO.BCM)
for motor in MOTOR_PINS.key():
    GPIO.setup(motor['step'], GPIO.OUT)
    GPIO.setup(motor['dir'], GPIO.OUT)
# --------------------------------------------------------------------------------------------
def move_all_motors_cw(steps, delay):
    # Move all motors CW
    for _ in range(steps):
        for motor in MOTOR_PINS.keys():
            GPIO.output(motor['dir'], GPIO.HIGH)
            GPIO.output(motor['step'], GPIO.HIGH)
        time.sleep(delay)
        for motor in MOTOR_PINS.values():
            GPIO.output(motor['step'], GPIO.LOW)
        time.sleep(delay)

# Define a function to control a single motor
def move_motor(motor, steps, clockwise):
    GPIO.output(MOTOR_PINS[motor]['dir'], GPIO.HIGH if clockwise else GPIO.LOW)
    for _ in range(abs(steps)):
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.LOW)
        time.sleep(0.001)

# Main loop
def balance_ball():
    try:
        while True:
            point = read_touch_coordinates()
            if point is not None:
                ball_x, ball_y = point.x, point.y
                error_x = CENTER_X - ball_x
                error_y = CENTER_Y - ball_y
                if (error_x > 0):
                    move_motor('motor1', 100, True)
                    move_motor('motor3', 100, True)
                elif (error_x < 0):
                    move_motor('motor1', 100, False)
                    move_motor('motor3', 100, False)  


    except KeyboardInterrupt:
        GPIO.cleanup()

# --------------------------------------------------------------------------------------------
if __name__ == "__main__":
    # Move all motors 100 Steps CW
    move_all_motors_cw(100, 0.05)

    # Begin Balance 
    balance_ball()