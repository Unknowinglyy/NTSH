import RPi.GPIO as GPIO
import time
import math
from touchScreenBasicCoordOutput import read_touch_coordinates, Point
import threading

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
for motor in MOTOR_PINS.values():
    GPIO.setup(motor['step'], GPIO.OUT)
    GPIO.setup(motor['dir'], GPIO.OUT)

def move_all_motors_cw(steps, delay):
    # Move all motors clockwise
    GPIO.output(MOTOR_PINS['motor1']['dir'], GPIO.HIGH)
    GPIO.output(MOTOR_PINS['motor2']['dir'], GPIO.HIGH)
    GPIO.output(MOTOR_PINS['motor3']['dir'], GPIO.HIGH)
    for _ in range(steps):
        GPIO.output(MOTOR_PINS['motor1']['step'], GPIO.HIGH)
        GPIO.output(MOTOR_PINS['motor2']['step'], GPIO.HIGH)
        GPIO.output(MOTOR_PINS['motor3']['step'], GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(MOTOR_PINS['motor1']['step'], GPIO.LOW)
        GPIO.output(MOTOR_PINS['motor2']['step'], GPIO.LOW)
        GPIO.output(MOTOR_PINS['motor3']['step'], GPIO.LOW)
        time.sleep(delay)

# Define a function to control a single motor
def move_motor(motor, steps, clockwise):
    GPIO.output(MOTOR_PINS[motor]['dir'], GPIO.HIGH if clockwise else GPIO.LOW)
    for _ in range(abs(steps)):
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.LOW)
        time.sleep(0.001)

# Function to check if the ball is near a specific motor
def is_ball_near_motor(ball_x, ball_y, motor_pos, threshold=100):
    distance = math.sqrt((ball_x - motor_pos[0])**2 + (ball_y - motor_pos[1])**2)
    return distance < threshold

# Main loop
def balance_ball():
    try:
        while True:
            point = read_touch_coordinates()
            if point is not None:
                ball_x, ball_y = point.x, point.y

                # Check if the ball is near any motor and move that motor
                for motor, pos in MOTOR_POSITIONS.items():
                    if is_ball_near_motor(ball_x, ball_y, pos):
                        move_motor(motor, 100, True)  # Move 100 steps clockwise
                        break  # Only move one motor at a time
            time.sleep(0.1)  # Update cycle delay
    except KeyboardInterrupt:
        GPIO.cleanup()

# --------------------------------------------------------------------------------------------
if __name__ == "__main__":
    # Move all motors 100 Steps CW
    move_all_motors_cw(100, 0.05)

    # Begin Balance 
    balance_ball()