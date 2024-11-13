import RPi.GPIO as GPIO
import time
import math
from simple_pid import PID
from touchScreenBasicCoordOutput import read_touch_coordinates, Point

# GPIO setup for stepper motors
MOTOR_PINS = {
    'motor1': {'step': 23, 'dir': 24},
    'motor2': {'step': 20, 'dir': 21},
    'motor3': {'step': 5, 'dir': 6}
}

# Motor corner positions
MOTOR_POSITIONS = {
    'motor1': (3685, 200),
    'motor2': (3685, 3820),
    'motor3': (390, 1810)
}

# Center position of the touchscreen
CENTER_X, CENTER_Y = 2025, 2045

# Set up GPIO
GPIO.setmode(GPIO.BCM)
for motor in MOTOR_PINS.values():
    GPIO.setup(motor['step'], GPIO.OUT)
    GPIO.setup(motor['dir'], GPIO.OUT)

# PID controllers for X and Y directions
pid_x = PID(0.6, 0.1, 0.05, setpoint=CENTER_X)
pid_y = PID(0.6, 0.1, 0.05, setpoint=CENTER_Y)

# Configure sample time (update frequency) and output limits
pid_x.sample_time = 0.1  # 100 ms update rate
pid_y.sample_time = 0.1
pid_x.output_limits = (-10, 10)  # Limiting output to max ±10 steps
pid_y.output_limits = (-10, 10)

# Define a function to control a single motor
def move_motor(motor, steps, clockwise):
    GPIO.output(MOTOR_PINS[motor]['dir'], GPIO.HIGH if clockwise else GPIO.LOW)
    for _ in range(abs(steps)):
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.LOW)
        time.sleep(0.001)

# Function to calculate motor steps based on PID outputs
def calculate_motor_steps(ball_x, ball_y):
    # Get PID output steps for X and Y directions
    steps_x = int(pid_x(ball_x))
    steps_y = int(pid_y(ball_y))

    # Determine steps and direction for each motor based on PID output
    motor_steps = {}
    for motor, pos in MOTOR_POSITIONS.items():
        motor_error_x = CENTER_X - pos[0]
        motor_error_y = CENTER_Y - pos[1]
        
        # Motor tilt direction based on error projections
        clockwise_x = steps_x > 0 if motor_error_x > 0 else steps_x < 0
        clockwise_y = steps_y > 0 if motor_error_y > 0 else steps_y < 0
        
        # Combine X and Y steps for each motor
        steps = abs(steps_x) + abs(steps_y)  # Adjust based on each motor's distance influence
        clockwise = clockwise_x if abs(steps_x) > abs(steps_y) else clockwise_y
        motor_steps[motor] = (steps, clockwise)

    return motor_steps

# Main loop
def balance_ball():
    try:
        while True:
            point = read_touch_coordinates()
            ball_x, ball_y = point.x, point.y
            motor_steps = calculate_motor_steps(ball_x, ball_y)

            # Move each motor according to the calculated steps
            for motor, (steps, clockwise) in motor_steps.items():
                move_motor(motor, steps, clockwise)
            time.sleep(0.1)  # Update cycle delay
    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == "__main__":
    balance_ball()