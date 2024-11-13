import RPi.GPIO as GPIO
import time
import math
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

# PID parameters (tune these values based on platform characteristics)
Kp = 0.6   # Proportional gain
Ki = 0.1   # Integral gain
Kd = 0.05  # Derivative gain

# Initialize error values for PID
prev_error_x, prev_error_y = 0, 0
integral_error_x, integral_error_y = 0, 0

# Define a function to control a single motor
def move_motor(motor, steps, clockwise):
    GPIO.output(MOTOR_PINS[motor]['dir'], GPIO.HIGH if clockwise else GPIO.LOW)
    for _ in range(steps):
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.HIGH)
        time.sleep(0.001)  # Control speed with sleep duration
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.LOW)
        time.sleep(0.001)

# PID calculation for each motor based on ball's position error
def calculate_motor_steps(ball_x, ball_y, dt=0.1):
    global prev_error_x, prev_error_y, integral_error_x, integral_error_y

    # Calculate proportional error
    error_x = CENTER_X - ball_x
    error_y = CENTER_Y - ball_y

    # Update integral error
    integral_error_x += error_x * dt
    integral_error_y += error_y * dt

    # Calculate derivative error
    derivative_error_x = (error_x - prev_error_x) / dt
    derivative_error_y = (error_y - prev_error_y) / dt

    # PID output for X and Y directions
    pid_output_x = Kp * error_x + Ki * integral_error_x + Kd * derivative_error_x
    pid_output_y = Kp * error_y + Ki * integral_error_y + Kd * derivative_error_y

    # Save current error as previous error for the next loop
    prev_error_x, prev_error_y = error_x, error_y

    # Scale PID output to steps, limiting to max 10 steps per control cycle
    max_steps = 10
    steps_x = max(-max_steps, min(int(pid_output_x), max_steps))
    steps_y = max(-max_steps, min(int(pid_output_y), max_steps))

    # Determine steps and direction for each motor based on PID output
    motor_steps = {}
    for motor, pos in MOTOR_POSITIONS.items():
        motor_error_x = CENTER_X - pos[0]
        motor_error_y = CENTER_Y - pos[1]
        
        # Motor tilt direction based on the error projections
        clockwise_x = steps_x > 0 if motor_error_x > 0 else steps_x < 0
        clockwise_y = steps_y > 0 if motor_error_y > 0 else steps_y < 0
        
        # Combine X and Y steps for each motor
        steps = abs(steps_x) + abs(steps_y)  # Simplified for illustration
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
            time.sleep(0.1)  # Update cycle delay (dt)
    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == "__main__":
    balance_ball()
