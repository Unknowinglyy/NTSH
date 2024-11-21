import RPi.GPIO as GPIO
import time
import threading
from touchScreenBasicCoordOutput import read_touch_coordinates, Point
from simple_pid import PID  # Import the PID class from the simple_pid library

# --------------------------------------------------------------------------------------------
# GPIO setup for stepper motors
MOTOR_PINS = {
    'motor1': {'step': 23, 'dir': 24},
    'motor2': {'step': 20, 'dir': 21},
    'motor3': {'step': 5, 'dir': 6}
}

# Center position of the touchscreen
CENTER_X, CENTER_Y = 2025, 2045

# Define step limits for each motor
STEP_LIMITS = {
    'motor1': (-500, 1000),  # Motor 1: -1000 to 1000 steps
    'motor2': (-500, 1000),  # Motor 2: -1000 to 1000 steps
    'motor3': (-500, 1000)   # Motor 3: -1000 to 1000 steps
}

# Track current motor positions
current_steps = {
    'motor1': 0,
    'motor2': 0,
    'motor3': 0
}

# --------------------------------------------------------------------------------------------
# GPIO Setup
GPIO.setmode(GPIO.BCM)
for motor in MOTOR_PINS.values():
    GPIO.setup(motor['step'], GPIO.OUT)
    GPIO.setup(motor['dir'], GPIO.OUT)

def move_motor(motor, steps, clockwise):
    """
    Moves a single motor a specified number of steps in a specified direction,
    ensuring it stays within defined limits.
    """
    global current_steps

    # Determine the new position
    if clockwise:
        new_position = current_steps[motor] + steps
    else:
        new_position = current_steps[motor] - steps

    # Clamp the position within the limits
    min_limit, max_limit = STEP_LIMITS[motor]
    if new_position < min_limit:
        steps = current_steps[motor] - min_limit  # Reduce steps to hit the minimum limit
        new_position = min_limit
    elif new_position > max_limit:
        steps = max_limit - current_steps[motor]  # Reduce steps to hit the maximum limit
        new_position = max_limit

    # Move the motor if steps are non-zero
    if steps != 0:
        GPIO.output(MOTOR_PINS[motor]['dir'], GPIO.HIGH if clockwise else GPIO.LOW)
        for _ in range(abs(steps)):
            GPIO.output(MOTOR_PINS[motor]['step'], GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(MOTOR_PINS[motor]['step'], GPIO.LOW)
            time.sleep(0.001)

        # Update the current step count
        current_steps[motor] = new_position

def setup_pid():
    """
    Sets up the PID controllers for X and Y axis.
    """
    # Create PID controllers for X and Y axis
    pid_x = PID(0.002, 0.5, 0.1, setpoint=2000)  # Kp, Ki, Kd for X axis
    pid_y = PID(0.002, 0.5, 0.1, setpoint=2000)  # Kp, Ki, Kd for Y axis

    pid_x.output_limits = (-10, 10)  # Step limits for motor 2 (X axis)
    pid_y.output_limits = (-10, 10)  # Step limits for motor 1 (Y axis)

    return pid_x, pid_y

def calculate_motor_steps(ball_x, ball_y, pid_x, pid_y):
    """
    Calculates the motor movements required to tilt the platform based on ball position
    using PID control.
    """
    # Calculate the errors for X and Y
    error_x = ball_x - CENTER_X
    error_y = ball_y - CENTER_Y

    # Use PID to compute the required steps for each motor
    pid_x.setpoint = 0  # We want to return to the center (X-axis)
    pid_y.setpoint = 0  # We want to return to the center (Y-axis)

    motor_x_steps = pid_x(error_x)  # Get PID output for X-axis
    motor_y_steps = pid_y(error_y)  # Get PID output for Y-axis

    # Combine the PID output for motors
    motor_steps = {
        'motor1': (int(abs(motor_y_steps)), motor_y_steps < 0),  # Motor1 handles Y-axis #DC ~ Changed signs here to fix wrong direction tilt
        'motor2': (int(abs(motor_x_steps)), motor_x_steps < 0),  # Motor2 handles X-axis
        'motor3': (int(abs((motor_x_steps + motor_y_steps) // 2)), (motor_x_steps + motor_y_steps) > 0)  # Combined effect for Motor3
    }

    return motor_steps

def move_motors_concurrently(motor_steps):
    """
    Moves the motors concurrently using threading.
    """
    threads = []
    for motor, (steps, clockwise) in motor_steps.items():
        if steps > 0:
            t = threading.Thread(target=move_motor, args=(motor, steps, clockwise))
            threads.append(t)
            t.start()
    for t in threads:
        t.join()

def balance_ball():
    """
    Main loop to balance the ball using PID-controlled motor movements.
    """
    pid_x, pid_y = setup_pid()  # Setup PID controllers for X and Y axis

    try:
        while True:
            point = read_touch_coordinates()
            if point is None:
                continue

            ball_x, ball_y = point.x, point.y

            # Calculate motor steps based on ball position using PID
            motor_steps = calculate_motor_steps(ball_x, ball_y, pid_x, pid_y)

            # Move each motor according to the calculated PID-controlled steps
            move_motors_concurrently(motor_steps)

            time.sleep(0.02)  # Update cycle delay (20 ms)
    except KeyboardInterrupt:
        print("Exiting program...")
    finally:
        GPIO.cleanup()

# --------------------------------------------------------------------------------------------
if __name__ == "__main__":
    # Centering motors before starting
    print("Centering motors...")
    for _ in range(400):  # Arbitrary 200 steps to center
        move_motor('motor1', 1, True)
        move_motor('motor2', 1, True)
        move_motor('motor3', 1, True)
    print("Motors centered. Starting balance loop...")

    balance_ball()
