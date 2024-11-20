import RPi.GPIO as GPIO
import time
import threading
from touchScreenBasicCoordOutput import read_touch_coordinates, Point

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
    'motor1': (-1000, 1000),  # Motor 1: -1000 to 1000 steps
    'motor2': (-1000, 1000),  # Motor 2: -1000 to 1000 steps
    'motor3': (-1000, 1000)   # Motor 3: -1000 to 1000 steps
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

def calculate_motor_steps(ball_x, ball_y):
    """
    Calculates the motor movements required to tilt the platform based on ball position.
    Steps are proportional to the ball's distance from the center.
    """
    dx = ball_x - CENTER_X
    dy = ball_y - CENTER_Y

    # Proportional step calculation
    steps_x = int(dx / 10)  # Adjust divisor for sensitivity
    steps_y = int(dy / 10)

    # Determine motor steps and directions
    motor_steps = {
        'motor1': (abs(steps_y), steps_y > 0),  # Motor1 handles Y-axis
        'motor2': (abs(steps_x), steps_x > 0),  # Motor2 handles X-axis
        'motor3': (abs((steps_x + steps_y) // 2), (steps_x + steps_y) > 0)  # Combined effect
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
    Main loop to balance the ball using motor control.
    """
    try:
        while True:
            point = read_touch_coordinates()
            if point is None:
                continue

            ball_x, ball_y = point.x, point.y

            # Calculate motor steps based on ball position
            motor_steps = calculate_motor_steps(ball_x, ball_y)

            # Move each motor according to the calculated steps
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
    for _ in range(200):  # Arbitrary 200 steps to center
        move_motor('motor1', 1, True)
        move_motor('motor2', 1, True)
        move_motor('motor3', 1, True)
    print("Motors centered. Starting balance loop...")

    balance_ball()
