import RPi.GPIO as GPIO
import time
import threading
from touchScreenBasicCoordOutput import read_touch_coordinates

# --------------------------------------------------------------------------------------------
# GPIO setup for stepper motors
MOTOR_PINS = {
    'motor1': {'step': 23, 'dir': 24},  # (3670, 210)
    'motor2': {'step': 20, 'dir': 21},  # (397, 2005)
    'motor3': {'step': 5, 'dir': 6}     # (3670, 3800)
}

# Center position of the platform
CENTER_X, CENTER_Y = 2025, 2045
STEP_DELAY = 0.001  # 1 ms delay for microstepping

# Maximum and minimum step limits
MAX_STEPS = 200  # Maximum steps to take in a single move
MIN_STEPS = 5    # Minimum steps to avoid unnecessary adjustments

# GPIO Setup
GPIO.setmode(GPIO.BCM)
for motor in MOTOR_PINS.values():
    GPIO.setup(motor['step'], GPIO.OUT)
    GPIO.setup(motor['dir'], GPIO.OUT)

# --------------------------------------------------------------------------------------------
def move_motor(motor, steps, clockwise):
    """
    Moves a single motor a specified number of steps in a specified direction.
    """
    GPIO.output(MOTOR_PINS[motor]['dir'], GPIO.HIGH if clockwise else GPIO.LOW)
    for _ in range(abs(steps)):
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.HIGH)
        time.sleep(STEP_DELAY)
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.LOW)
        time.sleep(STEP_DELAY)

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

def calculate_motor_steps(ball_x, ball_y):
    """
    Calculates motor steps and directions based on the ball's position.
    The step size is proportional to the ball's distance from the center.
    """
    # Calculate distances from the center
    distance_x = ball_x - CENTER_X
    distance_y = ball_y - CENTER_Y

    # Normalize distances to determine step proportions
    steps_x = int(MAX_STEPS * abs(distance_x) / CENTER_X)
    steps_y = int(MAX_STEPS * abs(distance_y) / CENTER_Y)

    # Enforce minimum and maximum step limits
    steps_x = max(MIN_STEPS, min(steps_x, MAX_STEPS))
    steps_y = max(MIN_STEPS, min(steps_y, MAX_STEPS))

    # Determine motor directions
    direction_motor1 = distance_x < 0  # Motor1 tilts up if ball is left of center
    direction_motor2 = distance_y > 0  # Motor2 tilts down if ball is below center
    direction_motor3 = distance_x > 0  # Motor3 tilts up if ball is right of center

    # Motor steps mapping
    motor_steps = {
        'motor1': (steps_x, direction_motor1),
        'motor2': (steps_y, direction_motor2),
        'motor3': (steps_x, direction_motor3)
    }

    return motor_steps

# --------------------------------------------------------------------------------------------
def balance_ball():
    """
    Main loop to balance the ball by proportional adjustment.
    """
    try:
        while True:
            point = read_touch_coordinates()
            if point is None:
                continue

            ball_x, ball_y = point.x, point.y

            # Calculate motor steps based on the ball's position
            motor_steps = calculate_motor_steps(ball_x, ball_y)

            # Move the motors accordingly
            move_motors_concurrently(motor_steps)

            time.sleep(0.02)  # Delay for next update
    except KeyboardInterrupt:
        print("Exiting program...")
    finally:
        GPIO.cleanup()

# --------------------------------------------------------------------------------------------
if __name__ == "__main__":
    print("Starting balance loop...")
    balance_ball()
