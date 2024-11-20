import RPi.GPIO as GPIO
import time
from touchScreenBasicCoordOutput import read_touch_coordinates, Point
import threading

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
    'motor1': (-1000, 1000),  # Motor 1: -2000 to 2000 steps
    'motor2': (-1000, 1000),  # Motor 2: -2000 to 2000 steps
    'motor3': (-1000, 1000)   # Motor 3: -2000 to 2000 steps
}

# Velocity tracking variables
prev_time = time.time()
prev_x, prev_y = CENTER_X, CENTER_Y

# --------------------------------------------------------------------------------------------
# GPIO Setup
GPIO.setmode(GPIO.BCM)
for motor in MOTOR_PINS.values():
    GPIO.setup(motor['step'], GPIO.OUT)
    GPIO.setup(motor['dir'], GPIO.OUT)

# --------------------------------------------------------------------------------------------
def move_motor(motor, steps, clockwise):
    """
    Moves a single motor a specified number of steps in a specified direction,
    respecting the motor's range limits.
    """
    step_limit_min, step_limit_max = STEP_LIMITS[motor]

    # Clamp steps to within the limits
    steps_to_move = min(max(steps, step_limit_min), step_limit_max)

    GPIO.output(MOTOR_PINS[motor]['dir'], GPIO.HIGH if clockwise else GPIO.LOW)
    for _ in range(abs(steps_to_move)):
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.LOW)
        time.sleep(0.001)

def calculate_motor_steps(ball_x, ball_y, velocity_x, velocity_y):
    """
    Calculates the motor movements required to tilt the platform based on ball position and speed.
    Steps are proportional to both distance and speed.
    """
    dx = ball_x - CENTER_X
    dy = ball_y - CENTER_Y

    # Proportional step calculation
    steps_x = int((dx + velocity_x * 10) / 10)  # Scale velocity contribution
    steps_y = int((dy + velocity_y * 10) / 10)

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
    Main loop to balance the ball using proportional control based on distance and speed.
    """
    global prev_time, prev_x, prev_y

    try:
        while True:
            point = read_touch_coordinates()
            if point is None:
                continue

            ball_x, ball_y = point.x, point.y

            # Calculate velocity
            current_time = time.time()
            dt = current_time - prev_time if current_time != prev_time else 0.02
            velocity_x = (ball_x - prev_x) / dt
            velocity_y = (ball_y - prev_y) / dt

            # Update previous values
            prev_x, prev_y = ball_x, ball_y
            prev_time = current_time

            # Calculate motor steps based on position and velocity
            motor_steps = calculate_motor_steps(ball_x, ball_y, velocity_x, velocity_y)

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
