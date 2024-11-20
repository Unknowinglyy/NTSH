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
CENTER_X = 2025
# Ball detection thresholds
BALL_DETECTION_THRESHOLD = 10

# Proportional gain for velocity control
Kp_velocity = 0.1

# --------------------------------------------------------------------------------------------
# GPIO Setup
GPIO.setmode(GPIO.BCM)
for motor in MOTOR_PINS.values():
    GPIO.setup(motor['step'], GPIO.OUT)
    GPIO.setup(motor['dir'], GPIO.OUT)

# Velocity tracking variables
prev_time = time.time()
prev_x = CENTER_X

# --------------------------------------------------------------------------------------------
def move_motor(motor, steps, clockwise):
    """
    Moves a single motor a specified number of steps in a specified direction.
    """
    GPIO.output(MOTOR_PINS[motor]['dir'], GPIO.HIGH if clockwise else GPIO.LOW)
    for _ in range(abs(steps)):
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.LOW)
        time.sleep(0.001)

def calculate_motor_steps(velocity_x):
    """
    Calculates the motor movements required to tilt the plane based on ball velocity along the x-axis.
    """
    # Compute steps proportional to velocity
    steps_x = int(Kp_velocity * velocity_x)

    # Determine motor steps and directions
    motor_steps = {
        'motor1': (abs(steps_x), steps_x < 0),  # Clockwise if steps_x < 0
        'motor2': (abs(steps_x), steps_x < 0),
        'motor3': (abs(steps_x), steps_x < 0)
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
    Main loop to balance the ball along the x-axis using velocity control.
    """
    global prev_time, prev_x

    try:
        while True:
            point = read_touch_coordinates()
            if point is None:
                continue

            ball_x = point.x
            current_time = time.time()

            # Calculate velocity
            dt = current_time - prev_time if current_time != prev_time else 0.02
            velocity_x = (ball_x - prev_x) / dt

            # Update previous values
            prev_x = ball_x
            prev_time = current_time

            # Calculate motor steps based on velocity
            motor_steps = calculate_motor_steps(velocity_x)

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