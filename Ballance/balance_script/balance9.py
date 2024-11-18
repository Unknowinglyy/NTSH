import RPi.GPIO as GPIO
import time
from simple_pid import PID
from touchScreenBasicCoordOutput import read_touch_coordinates
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
# Ball detection thresholds
BALL_DETECTION_THRESHOLD = 20

# --------------------------------------------------------------------------------------------
# GPIO Setup
GPIO.setmode(GPIO.BCM)
for motor in MOTOR_PINS.values():
    GPIO.setup(motor['step'], GPIO.OUT)
    GPIO.setup(motor['dir'], GPIO.OUT)

# PID controllers for X and Y directions
pid_x = PID(5, 0.1, .05, setpoint=CENTER_X)
pid_y = PID(5, 0.1, .05, setpoint=CENTER_Y)

# Configure sample time (update frequency) and output limits
pid_x.sample_time = 0.01  # ms update rate
pid_y.sample_time = 0.01
pid_x.output_limits = (-2, 2)  # Limit ±steps
pid_y.output_limits = (-2, 2)

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

def calculate_motor_steps(ball_x, ball_y):
    """
    Calculates the motor movements required to tilt the plane based on ball position.
    """
    if abs(ball_x - CENTER_X) < BALL_DETECTION_THRESHOLD and abs(ball_y - CENTER_Y) < BALL_DETECTION_THRESHOLD:
        return {motor: (0, True) for motor in MOTOR_PINS}  # No movement if ball is near center.

    # Compute PID outputs
    steps_x = int(pid_x(ball_x))
    steps_y = int(pid_y(ball_y))
    print(f"Steps to move: {steps_x}, {steps_y}")
    # Determine motor steps and directions
    motor_steps = {
        'motor1': (abs(steps_x), steps_x < 0),  # Clockwise if steps_x < 0
        'motor2': (abs(steps_y), steps_y < 0),
        'motor3': (abs((steps_x + steps_y) // 2), (steps_x + steps_y) < 0)
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
    Main loop to balance the ball using PID and motor control.
    """
    try:
        while True:
            point = read_touch_coordinates()
            if point is None:
                continue

            ball_x, ball_y = point.x, point.y
            motor_steps = calculate_motor_steps(ball_x, ball_y)

            # Move each motor according to the calculated steps
            move_motors_concurrently(motor_steps)

            time.sleep(0.01)  # Update cycle delay (10 ms)
    except KeyboardInterrupt:
        print("Exiting program...")
        # Move all motors CCW 100 steps
        print("Resetting motors...")
        for motor in MOTOR_PINS.keys():
            move_motor(motor, 100, False)
    finally:
        GPIO.cleanup()

# --------------------------------------------------------------------------------------------
if __name__ == "__main__":
    # Centering motors before starting
    print("Centering motors...")
    for _ in range(300):  # Arbitrary 100 steps to center
        move_motor('motor1', 1, True)
        move_motor('motor2', 1, True)
        move_motor('motor3', 1, True)
    print("Motors centered. Starting balance loop...")
    move_motor('motor2', 2, False)
    time.sleep(1)
    print("Motor 2 test")
    move_motor('motor2', 2, False)
    move_motor('motor2', 2, True)
    # balance_ball()