import RPi.GPIO as GPIO
import time
from simple_pid import PID
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
# Ball detection thresholds
BALL_DETECTION_THRESHOLD = 10

# --------------------------------------------------------------------------------------------
# GPIO Setup
GPIO.setmode(GPIO.BCM)
for motor in MOTOR_PINS.values():
    GPIO.setup(motor['step'], GPIO.OUT)
    GPIO.setup(motor['dir'], GPIO.OUT)

# PID controllers for X and Y directions
pid_x = PID(0.01, 0.1, 0.01, setpoint=CENTER_X)
pid_y = PID(0.01, 0.1, 0.01, setpoint=CENTER_Y)

# Configure sample time (update frequency) and output limits
pid_x.sample_time = 0.02  # 20 ms update rate
pid_y.sample_time = 0.02
pid_x.output_limits = (-10, 10)  # Limit to ±10 steps
pid_y.output_limits = (-10, 10)

# Velocity tracking variables
prev_time = time.time()
prev_x, prev_y = CENTER_X, CENTER_Y

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

def calculate_motor_steps(ball_x, ball_y, velocity_x, velocity_y):
    """
    Calculates the motor movements required to tilt the platform along X and Y axes.
    """
    if abs(ball_x - CENTER_X) < BALL_DETECTION_THRESHOLD and abs(ball_y - CENTER_Y) < BALL_DETECTION_THRESHOLD:
        return {motor: (0, True) for motor in MOTOR_PINS}  # No movement if ball is near center.

    # Update PID setpoints dynamically based on velocity
    pid_x.setpoint = CENTER_X - velocity_x * 0.1
    pid_y.setpoint = CENTER_Y - velocity_y * 0.1

    # Compute PID outputs
    pid_output_x = int(pid_x(ball_x))
    pid_output_y = int(pid_y(ball_y))

    # Map PID outputs to motor steps
    # Assuming:
    #   motor1 tilts along -X and +Y
    #   motor2 tilts along +X and +Y
    #   motor3 tilts along -Y
    motor_steps = {
        'motor1': (abs(pid_output_x + pid_output_y), (pid_output_x + pid_output_y) < 0),
        'motor2': (abs(-pid_output_x + pid_output_y), (-pid_output_x + pid_output_y) < 0),
        'motor3': (abs(-pid_output_y), (-pid_output_y) < 0)
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
    Main loop to balance the ball using PID, motor control, and velocity calculations.
    """
    global prev_time, prev_x, prev_y

    try:
        while True:
            point = read_touch_coordinates()
            if point is None:
                continue

            ball_x, ball_y = point.x, point.y
            current_time = time.time()

            # Calculate velocity
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
