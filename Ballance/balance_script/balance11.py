import RPi.GPIO as GPIO
import time
from simple_pid import PID
from touchScreenBasicCoordOutput import read_touch_coordinates, Point

# --------------------------------------------------------------------------------------------
# GPIO setup for stepper motors
MOTOR_PINS = {
    'motor1': {'step': 23, 'dir': 24},
    'motor2': {'step': 20, 'dir': 21},
    'motor3': {'step': 5, 'dir': 6}
}

# Stepper motor constants
STEPS_PER_REV = 3200  # 16 microstepping
ANG_TO_STEP = STEPS_PER_REV / 360  # Steps per degree

# Touchscreen center offsets
CENTER_X, CENTER_Y = 2025, 2045

# PID constants
PID_KP = 4E-4
PID_KI = 2E-6
PID_KD = 7E-3

# Enable GPIO
ENABLE_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENABLE_PIN, GPIO.OUT)
GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Enable drivers

# Stepper motor setup
for motor in MOTOR_PINS.values():
    GPIO.setup(motor['step'], GPIO.OUT)
    GPIO.setup(motor['dir'], GPIO.OUT)

# PID controllers for X and Y directions
pid_x = PID(PID_KP, PID_KI, PID_KD, setpoint=CENTER_X)
pid_y = PID(PID_KP, PID_KI, PID_KD, setpoint=CENTER_Y)
pid_x.sample_time = 0.02  # Update every 20ms
pid_y.sample_time = 0.02

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

def calculate_steps(ball_x, ball_y):
    """
    Calculate stepper movements based on ball position relative to the center.
    """
    dx = ball_x - CENTER_X
    dy = ball_y - CENTER_Y

    # Calculate proportional steps based on distance
    steps_x = int(pid_x(dx) * ANG_TO_STEP)
    steps_y = int(pid_y(dy) * ANG_TO_STEP)

    # Determine motor directions and steps
    motor_steps = {
        'motor1': (steps_x, steps_x < 0),
        'motor2': (steps_y, steps_y < 0),
        'motor3': (steps_x + steps_y, (steps_x + steps_y) < 0)
    }

    return motor_steps

def move_motors(motor_steps):
    """
    Moves all motors concurrently.
    """
    for motor, (steps, clockwise) in motor_steps.items():
        if steps != 0:
            move_motor(motor, abs(steps), clockwise)

def balance_ball():
    """
    Main balancing loop.
    """
    try:
        while True:
            # Read ball position from touchscreen
            point = read_touch_coordinates()
            ball_x, ball_y = point.x, point.y

            # Calculate stepper motor movements
            motor_steps = calculate_steps(ball_x, ball_y)

            # Move the motors to balance the ball
            move_motors(motor_steps)

            time.sleep(0.02)  # PID update interval
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()

# --------------------------------------------------------------------------------------------
if __name__ == "__main__":
    print("Starting ball balancing...")
    balance_ball()
