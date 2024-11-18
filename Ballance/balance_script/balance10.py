import math
import RPi.GPIO as GPIO
import time
from kine2 import Kinematics  # Import the Kinematics class
from touchScreenBasicCoordOutput import read_touch_coordinates
import threading

# --------------------------------------------------------------------------------------------
# GPIO setup for stepper motors
MOTOR_PINS = {
    'motor1': {'step': 23, 'dir': 24},
    'motor2': {'step': 20, 'dir': 21},
    'motor3': {'step': 5, 'dir': 6}
}

# Parameters
CENTER_X, CENTER_Y = 2025, 2045  # Touchscreen center offsets
BALL_DETECTION_THRESHOLD = 20    # Ball detection range
angOrig = 160          # Original angle
angToStep = 3200 / 360           # Steps per degree
ks = 20                          # Speed amplifying constant
kp, ki, kd = 4E-4, 2E-6, 7E-3    # PID constants

# Kinematics parameters
d, e, f, g = 2, 3.125, 1.75, 3.669291339
kinematics = Kinematics(d, e, f, g)

# Initialize stepper variables
pos = [0, 0, 0]
speed = [0, 0, 0]
speedPrev = [0, 0, 0]
error = [0, 0]
integr = [0, 0]
deriv = [0, 0]
out = [0, 0]
detected = False

# GPIO setup
GPIO.setmode(GPIO.BCM)
for motor in MOTOR_PINS.values():
    GPIO.setup(motor['step'], GPIO.OUT)
    GPIO.setup(motor['dir'], GPIO.OUT)

# --------------------------------------------------------------------------------------------
def debug_log(msg):
    """
    Helper function to print debugging messages with a timestamp.
    """
    print(f"[{time.time():.2f}] {msg}")

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

def move_motors_concurrently(motor_steps):
    """
    Moves the motors concurrently using threading.
    """
    threads = []
    for motor, (steps, clockwise) in motor_steps.items():
        if steps > 0:  # Only move motors with non-zero steps
            t = threading.Thread(target=move_motor, args=(motor, steps, clockwise))
            threads.append(t)
            t.start()
    for t in threads:
        t.join()

def calculate_motor_steps(ball_x, ball_y):
    """
    Calculates the motor movements required to tilt the plane based on ball position.
    """
    global detected, error, integr, deriv, out, speed

    if abs(ball_x - CENTER_X) < BALL_DETECTION_THRESHOLD and abs(ball_y - CENTER_Y) < BALL_DETECTION_THRESHOLD:
        return {motor: (0, True) for motor in MOTOR_PINS}  # No movement if ball is near center.

    detected = True
    for i in range(2):
        error[i] = (CENTER_X - ball_x) if i == 0 else (CENTER_Y - ball_y)
        integr[i] += error[i]
        deriv[i] = error[i] - error[i - 1] if i > 0 else 0
        out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i]
        out[i] = max(min(out[i], 0.25), -0.25)  # Constrain output
        debug_log(f"PID output {['X', 'Y'][i]}: error={error[i]}, integr={integr[i]}, deriv={deriv[i]}, out={out[i]}")

    motor_steps = {
        'motor1': (abs(int(out[0] * angToStep)), out[0] < 0),
        'motor2': (abs(int(out[1] * angToStep)), out[1] < 0),
        'motor3': (abs(int((out[0] + out[1]) * angToStep // 2)), (out[0] + out[1]) < 0)
    }

    return motor_steps

def balance_ball():
    """
    Main loop to balance the ball using PID and motor control.
    """
    debug_log("Starting balance loop...")
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
        debug_log("Exiting program...")
        # Move all motors CCW 100 steps
        debug_log("Resetting motors...")
        for motor in MOTOR_PINS.keys():
            move_motor(motor, 100, False)
    finally:
        GPIO.cleanup()

# --------------------------------------------------------------------------------------------
if __name__ == "__main__":
    debug_log("Centering motors...")
    for _ in range(300):  # Arbitrary 100 steps to center
        move_motor('motor1', 1, True)
        move_motor('motor2', 1, True)
        move_motor('motor3', 1, True)
    debug_log("Motors centered. Starting balance loop...")

    balance_ball()