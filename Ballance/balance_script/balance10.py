import RPi.GPIO as GPIO
import time
from simple_pid import PID
from touchScreenBasicCoordOutput import read_touch_coordinates
import threading
import math

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
BALL_DETECTION_THRESHOLD = 1

# PID constants
kp = 4E-4
ki = 2E-6
kd = 7E-3

# PID controllers for X and Y directions
pid_x = PID(kp, ki, kd, setpoint=CENTER_X)
pid_y = PID(kp, ki, kd, setpoint=CENTER_Y)

# Configure sample time (update frequency) and output limits
pid_x.sample_time = 0.01  # 10 ms update rate
pid_y.sample_time = 0.01
pid_x.output_limits = (-10, 10)  # Limit to ±10 steps
pid_y.output_limits = (-10, 10)

# Stepper motor variables
angOrig = 206.662752199
angToStep = 3200 / 360  # angle to step conversion factor (steps per degree)
speed = [0, 0, 0]
speedPrev = [0, 0, 0]
ks = 20

# Touch screen offsets
Xoffset = 500
Yoffset = 500

# Ball detection flag
detected = False

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
        time.sleep(0.001)
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.LOW)
        time.sleep(0.001)

def move_to(hz, nx, ny):
    """
    Moves/positions the platform with the given parameters.
    """
    global detected
    pos = [0, 0, 0]
    if detected:
        for i in range(3):
            pos[i] = round((angOrig - machine_theta(i, hz, nx, ny)) * angToStep)
        for i, motor in enumerate(MOTOR_PINS.keys()):
            move_motor(motor, pos[i], True)
    else:
        for i in range(3):
            pos[i] = round((angOrig - machine_theta(i, hz, 0, 0)) * angToStep)
        for i, motor in enumerate(MOTOR_PINS.keys()):
            move_motor(motor, pos[i], True)

def machine_theta(i, hz, nx, ny):
    """
    Placeholder function for machine.theta(i, hz, nx, ny).
    Replace this with the actual inverse kinematics calculation.
    """
    # Simple inverse kinematics calculation for demonstration purposes
    if i == 0:
        return math.atan2(ny, nx) * 180 / math.pi
    elif i == 1:
        return math.atan2(ny, nx) * 180 / math.pi + 120
    elif i == 2:
        return math.atan2(ny, nx) * 180 / math.pi - 120

def pid_control(setpointX, setpointY):
    """
    Takes in an X and Y setpoint/position and moves the ball to that position.
    """
    global detected
    point = read_touch_coordinates()
    if point is not None:
        detected = True
        p_x, p_y = point.x, point.y
        error = [0, 0]
        errorPrev = [0, 0]
        integr = [0, 0]
        deriv = [0, 0]
        out = [0, 0]
        pos = [0, 0, 0]

        for i in range(2):
            errorPrev[i] = error[i]
            error[i] = (Xoffset - p_x - setpointX) if i == 0 else (Yoffset - p_y - setpointY)
            integr[i] += error[i] + errorPrev[i]
            deriv[i] = error[i] - errorPrev[i]
            deriv[i] = 0 if math.isnan(deriv[i]) or math.isinf(deriv[i]) else deriv[i]
            out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i]
            out[i] = max(min(out[i], 0.25), -0.25)

        for i in range(3):
            speedPrev[i] = speed[i]
            speed[i] = abs(speed[i] - pos[i]) * ks
            speed[i] = max(min(speed[i], speedPrev[i] + 200), speedPrev[i] - 200)
            speed[i] = max(min(speed[i], 1000), 0)

        print(f"X OUT = {out[0]}   Y OUT = {out[1]}   Speed A: {speed[0]}")
    else:
        time.sleep(0.01)
        point = read_touch_coordinates()
        if point is None:
            detected = False

    timeI = time.time()
    while time.time() - timeI < 0.02:
        move_to(4.25, -out[0], -out[1])

def balance_ball():
    """
    Main loop to balance the ball using PID and motor control.
    """
    try:
        while True:
            pid_control(0, 0)
            time.sleep(0.01)  # Update cycle delay (10 ms)
    except KeyboardInterrupt:
        print("Exiting program...")
        # Move all motors CCW 100 steps
        for motor in MOTOR_PINS.keys():
            move_motor(motor, 100, False)
    finally:
        GPIO.cleanup()

# --------------------------------------------------------------------------------------------
if __name__ == "__main__":
    # Centering motors before starting
    print("Centering motors...")
    for _ in range(300):  # Arbitrary 300 steps to center
        move_motor('motor1', 1, True)
        move_motor('motor2', 1, True)
        move_motor('motor3', 1, True)
    print("Motors centered. Starting balance loop...")

    balance_ball()