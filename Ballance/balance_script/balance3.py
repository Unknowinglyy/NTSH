import sys
import os
import time
import math
from gpiozero import OutputDevice
from touchScreenBasicCoordOutput import *

# Add the root directory to the sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Motor Pins
MOTOR_PINS = [
    {'step': 23, 'dir': 24},  # Motor A pins
    {'step': 20, 'dir': 21},  # Motor B pins
    {'step': 5, 'dir': 6},    # Motor C pins
]

# Motor movement parameters
ANGLE_ORIGINAL = 206.662752199
ANGLE_TO_STEP = 3200 / 360
SPEED_CONSTANT = 20

# PID Constants
KP = 4E-4
KI = 2E-6
KD = 7E-3

# Touch screen offsets
X_OFFSET = 500
Y_OFFSET = 500

# PID Control Variables
error = [0, 0]
error_prev = [0, 0]
integral = [0, 0]
derivative = [0, 0]
output = [0, 0]
detected = False

# Setup GPIO for Motors
steppers = [OutputDevice(pin['step']) for pin in MOTOR_PINS]
directions = [OutputDevice(pin['dir']) for pin in MOTOR_PINS]

class ThreeRPSManipulator:
    def __init__(self, param1, param2, param3, param4):
        self.param1 = param1
        self.param2 = param2
        self.param3 = param3
        self.param4 = param4

    def calculate_theta(self, leg_index, hz, nx, ny):
        # Simplified inverse kinematics calculation for each leg
        base_angle = math.atan2(ny, nx)
        return base_angle + (leg_index * 2 * math.pi / 3)

# Initialize the manipulator
manipulator = ThreeRPSManipulator(2, 3.125, 1.75, 3.669)

def move_motor(stepper, direction, steps):
    direction.off() if steps < 0 else direction.on()
    steps = abs(steps)

    for _ in range(steps):
        stepper.on()
        time.sleep(0.0003)  # Smoother movement
        stepper.off()
        time.sleep(0.0003)

def move_to_target(hz, nx, ny):
    global detected
    positions = [
        round((ANGLE_ORIGINAL - manipulator.calculate_theta(i, hz, nx, ny)) * ANGLE_TO_STEP)
        for i in range(3)
    ]

    # Move motors to the calculated positions
    for i in range(3):
        move_motor(steppers[i], directions[i], positions[i])

def pid_controller(setpoint_x, setpoint_y):
    global detected, error, error_prev, integral, derivative, output
    touch_data = read_touch_coordinates()

    if touch_data and touch_data.x is not None:
        detected = True
        for i in range(2):
            error_prev[i] = error[i]
            error[i] = (X_OFFSET - touch_data.x - setpoint_x) if i == 0 else (Y_OFFSET - touch_data.y - setpoint_y)
            integral[i] += error[i]
            derivative[i] = error[i] - error_prev[i]
            derivative[i] = 0 if math.isnan(derivative[i]) or math.isinf(derivative[i]) else derivative[i]
            output[i] = KP * error[i] + KI * integral[i] + KD * derivative[i]
            output[i] = max(min(output[i], 0.25), -0.25)

        print(f"X OUTPUT: {output[0]}, Y OUTPUT: {output[1]}")
    else:
        detected = False

    start_time = time.time()
    while time.time() - start_time < 0.02:
        move_to_target(4.25, -output[0], -output[1])

def main():
    try:
        print("Starting motor control...")
        while True:
            pid_controller(0, 0)
    except KeyboardInterrupt:
        print("Motor control interrupted.")
    finally:
        for stepper in steppers:
            stepper.close()
        for direction in directions:
            direction.close()
        print("GPIO cleaned up.")

if __name__ == "__main__":
    main()
