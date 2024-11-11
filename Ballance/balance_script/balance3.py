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
ANGLE_TO_STEP = 3200 / 360

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
    def __init__(self, d, e, f, g):
        self.d = d
        self.e = e
        self.f = f
        self.g = g

    def calculate_theta(self, leg, hz, nx, ny):
        # Create unit normal vector
        nmag = math.sqrt(nx ** 2 + ny ** 2 + 1)
        nx /= nmag
        ny /= nmag
        nz = 1 / nmag

        # Calculate angle for each leg
        if leg == 0:  # Leg A
            y = self.d + (self.e / 2) * (1 - (nx ** 2 + 3 * nz ** 2 + 3 * nz) / (nz + 1 - nx ** 2 + (nx ** 4 - 3 * nx ** 2 * ny ** 2) / ((nz + 1) * (nz + 1 - nx ** 2))))
            z = hz + self.e * ny
            mag = math.sqrt(y ** 2 + z ** 2)
            angle = math.acos(y / mag) + math.acos((mag ** 2 + self.f ** 2 - self.g ** 2) / (2 * mag * self.f))
        elif leg == 1:  # Leg B
            x = (math.sqrt(3) / 2) * (self.e * (1 - (nx ** 2 + math.sqrt(3) * nx * ny) / (nz + 1)) - self.d)
            y = x / math.sqrt(3)
            z = hz - (self.e / 2) * (math.sqrt(3) * nx + ny)
            mag = math.sqrt(x ** 2 + y ** 2 + z ** 2)
            angle = math.acos((math.sqrt(3) * x + y) / (-2 * mag)) + math.acos((mag ** 2 + self.f ** 2 - self.g ** 2) / (2 * mag * self.f))
        elif leg == 2:  # Leg C
            x = (math.sqrt(3) / 2) * (self.d - self.e * (1 - (nx ** 2 - math.sqrt(3) * nx * ny) / (nz + 1)))
            y = -x / math.sqrt(3)
            z = hz + (self.e / 2) * (math.sqrt(3) * nx - ny)
            mag = math.sqrt(x ** 2 + y ** 2 + z ** 2)
            angle = math.acos((math.sqrt(3) * x - y) / (2 * mag)) + math.acos((mag ** 2 + self.f ** 2 - self.g ** 2) / (2 * mag * self.f))
        else:
            raise ValueError("Invalid leg index")

        return angle * (180 / math.pi)  # Convert angle to degrees

# Initialize the manipulator
manipulator = ThreeRPSManipulator(2, 3.125, 1.75, 3.669)

def move_motor(stepper, direction, steps):
    direction.off() if steps < 0 else direction.on()
    steps = abs(steps) // 5

    for _ in range(steps):
        stepper.on()
        time.sleep(0.005)  # Smoother movement
        stepper.off()
        time.sleep(0.005)

def move_to_target(hz, nx, ny):
    for leg_index in range(3):
        # Calculate theta for each leg
        theta = manipulator.calculate_theta(leg_index, hz, nx, ny)
        position = round(theta * ANGLE_TO_STEP)  # Convert angle to steps

        # Move the selected motor to the calculated position
        move_motor(steppers[leg_index], directions[leg_index], position)

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