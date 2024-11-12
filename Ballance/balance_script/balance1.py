import time
import math
from gpiozero import OutputDevice
from touchScreenBasicCoordOutput import read_touch_coordinates, Point

# Motor Pins
step_pin_A = 23  # Pin connected to STEP on TMC2208 for Motor A (3685, 200)
dir_pin_A = 24   # Pin connected to DIR on TMC2208 for Motor A

step_pin_B = 5   # Pin connected to STEP on TMC2208 for Motor B (3685, 3820)
dir_pin_B = 6    # Pin connected to DIR on TMC2208 for Motor B

step_pin_C = 20  # Pin connected to STEP on TMC2208 for Motor C (halfway on x-axis between 390, 200 and 390, 3820)
dir_pin_C = 21   # Pin connected to DIR on TMC2208 for Motor C

# Motor movement parameters
angOrig = 206.662752199
angToStep = 800 / 360  # Steps per degree
ks = 20  # Speed amplifying constant

# PID variables
kp = 1E-3  # Increased proportional gain
ki = 5E-6  # Adjusted integral gain
kd = 1E-2  # Adjusted derivative gain
error = [0, 0]
errorPrev = [0, 0]
integr = [0, 0]
deriv = [0, 0]
out = [0, 0]
detected = False

# Touch screen offsets (center coordinates)
Xoffset = 2040
Yoffset = 2010

# Setup GPIO
stepperA = OutputDevice(step_pin_A)
directionA = OutputDevice(dir_pin_A)

stepperB = OutputDevice(step_pin_B)
directionB = OutputDevice(dir_pin_B)

stepperC = OutputDevice(step_pin_C)
directionC = OutputDevice(dir_pin_C)

# Store initial positions and current positions
initial_positions = [0, 0, 0]
current_positions = [0, 0, 0]

class Machine:
    def __init__(self, d, e, f, g):
        self.d = d
        self.e = e
        self.f = f
        self.g = g

    def theta(self, i, hz, nx, ny):
        # Simplified inverse kinematics calculation
        if i == 0:
            return math.atan2(ny, nx)
        elif i == 1:
            return math.atan2(ny, nx) + 2 * math.pi / 3
        elif i == 2:
            return math.atan2(ny, nx) + 4 * math.pi / 3
        else:
            return 0

# Initialize the machine
machine = Machine(2, 3.125, 1.75, 3.669291339)

def move_motor(step, direction, steps, motor_name):
    if steps > 0:
        direction.on()
        print(f"{motor_name} moving CW {steps} steps")
    else:
        direction.off()
        steps = -steps
        print(f"{motor_name} moving CCW {steps} steps")
    for _ in range(steps):
        step.on()
        time.sleep(0.0005)  # Reduced delay to make motors move faster
        step.off()
        time.sleep(0.0005)  # Reduced delay to make motors move faster

def move_to(hz, nx, ny):
    global detected
    if detected:
        pos = [round((angOrig - machine.theta(i, hz, nx, ny)) * angToStep) for i in range(3)]
        # Move motors to the calculated positions incrementally
        for i in range(3):
            steps = pos[i] - current_positions[i]
            if steps != 0:
                move_motor([stepperA, stepperB, stepperC][i], [directionA, directionB, directionC][i], steps, f"Motor {chr(65 + i)}")
                current_positions[i] = pos[i]
    else:
        # Revert to initial positions if the ball is not detected
        for i in range(3):
            steps = initial_positions[i] - current_positions[i]
            if steps != 0:
                move_motor([stepperA, stepperB, stepperC][i], [directionA, directionB, directionC][i], steps, f"Motor {chr(65 + i)}")
                current_positions[i] = initial_positions[i]

def pid(setpointX, setpointY):
    global detected, error, errorPrev, integr, deriv, out
    p = read_touch_coordinates()
    if p is not None and p.x is not None:
        detected = True
        for i in range(2):
            errorPrev[i] = error[i]
            error[i] = (i == 0) * (Xoffset - p.x - setpointX) + (i == 1) * (Yoffset - p.y - setpointY)
            integr[i] += error[i] + errorPrev[i]
            deriv[i] = error[i] - errorPrev[i]
            deriv[i] = 0 if math.isnan(deriv[i]) or math.isinf(deriv[i]) else deriv[i]
            out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i]
            out[i] = max(min(out[i], 0.25), -0.25)
        # print(f"X OUT = {out[0]}   Y OUT = {out[1]}")
    else:
        print("No ball detected")
        detected = False

    timeI = time.time()
    while time.time() - timeI < 0.02:
        move_to(4.25, -out[0], -out[1])

def main():
    global detected, initial_positions, current_positions
    try:
        print("Starting motor test...")
        # Store initial positions
        initial_positions = [0, 0, 0]
        current_positions = [0, 0, 0]
        while True:
            pid(2040, 2010)  # Setpoint is the center coordinate
    except KeyboardInterrupt:
        print("Motor test interrupted.")
        for i in range(3):
            steps = initial_positions[i] - current_positions[i]
            if steps != 0:
                move_motor([stepperA, stepperB, stepperC][i], [directionA, directionB, directionC][i], steps, f"Motor {chr(65 + i)}")
                current_positions[i] = initial_positions[i]
    finally:
        stepperA.close()
        stepperB.close()
        stepperC.close()
        directionA.close()
        directionB.close()
        directionC.close()
        print("GPIO cleaned up.")

if __name__ == "__main__":
    main()