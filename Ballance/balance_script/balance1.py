import time
import math
from gpiozero import OutputDevice
from touchScreenBasicCoordOutput import read_touch_coordinates, Point

# Motor Pins
step_pin_A = 23  # Pin connected to STEP on TMC2208 for Motor A
dir_pin_A = 24   # Pin connected to DIR on TMC2208 for Motor A

step_pin_B = 20  # Pin connected to STEP on TMC2208 for Motor B
dir_pin_B = 21   # Pin connected to DIR on TMC2208 for Motor B

step_pin_C = 5   # Pin connected to STEP on TMC2208 for Motor C
dir_pin_C = 6    # Pin connected to DIR on TMC2208 for Motor C

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
Xoffset = 2025
Yoffset = 2045

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

def move_motor(step, direction, steps, current_position):
    if steps > 0:
        direction.on()
        if current_position + steps > 200:
            steps = 200 - current_position
    else:
        direction.off()
        if current_position + steps < -200:
            steps = -200 - current_position
        steps = -steps
    for _ in range(steps):
        step.on()
        time.sleep(0.0005)  # Reduced delay to make motors move faster
        step.off()
        time.sleep(0.0005)  # Reduced delay to make motors move faster
    return current_position + steps if direction.value else current_position - steps

def move_to(hz, nx, ny):
    global detected, current_positions
    if detected:
        pos = [round((angOrig - machine.theta(i, hz, nx, ny)) * angToStep) for i in range(3)]
        # Constrain positions to prevent moving past range of motion
        pos = [max(min(p, 200), -200) for p in pos]
        # Move motors to the calculated positions
        current_positions[0] = move_motor(stepperA, directionA, pos[0] - current_positions[0], current_positions[0])
        current_positions[1] = move_motor(stepperB, directionB, pos[1] - current_positions[1], current_positions[1])
        current_positions[2] = move_motor(stepperC, directionC, pos[2] - current_positions[2], current_positions[2])
    else:
        # Revert to initial positions if the ball is not detected
        current_positions[0] = move_motor(stepperA, directionA, initial_positions[0] - current_positions[0], current_positions[0])
        current_positions[1] = move_motor(stepperB, directionB, initial_positions[1] - current_positions[1], current_positions[1])
        current_positions[2] = move_motor(stepperC, directionC, initial_positions[2] - current_positions[2], current_positions[2])

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
        print(f"X OUT = {out[0]}   Y OUT = {out[1]}")
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
            pid(2025, 2045)  # Setpoint is the center coordinate
    except KeyboardInterrupt:
        print("Motor test interrupted.")
        move_motor(stepperA, directionA, initial_positions[0] - current_positions[0], current_positions[0])
        move_motor(stepperB, directionB, initial_positions[1] - current_positions[1], current_positions[1])
        move_motor(stepperC, directionC, initial_positions[2] - current_positions[2], current_positions[2])
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