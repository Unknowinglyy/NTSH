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
kp = 1E-3
ki = 5E-6
kd = 1E-2
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

# Store initial and current positions
initial_positions = [0, 0, 0]
current_positions = [0, 0, 0]

class Machine:
    def __init__(self, d, e, f, g):
        self.d = d
        self.e = e
        self.f = f
        self.g = g

    def theta(self, i, hz, nx, ny):
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

def move_motor(step, direction, steps, current_position, motor_name):
    steps = max(-current_position, min(steps, 200 - current_position))
    if steps > 0:
        direction.on()
        print(f"{motor_name} moving CW {steps} steps (current position: {current_position + steps}/200)")
    else:
        direction.off()
        print(f"{motor_name} moving CCW {steps} steps (current position: {current_position + steps}/200)")
    
    for _ in range(abs(steps)):
        step.on()
        time.sleep(0.0005)
        step.off()
        time.sleep(0.0005)
    
    return current_position + steps if direction.value else current_position - steps

def move_to(hz, nx, ny):
    global detected, current_positions
    if detected:
        pos = [round((angOrig - machine.theta(i, hz, nx, ny)) * angToStep) for i in range(3)]
        pos = [max(min(p, 200), 0) for p in pos]
        for i in range(3):
            steps = pos[i] - current_positions[i]
            if steps != 0:
                current_positions[i] = move_motor(
                    [stepperA, stepperB, stepperC][i],
                    [directionA, directionB, directionC][i],
                    steps, current_positions[i],
                    f"Motor {chr(65 + i)}"
                )
    else:
        for i in range(3):
            steps = initial_positions[i] - current_positions[i]
            if steps != 0:
                current_positions[i] = move_motor(
                    [stepperA, stepperB, stepperC][i],
                    [directionA, directionB, directionC][i],
                    steps, current_positions[i],
                    f"Motor {chr(65 + i)}"
                )

def pid(setpointX, setpointY):
    global detected, error, errorPrev, integr, deriv, out
    p = read_touch_coordinates()
    if p is not None and p.x is not None:
        detected = True
        for i in range(2):
            errorPrev[i] = error[i]
            error[i] = ((Xoffset - p.x - setpointX) if i == 0 else (Yoffset - p.y - setpointY))
            integr[i] += error[i]
            deriv[i] = error[i] - errorPrev[i]
            deriv[i] = 0 if math.isnan(deriv[i]) or math.isinf(deriv[i]) else deriv[i]
            out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i]
            out[i] = max(min(out[i], 0.25), -0.25)
    else:
        detected = False

    timeI = time.time()
    while time.time() - timeI < 0.02:
        move_to(4.25, -out[0], -out[1])

def main():
    global detected, initial_positions, current_positions
    try:
        initial_positions = [0, 0, 0]
        current_positions = [0, 0, 0]
        while True:
            pid(2025, 2045)
    except KeyboardInterrupt:
        for i in range(3):
            steps = initial_positions[i] - current_positions[i]
            if steps != 0:
                current_positions[i] = move_motor(
                    [stepperA, stepperB, stepperC][i],
                    [directionA, directionB, directionC][i],
                    steps, current_positions[i],
                    f"Motor {chr(65 + i)}"
                )
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
