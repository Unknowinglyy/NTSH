import time
import math
from gpiozero import OutputDevice
from touchScreenBasicCoordOutput import read_touch_coordinates, Point
# -----------------------------------------------------------
# Motor Pins
step_pin = 23  # Pin connected to STEP on TMC2208
dir_pin = 24   # Pin connected to DIR on TMC2208

step_pin2 = 20  # Pin connected to STEP on 2nd TMC2208
dir_pin2 = 21   # Pin connected to DIR on 2nd TMC2208

step_pin3 = 5  # Pin connected to STEP on 3rd TMC2208
dir_pin3 = 6   # Pin connected to DIR on 3rd TMC2208

# Motor movement parameters
angOrig = 206.662752199
angToStep = 3200 / 360
ks = 20  # Speed amplifying constant

# PID variables
kp = 4E-4
ki = 2E-6
kd = 7E-3
error = [0, 0]
errorPrev = [0, 0]
integr = [0, 0]
deriv = [0, 0]
out = [0, 0]
detected = False

# Touch screen offsets
Xoffset = 2025
Yoffset = 2045

# Setup GPIO
stepperA = OutputDevice(step_pin)
directionA = OutputDevice(dir_pin)

stepperB = OutputDevice(step_pin2)
directionB = OutputDevice(dir_pin2)

stepperC = OutputDevice(step_pin3)
directionC = OutputDevice(dir_pin3)

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

def move_motor(step, direction, steps):
    if steps > 0:
        direction.on()
    else:
        direction.off()
        steps = -steps
    for _ in range(steps):
        step.on()
        time.sleep(0.001)
        step.off()
        time.sleep(0.001)

def move_to(hz, nx, ny):
    global detected
    if detected:
        pos = [round((angOrig - machine.theta(i, hz, nx, ny)) * angToStep) for i in range(3)]
        # Constrain positions to prevent moving past range of motion
        pos = [max(min(p, 800), 0) for p in pos]
        # Move motors to the calculated positions
        move_motor(stepperA, directionA, pos[0])
        move_motor(stepperB, directionB, pos[1])
        move_motor(stepperC, directionC, pos[2])
    else:
        pos = [round((angOrig - machine.theta(i, hz, 0, 0)) * angToStep) for i in range(3)]
        pos = [max(min(p, 3200), 0) for p in pos]
        move_motor(stepperA, directionA, pos[0])
        move_motor(stepperB, directionB, pos[1])
        move_motor(stepperC, directionC, pos[2])

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
    global detected
    try:
        print("Starting motor test...")
        while True:
            pid(2025, 2045)  # Setpoint is the center coordinate
    except KeyboardInterrupt:
        print("Motor test interrupted.")
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