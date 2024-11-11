from gpiozero import OutputDevice
import time
import math
import threading
from touchScreenBasicCoordOutput import read_touch_coordinates

# Motor Pins
step_pin = 23  # Pin connected to STEP on TMC2208
dir_pin = 24   # Pin connected to DIR on TMC2208

step_pin2 = 20  # Pin connected to STEP on 2nd TMC2208
dir_pin2 = 21   # Pin connected to DIR on 2nd TMC2208

step_pin3 = 5  # Pin connected to STEP on 3rd TMC2208
dir_pin3 = 6   # Pin connected to DIR on 3rd TMC2208

# Motor movement parameters
test_steps = 200              # Number of steps to move in each direction
delay_time = 0.005            # Delay in seconds between steps
wave_frequency = 1            # Frequency of the wave motion

# Setup GPIO
stepperA = OutputDevice(step_pin)
directionA = OutputDevice(dir_pin)

stepperB = OutputDevice(step_pin2)
directionB = OutputDevice(dir_pin2)

stepperC = OutputDevice(step_pin3)
directionC = OutputDevice(dir_pin3)

# PID variables
kp = 4E-4
ki = 2E-6
kd = 7E-3
error = [0, 0]
errorPrev = [0, 0]
integr = [0, 0]
deriv = [0, 0]
out = [0, 0]
angOrig = 206.662752199
angToStep = 3200 / 360
detected = False

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

def move_motor(step, direction, steps, delay, phase_shift):
    for i in range(steps):
        angle = 2 * math.pi * wave_frequency * i / steps + phase_shift
        if math.sin(angle) > 0:
            direction.on()
        else:
            direction.off()
        step.on()
        time.sleep(delay)
        step.off()
        time.sleep(delay)

def move_to(hz, nx, ny):
    global detected
    if detected:
        pos = [round((angOrig - machine.theta(i, hz, nx, ny)) * angToStep) for i in range(3)]
        # Assuming setMaxSpeed and setAcceleration are methods to control speed and acceleration
        # These methods need to be implemented or replaced with appropriate logic
        stepperA.setMaxSpeed(speed[0])
        stepperB.setMaxSpeed(speed[1])
        stepperC.setMaxSpeed(speed[2])
        stepperA.setAcceleration(speed[0] * 30)
        stepperB.setAcceleration(speed[1] * 30)
        stepperC.setAcceleration(speed[2] * 30)
        stepperA.moveTo(pos[0])
        stepperB.moveTo(pos[1])
        stepperC.moveTo(pos[2])
        stepperA.run()
        stepperB.run()
        stepperC.run()
    else:
        pos = [round((angOrig - machine.theta(i, hz, 0, 0)) * angToStep) for i in range(3)]
        stepperA.setMaxSpeed(800)
        stepperB.setMaxSpeed(800)
        stepperC.setMaxSpeed(800)
        steppers.moveTo(pos)
        steppers.run()

def pid(setpointX, setpointY):
    global detected, error, errorPrev, integr, deriv, out
    p = read_touch_coordinates()
    if p.x != 0:
        detected = True
        for i in range(2):
            errorPrev[i] = error[i]
            error[i] = (i == 0) * (Xoffset - p.x - setpointX) + (i == 1) * (Yoffset - p.y - setpointY)
            integr[i] += error[i] + errorPrev[i]
            deriv[i] = error[i] - errorPrev[i]
            deriv[i] = 0 if math.isnan(deriv[i]) or math.isinf(deriv[i]) else deriv[i]
            out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i]
            out[i] = max(min(out[i], 0.25), -0.25)
        for i in range(3):
            speedPrev[i] = speed[i]
            speed[i] = (i == 0) * stepperA.currentPosition() + (i == 1) * stepperB.currentPosition() + (i == 2) * stepperC.currentPosition()
            speed[i] = abs(speed[i] - pos[i]) * ks
            speed[i] = max(min(speed[i], speedPrev[i] + 200), speedPrev[i] - 200)
            speed[i] = max(min(speed[i], 1000), 0)
        print(f"X OUT = {out[0]}   Y OUT = {out[1]}   Speed A: {speed[0]}")
    else:
        time.sleep(0.01)
        p = read_touch_coordinates()
        if p.x == 0:
            detected = False
    timeI = time.time()
    while time.time() - timeI < 0.02:
        move_to(4.25, -out[0], -out[1])

def main():
    global detected
    try:
        print("Starting motor test...")
        while True:
            pid(0, 0)
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