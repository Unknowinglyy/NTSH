import time
from gpiozero import OutputDevice

# Motor Pins
step_pin_A = 23  # Pin connected to STEP on TMC2208 for Motor A
dir_pin_A = 24   # Pin connected to DIR on TMC2208 for Motor A

step_pin_B = 20  # Pin connected to STEP on TMC2208 for Motor B
dir_pin_B = 21   # Pin connected to DIR on TMC2208 for Motor B

step_pin_C = 5   # Pin connected to STEP on TMC2208 for Motor C
dir_pin_C = 6    # Pin connected to DIR on TMC2208 for Motor C

# Setup GPIO
stepperA = OutputDevice(step_pin_A)
directionA = OutputDevice(dir_pin_A)

stepperB = OutputDevice(step_pin_B)
directionB = OutputDevice(dir_pin_B)

stepperC = OutputDevice(step_pin_C)
directionC = OutputDevice(dir_pin_C)

def move_motor(step, direction, steps):
    if steps > 0:
        direction.on()
    else:
        direction.off()
        steps = -steps
    for _ in range(steps):
        step.on()
        time.sleep(0.001)  # Adjust delay as needed
        step.off()
        time.sleep(0.001)  # Adjust delay as needed

def test_motors():
    try:
        print("Testing Motor A...")
        move_motor(stepperA, directionA, 200)  # Move 200 steps clockwise
        time.sleep(1)
        move_motor(stepperA, directionA, -200)  # Move 200 steps counterclockwise
        time.sleep(1)

        print("Testing Motor B...")
        move_motor(stepperB, directionB, 200)  # Move 200 steps clockwise
        time.sleep(1)
        move_motor(stepperB, directionB, -200)  # Move 200 steps counterclockwise
        time.sleep(1)

        print("Testing Motor C...")
        move_motor(stepperC, directionC, 200)  # Move 200 steps clockwise
        time.sleep(1)
        move_motor(stepperC, directionC, -200)  # Move 200 steps counterclockwise
        time.sleep(1)

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
    test_motors()