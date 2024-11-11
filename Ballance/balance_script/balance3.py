import time
import math
from gpiozero import OutputDevice

# Motor Pins
MOTOR_PINS = [
    {'step': 23, 'dir': 24},  # Motor A pins
    {'step': 20, 'dir': 21},  # Motor B pins
    {'step': 5, 'dir': 6},    # Motor C pins
]

# Motor movement parameters
ANGLE_TO_STEP = 3200 / 360  # Steps per degree for 1/16 microstepping

# Setup GPIO for Motors
steppers = [OutputDevice(pin['step']) for pin in MOTOR_PINS]
directions = [OutputDevice(pin['dir']) for pin in MOTOR_PINS]

# Step counters for each motor
step_counts = [0, 0, 0]

# Function to update the step count and direction
def move_motor(motor_index, steps):
    direction = 1 if steps > 0 else -1
    directions[motor_index].value = steps > 0
    for _ in range(abs(steps)):
        steppers[motor_index].on()
        time.sleep(0.001)  # Pulse width
        steppers[motor_index].off()
        time.sleep(0.001)
        step_counts[motor_index] += direction

# Function to get the current angle of the motor
def get_motor_angle(motor_index):
    return step_counts[motor_index] / ANGLE_TO_STEP

# Main program to display motor angles in real time
if __name__ == "__main__":
    try:
        while True:
            # Display the current angle for each motor
            angles = [get_motor_angle(i) for i in range(3)]
            print(f"Motor A Angle: {angles[0]:.2f} degrees")
            print(f"Motor B Angle: {angles[1]:.2f} degrees")
            print(f"Motor C Angle: {angles[2]:.2f} degrees")
            print("-------------------------------")
            time.sleep(0.5)  # Update every 500ms
    except KeyboardInterrupt:
        print("Program terminated.")
