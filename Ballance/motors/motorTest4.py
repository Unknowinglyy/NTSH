from gpiozero import OutputDevice
import time
import math

# ----------------------------------------------------------------------------------
# Motor Pins
step_pin = 23  # Pin connected to STEP on TMC2208
dir_pin = 24   # Pin connected to DIR on TMC2208

step_pin2 = 20  # Pin connected to STEP on 2nd TMC2208
dir_pin2 = 21   # Pin connected to DIR on 2nd TMC2208

step_pin3 = 5 # Pin connected to STEP on 3rd TMC2208
dir_pin3 = 6  # Pin connected to DIR on 3rd TMC2208
# ----------------------------------------------------------------------------------
# Motor movement parameters
test_steps = 200              # Number of steps to move in each direction
delay_time = 0.005          # Delay in seconds between steps
wave_frequency = 5         # Frequency of the wave motion
# ----------------------------------------------------------------------------------

# Setup GPIO
step = OutputDevice(step_pin)
direction = OutputDevice(dir_pin)

step2 = OutputDevice(step_pin2)
direction2 = OutputDevice(dir_pin2)

step3 = OutputDevice(step_pin3)
direction3 = OutputDevice(dir_pin3)
# ----------------------------------------------------------------------------------

def move_motor(step, direction, steps, delay, phase_shift):
    for i in range(steps):
        angle = 2 * math.pi * wave_frequency * i / steps + phase_shift
        print(angle)
        if math.sin(angle) > 0:
            #clockwise
            direction.on()
        else:
            direction.off()
        step.on()
        time.sleep(delay)
        step.off()
        time.sleep(delay)

try:
    print("Starting motor test...")

    while True:
        print("Moving in circular motion")
        move_motor(step, direction, test_steps, delay_time, 0)
        move_motor(step2, direction2, test_steps, delay_time, 2 * math.pi / 3)
        move_motor(step3, direction3, test_steps, delay_time, 4 * math.pi / 3)
        print("Completed one cycle of circular motion")

except KeyboardInterrupt:
    print("Motor test interrupted.")

finally:
    step.close()
    step2.close()
    step3.close()
    direction.close()
    direction2.close()
    direction3.close()
    print("GPIO cleaned up.")