from gpiozero import OutputDevice
import time
import math
import threading

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
wave_frequency = 1         # Frequency of the wave motion
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
        if math.sin(angle) > 0:
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

        # Create threads for each motor
        thread1 = threading.Thread(target=move_motor, args=(step, direction, test_steps, delay_time, 0))
        thread2 = threading.Thread(target=move_motor, args=(step2, direction2, test_steps, delay_time, 2 * math.pi / 3))
        thread3 = threading.Thread(target=move_motor, args=(step3, direction3, test_steps, delay_time, 4 * math.pi / 3))

        # Start all threads
        thread1.start()
        thread2.start()
        thread3.start()

        # Wait for all threads to complete
        thread1.join()
        thread2.join()
        thread3.join()

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