from gpiozero import OutputDevice
import time

# Pin definitions
step_pin = 23  # Pin connected to STEP on TMC2208
dir_pin = 24   # Pin connected to DIR on TMC2208

step_pin2 = 20  # Pin connected to STEP on 2nd TMC2208
dir_pin2 = 21   # Pin connected to DIR on 2nd TMC2208

# Motor movement parameters
test_steps = 200              # Number of steps to move in each direction
delay_time = 0.001           # Delay in seconds between steps

# Setup GPIO
step = OutputDevice(step_pin)
direction = OutputDevice(dir_pin)

step2 = OutputDevice(step_pin2)
direction2 = OutputDevice(dir_pin2)

try:
    print("Starting motor test...")

    while True: 
        # Move clockwise
        direction.on()
        direction2.on()
        print("moving clockwise") 
        for _ in range(test_steps):   
            step.on()
            step2.on()
            time.sleep(delay_time)  # Adjust for speed
            step.off()
            step2.off()
            time.sleep(delay_time)  # Adjust for speed
        print("just moved clockwise")

        # Move counterclockwise
        direction.off()
        direction2.off()
        print("moving counter-clockwise")
        for _ in range(test_steps):
            step.on()
            step2.on()
            time.sleep(delay_time)  # Adjust for speed
            step.off()
            step2.off()
            time.sleep(delay_time)  # Adjust for speed
        print("just moved counter-clockwise")

except KeyboardInterrupt:
    print("Motor test interrupted.")

finally:
    step.close()
    step2.close()
    direction.close()
    direction2.close()
    print("GPIO cleaned up.")
