from gpiozero import OutputDevice
import time

# Pin definitions
step_pin = 20  # Pin connected to STEP on TMC2208
dir_pin = 21   # Pin connected to DIR on TMC2208

# Motor movement parameters
test_steps = 200              # Number of steps to move in each direction
delay_time = 0.005          # Delay in seconds between steps

# Setup GPIO
step = OutputDevice(step_pin)
direction = OutputDevice(dir_pin)

try:
    print("Starting motor test...")

    while True: 
        # Move clockwise
        direction.on()
        print("moving clockwise") 
        for _ in range(test_steps):   
            step.on()
            time.sleep(delay_time)  # Adjust for speed
            step.off()
            time.sleep(delay_time)  # Adjust for speed
        print("just moved clockwise")

        # Move counterclockwise
        direction.off()
        print("moving counter-clockwise")
        for _ in range(test_steps):
            step.on()
            time.sleep(delay_time)  # Adjust for speed
            step.off()
            time.sleep(delay_time)  # Adjust for speed
        print("just moved counter-clockwise")

except KeyboardInterrupt:
    print("Motor test interrupted.")

finally:
    step.close()
    direction.close()
    print("GPIO cleaned up.")
