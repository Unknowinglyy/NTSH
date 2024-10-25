from gpiozero import OutputDevice
import time

# Pin definitions
step_pin = 23  # Pin connected to STEP on TMC2208
dir_pin = 24   # Pin connected to DIR on TMC2208

# Motor movement parameters
test_steps = 200            # Number of steps to move in each direction (for a full revolution)
delay_time = 0.001         # Delay in seconds between steps

# Setup GPIO
step = OutputDevice(step_pin)
direction = OutputDevice(dir_pin)

try:
    print("Starting motor test...")

    while True: 
        # Move clockwise
        direction.on()
        print("Moving clockwise") 
        for _ in range(test_steps):   
            step.on()
            time.sleep(delay_time)  # Adjust for speed
            step.off()
            time.sleep(delay_time)  # Adjust for speed
            
        print("Just moved clockwise")

        # Move counterclockwise
        direction.off()
        print("Moving counter-clockwise")
        for _ in range(test_steps):
            step.on()
            time.sleep(delay_time)  # Adjust for speed
            step.off()
            time.sleep(delay_time)  # Adjust for speed
            
        print("Just moved counter-clockwise")

except KeyboardInterrupt:
    print("Motor test interrupted.")

finally:
    step.close()
    direction.close()
    print("GPIO cleaned up.")
