from gpiozero import OutputDevice
import time

# Define your step and direction pins
step_pin = 23  # Example GPIO pin number
dir_pin = 24   # Example GPIO pin number

# Define the number of steps and delay time
test_steps = 100  # Example number of steps
delay_time = 0.01  # Example delay time in seconds

# Initialize the step and direction pins
step = OutputDevice(step_pin)
direction = OutputDevice(dir_pin)

try:
    print("Starting motor test...")

    # Move counter clockwise
    direction.off()
    print("moving clockwise")
    for _ in range(test_steps):
        step.on()
        time.sleep(delay_time)  # Adjust for speed
        step.off()
        time.sleep(delay_time)  # Adjust for speed

    print("just moved counter clockwise")

except KeyboardInterrupt:
    print("Motor test interrupted.")

finally:
    step.close()
    direction.close()
    print("GPIO cleaned up.")