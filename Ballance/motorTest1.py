import RPi.GPIO as GPIO
import time

# Pin definitions
step_pin = 17  # Pin connected to STEP on TMC2208
dir_pin = 27   # Pin connected to DIR on TMC2208
en_pin = 22    # Pin connected to EN on TMC2208

# Motor movement parameters
steps_per_revolution = 200  # Change if your motor has a different step count
test_steps = 100            # Number of steps to move in each direction
delay_time = 0.01           # Delay in seconds between steps

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(step_pin, GPIO.OUT)
GPIO.setup(dir_pin, GPIO.OUT)
GPIO.setup(en_pin, GPIO.OUT)

# Enable the driver
GPIO.output(en_pin, GPIO.LOW)

try:
    print("Starting motor test...")

    while True:
        # Move clockwise
        GPIO.output(dir_pin, GPIO.HIGH)
        for _ in range(test_steps):
            GPIO.output(step_pin, GPIO.HIGH)
            time.sleep(delay_time)  # Adjust for speed
            GPIO.output(step_pin, GPIO.LOW)
            time.sleep(delay_time)  # Adjust for speed
        
        print(f"Moved {test_steps} steps clockwise.")
        
        # Delay before changing direction
        time.sleep(1)

        # Move counterclockwise
        GPIO.output(dir_pin, GPIO.LOW)
        for _ in range(test_steps):
            GPIO.output(step_pin, GPIO.HIGH)
            time.sleep(delay_time)  # Adjust for speed
            GPIO.output(step_pin, GPIO.LOW)
            time.sleep(delay_time)  # Adjust for speed
        
        print(f"Moved {test_steps} steps counterclockwise.")
        
        # Delay before repeating
        time.sleep(1)

except KeyboardInterrupt:
    print("Motor test interrupted.")

finally:
    # Cleanup GPIO settings
    GPIO.cleanup()