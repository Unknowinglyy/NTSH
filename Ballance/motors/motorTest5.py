from gpiozero import OutputDevice
import time

# Pin definitions for Motor 1
step_pin1 = 23  # Pin connected to STEP on TMC2208 for Motor 1
dir_pin1 = 24   # Pin connected to DIR on TMC2208 for Motor 1

# Pin definitions for Motor 2
step_pin2 = 20  # Pin connected to STEP on TMC2208 for Motor 2
dir_pin2 = 21   # Pin connected to DIR on TMC2208 for Motor 2

# Pin definitions for Motor 3
step_pin3 = 5   # Pin connected to STEP on TMC2208 for Motor 3
dir_pin3 = 6    # Pin connected to DIR on TMC2208 for Motor 3

# Motor movement parameters
test_steps = 200              # Number of steps to move in each direction
delay_time = 0.001            # Delay in seconds between steps

# Setup GPIO for each motor
step1 = OutputDevice(step_pin1)
direction1 = OutputDevice(dir_pin1)

step2 = OutputDevice(step_pin2)
direction2 = OutputDevice(dir_pin2)

step3 = OutputDevice(step_pin3)
direction3 = OutputDevice(dir_pin3)

try:
    print("Starting motor test...")

    while True: 
        # Move each motor clockwise
        direction1.on()
        direction2.on()
        direction3.on()
        print("Moving motors clockwise")
        
        for _ in range(test_steps):   
            step1.on()
            step2.on()
            step3.on()
            time.sleep(delay_time)
            step1.off()
            step2.off()
            step3.off()
            time.sleep(delay_time)
        
        print("Motors moved clockwise")

        # Move each motor counterclockwise
        direction1.off()
        direction2.off()
        direction3.off()
        print("Moving motors counter-clockwise")
        
        for _ in range(test_steps):
            step1.on()
            step2.on()
            step3.on()
            time.sleep(delay_time)
            step1.off()
            step2.off()
            step3.off()
            time.sleep(delay_time)
        
        print("Motors moved counter-clockwise")

except KeyboardInterrupt:
    print("Motor test interrupted.")

finally:
    step1.close()
    direction1.close()
    step2.close()
    direction2.close()
    step3.close()
    direction3.close()
    print("GPIO cleaned up.")
