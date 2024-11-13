from gpiozero import OutputDevice
import time

# Pin definitions
step_pin = 23  # Pin connected to STEP on TMC2208
dir_pin = 24   # Pin connected to DIR on TMC2208

step_pin2 = 20  # Pin connected to STEP on 2nd TMC2208
dir_pin2 = 21   # Pin connected to DIR on 2nd TMC2208

step_pin3 = 5 # Pin connected to STEP on 3rd TMC2208
dir_pin3 = 6  # Pin connected to DIR on 3rd TMC2208

# Motor movement parameters
test_steps = 200              # Number of steps to move in each direction
delay_time = 0.005          # Delay in seconds between steps

# Setup GPIO
#motor 1
step = OutputDevice(step_pin)
direction = OutputDevice(dir_pin)

#motor 2
step2 = OutputDevice(step_pin2)
direction2 = OutputDevice(dir_pin2)

#motor 3
step3 = OutputDevice(step_pin3)
direction3 = OutputDevice(dir_pin3)

try:
    print("Starting motor test...")

    while True: 
        # 2 motors moving clockwise, 1 motor moving counter-clockwise
        direction.on()
        direction2.off()
        direction3.on()
        print("moving clockwise") 
        for _ in range(test_steps):   
            step.on()
            step2.on()
            step3.on()
            time.sleep(delay_time)  # Adjust for speed
            step.off()
            step2.off()
            step3.off()
            time.sleep(delay_time)  # Adjust for speed
        print("just moved clockwise")

        # 2 motors moving counter-clockwise, 1 motor moving clockwise
        direction.off()
        direction2.on()
        direction3.off()
        print("moving counter-clockwise")
        for _ in range(test_steps):
            step.on()
            step2.on()
            step3.on()
            time.sleep(delay_time)  # Adjust for speed
            step.off()
            step2.off()
            step3.off()
            time.sleep(delay_time)  # Adjust for speed
        print("just moved counter-clockwise")

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
