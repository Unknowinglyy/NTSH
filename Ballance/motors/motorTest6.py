from gpiozero import OutputDevice
import time
# ----------------------------------------------------------------------------------
# Motor 1
step_pin_1 = 23  
dir_pin_1  = 24 

# Motor 2
step_pin_2 = 20  
dir_pin_2  = 21 

# Motor 3
step_pin_3 = 5
dir_pin_3  = 6 

# ----------------------------------------------------------------------------------
# Motor movement parameters
test_steps = 200              # Number of steps to move in each direction
delay_time = 0.001          # Delay in seconds between steps
# ----------------------------------------------------------------------------------
# Setup GPIO
step_1      = OutputDevice(step_pin_1)
direction_1 = OutputDevice(dir_pin_1)

step_2      = OutputDevice(step_pin_2)
direction_2 = OutputDevice(dir_pin_2)

step_3      = OutputDevice(step_pin_3)
direction_3 = OutputDevice(dir_pin_3)
# ----------------------------------------------------------------------------------
def move_motor(step, direction, steps, delay, motor_name):
    # Move clockwise
    direction.on()
    print(f"{motor_name} moving CW {steps} steps")
    for _ in range(steps):
        step.on()
        time.sleep(delay)
        step.off()
        time.sleep(delay)

    # Move counterclockwise
    direction.off()
    print(f"{motor_name} moving CCW {steps} steps")
    for _ in range(steps):
        step.on()
        time.sleep(delay)
        step.off()
        time.sleep(delay)
# ----------------------------------------------------------------------------------
try:
    print("Starting motor test...")

    while True:
        # Test Motor 1
        move_motor(step_1, direction_1, test_steps, delay_time, "Motor 1")
        time.sleep(0.25)
        # Test Motor 2
        move_motor(step_2, direction_2, test_steps, delay_time, "Motor 2")
        time.sleep(0.25)
        # Test Motor 3
        move_motor(step_3, direction_3, test_steps, delay_time, "Motor 3")
        time.sleep(0.25)

except KeyboardInterrupt:
    print("Motor test interrupted.")

finally:
    step_1.close()
    direction_1.close()
    step_2.close()
    direction_2.close()
    step_3.close()
    direction_3.close()
    print("GPIO cleaned up.")
