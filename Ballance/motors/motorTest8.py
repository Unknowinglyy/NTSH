from gpiozero import OutputDevice
import time
import sys
import termios
import tty
import threading

# Pin definitions for Motor 1
step_pin_1 = 23  # Pin connected to STEP on TMC2208 for Motor 1
dir_pin_1  = 24  # Pin connected to DIR on TMC2208 for Motor 1

# Pin definitions for Motor 2
step_pin_2 = 20  # Pin connected to STEP on TMC2208 for Motor 2
dir_pin_2  = 21  # Pin connected to DIR on TMC2208 for Motor 2

# Pin definitions for Motor 3
step_pin_3 = 5   # Pin connected to STEP on TMC2208 for Motor 3
dir_pin_3  = 6   # Pin connected to DIR on TMC2208 for Motor 3

# Motor movement parameters
test_steps = 100              # Number of steps to move in each direction
delay_time = 0.0008            # Delay in seconds between steps

# Setup GPIO for each motor
step_1      = OutputDevice(step_pin_1)
direction_1 = OutputDevice(dir_pin_1)

step_2      = OutputDevice(step_pin_2)
direction_2 = OutputDevice(dir_pin_2)

step_3      = OutputDevice(step_pin_3)
direction_3 = OutputDevice(dir_pin_3)

def move_all_motors_cw(steps, delay):
    # Move all motors clockwise
    direction_1.on()
    direction_2.on()
    direction_3.on()
    print(f"All motors moving CW {steps} steps")
    for _ in range(steps):
        step_1.on()
        step_2.on()
        step_3.on()
        time.sleep(delay)
        step_1.off()
        step_2.off()
        step_3.off()
        time.sleep(delay)
    print("All motors just moved CW")

def move_all_motors_cww(steps, delay):
    # Move all motors clockwise
    direction_1.off()
    direction_2.off()
    direction_3.off()
    print(f"All motors moving CW {steps} steps")
    for _ in range(steps):
        step_1.on()
        step_2.on()
        step_3.on()
        time.sleep(delay)
        step_1.off()
        step_2.off()
        step_3.off()
        time.sleep(delay)
    print("All motors just moved CWW")


def move_motor(step, direction, steps, delay, motor_name):
    # Move up
    direction.on()
    print(f"{motor_name} moving up {steps} steps")
    for _ in range(steps):
        step.on()
        time.sleep(delay)
        step.off()
        time.sleep(delay)
    print(f"{motor_name} just moved up")

    # Move down
    direction.off()
    print(f"{motor_name} moving down {steps} steps")
    for _ in range(steps):
        step.on()
        time.sleep(delay)
        step.off()
        time.sleep(delay)
    print(f"{motor_name} just moved down")

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

try:
    print("Starting motor test... Press '1', '2', '3', or '4' to run the motors.")

    while True:
        key = get_key()
        if key == '1':
            t1 = threading.Thread(target=move_motor, args=(step_1, direction_1, 133, delay_time, "Motor 1"))
            t1.start()
            t1.join()
        elif key == '2':
            # Motor 2 steps up 100 steps and then back down
            t1 = threading.Thread(target=move_motor, args=(step_2, direction_2, 133, delay_time, "Motor 2"))
            t1.start()
            t1.join()
        elif key == '3':
            t1 = threading.Thread(target=move_motor, args=(step_3, direction_3, 133, delay_time, "Motor 3"))
            t1.start()
            t1.join()
        elif key == 'w':
            # Motor 1 and Motor 3 step up 100 steps and then back down
            t1 = threading.Thread(target=move_motor, args=(step_1, direction_1, 133, delay_time, "Motor 1"))
            t3 = threading.Thread(target=move_motor, args=(step_3, direction_3, 133, delay_time, "Motor 3"))
            t1.start()
            t3.start()
            t1.join()
            t3.join()
        elif key == 'd':
            # Motor 2 and Motor 3 step up 100 steps and then back down
            t2 = threading.Thread(target=move_motor, args=(step_2, direction_2, 133, delay_time, "Motor 2"))
            t3 = threading.Thread(target=move_motor, args=(step_3, direction_3, 222, delay_time/4, "Motor 3"))
            t2.start()
            t3.start()
            t2.join()
            t3.join()
        elif key == 'a':
            # Motor 1 steps up 133 steps and Motor 2 steps up 100 steps
            t1 = threading.Thread(target=move_motor, args=(step_1, direction_1, 222, delay_time/4, "Motor 1"))
            t2 = threading.Thread(target=move_motor, args=(step_2, direction_2, 133, delay_time, "Motor 2"))
            t1.start()
            t2.start()
            t1.join()
            t2.join()
        elif key == 's':
            # Motor 2 steps up 100 steps and then back down
            t1 = threading.Thread(target=move_motor, args=(step_2, direction_2, 133, delay_time, "Motor 2"))
            t1.start()
            t1.join()
        elif key == 'o':    
            # Move all motors 100 steps clockwise
            move_all_motors_cw(100, delay_time)
        elif key == 'p':    
            # Move all motors 100 steps clockwise
            move_all_motors_cww(100, delay_time)
        elif key == 'q':
            break

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