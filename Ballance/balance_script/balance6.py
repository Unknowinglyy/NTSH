import evdev
import time
from gpiozero import OutputDevice
from simple_pid import PID

# Motor pins
step_pin = 23
dir_pin = 24
step_pin2 = 20
dir_pin2 = 21
step_pin3 = 5
dir_pin3 = 6

# Motor setup
step = OutputDevice(step_pin)
direction = OutputDevice(dir_pin)
step2 = OutputDevice(step_pin2)
direction2 = OutputDevice(dir_pin2)
step3 = OutputDevice(step_pin3)
direction3 = OutputDevice(dir_pin3)

# Motor movement parameters
test_steps = 200
delay_time = 0.005

# Step counters
clockwise_steps_motor1 = 0
clockwise_steps_motor2 = 0
clockwise_steps_motor3 = 0

def read_touch_coordinates(device_path='/dev/input/event3'):
    device = evdev.InputDevice(device_path)
    x, y = None, None
    for event in device.read_loop():
        if event.type == evdev.ecodes.EV_ABS:
            if event.code == evdev.ecodes.ABS_X or event.code == evdev.ecodes.ABS_MT_POSITION_X:
                x = event.value
            elif event.code == evdev.ecodes.ABS_Y or event.code == evdev.ecodes.ABS_MT_POSITION_Y:
                y = event.value
            if x is not None and y is not None:
                yield (x, y, time.time())

def move_motor(motor, steps, direction_pin, direction):
    global clockwise_steps_motor1, clockwise_steps_motor2, clockwise_steps_motor3
    direction_pin.value = direction
    for _ in range(abs(steps)):
        motor.on()
        time.sleep(0.0005)
        motor.off()
        time.sleep(0.0005)
        if direction:  # Clockwise direction
            if motor == step:
                clockwise_steps_motor1 += 1
            elif motor == step2:
                clockwise_steps_motor2 += 1
            elif motor == step3:
                clockwise_steps_motor3 += 1
        else:  # Counterclockwise direction
            if motor == step:
                clockwise_steps_motor1 -= 1
            elif motor == step2:
                clockwise_steps_motor2 -= 1
            elif motor == step3:
                clockwise_steps_motor3 -= 1

def calculate_motor_steps(x, y):
    # Placeholder function to calculate motor steps based on x, y coordinates
    motor_steps = {}
    steps_x = x - 2025
    steps_y = y - 2045
    motor_error_x = steps_x
    motor_error_y = steps_y
    clockwise_x = steps_x > 0 if motor_error_x > 0 else steps_x < 0
    clockwise_y = steps_y > 0 if motor_error_y > 0 else steps_y < 0
    steps = abs(steps_x) + abs(steps_y)
    clockwise = clockwise_x if abs(steps_x) > abs(steps_y) else clockwise_y
    motor_steps[step] = (steps, clockwise)
    return motor_steps

def move_all_motors_cw(steps, delay):
    # Move all motors clockwise
    direction.on()
    direction2.on()
    direction3.on()
    print(f"All motors moving CW {steps} steps")
    for _ in range(steps):
        step.on()
        step2.on()
        step3.on()
        time.sleep(delay)
        step.off()
        step2.off()
        step3.off()
        time.sleep(delay)
    print("All motors just moved CW")

def balance_ball():
    try:
        for x, y in read_touch_coordinates():
            motor_steps = calculate_motor_steps(x, y)
            for motor, (steps, clockwise) in motor_steps.items():
                move_motor(motor, steps, clockwise)
            time.sleep(0.001)
    except KeyboardInterrupt:
        # Reset all motors to initial position
        move_motor(step, clockwise_steps_motor1, direction, False)
        move_motor(step2, clockwise_steps_motor2, direction2, False)
        move_motor(step3, clockwise_steps_motor3, direction3, False)
        print("Motors reset to initial position.")

        step.close()
        step2.close()
        step3.close()
        direction.close()
        direction2.close()
        direction3.close()
        print("GPIO cleaned up.")

if __name__ == "__main__":
    try:
        move_all_motors_cw(100, 0.001)
        balance_ball()
    except KeyboardInterrupt:
        print("Motor test interrupted.")