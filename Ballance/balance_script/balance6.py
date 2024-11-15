import RPi.GPIO as GPIO
import time
import math
from simple_pid import PID
from evdev import InputDevice, ecodes

# GPIO setup for stepper motors
MOTOR_PINS = {
    'motor1': {'step': 23, 'dir': 24},
    'motor2': {'step': 20, 'dir': 21},
    'motor3': {'step': 5, 'dir': 6}
}

# Motor corner positions
MOTOR_POSITIONS = {
    'motor1': (3685, 200),
    'motor2': (3685, 3820),
    'motor3': (390, 1810)
}

# Center position of the touchscreen
CENTER_X, CENTER_Y = 2025, 2045

clockwise_steps_motor1 = 0
clockwise_steps_motor2 = 0
clockwise_steps_motor3 = 0

# Set up GPIO
GPIO.setmode(GPIO.BCM)
for motor in MOTOR_PINS.values():
    GPIO.setup(motor['step'], GPIO.OUT)
    GPIO.setup(motor['dir'], GPIO.OUT)

# PID controllers for X and Y directions
pid_x = PID(0.6, 0.1, 0.05, setpoint=CENTER_X)
pid_y = PID(0.6, 0.1, 0.05, setpoint=CENTER_Y)
pid_x.sample_time = 0.1  # 100 ms update rate
pid_y.sample_time = 0.1
pid_x.output_limits = (-10, 10)  # Limiting output to max ±10 steps
pid_y.output_limits = (-10, 10)


def read_touch_coordinates(device_path='/dev/input/event3'):
    device = InputDevice(device_path)
    x, y = None, None
    for event in device.read_loop():
        if event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_X or event.code == ecodes.ABS_MT_POSITION_X:
                x = event.value
            elif event.code == ecodes.ABS_Y or event.code == ecodes.ABS_MT_POSITION_Y:
                y = event.value
            if x is not None and y is not None:
                yield (x, y)

def move_motor(motor, steps, clockwise):
    GPIO.output(MOTOR_PINS[motor]['dir'], GPIO.HIGH if clockwise else GPIO.LOW)
    for _ in range(abs(steps)):
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.LOW)
        time.sleep(0.001)
        # Update the step count for the motor
        if clockwise:
            if motor == 'motor1':
                global clockwise_steps_motor1
                clockwise_steps_motor1 += 1
            elif motor == 'motor2':
                global clockwise_steps_motor2
                clockwise_steps_motor2 += 1
            elif motor == 'motor3':
                global clockwise_steps_motor3
                clockwise_steps_motor3 += 1
        else:
            if motor == 'motor1':
                global clockwise_steps_motor1
                clockwise_steps_motor1 -= 1
            elif motor == 'motor2':
                global clockwise_steps_motor2
                clockwise_steps_motor2 -= 1
            elif motor == 'motor3':
                global clockwise_steps_motor3
                clockwise_steps_motor3 -= 1

def move_all_motors_cw(steps, delay):
    # Move all motors clockwise
    for _ in range(steps):
        for motor in MOTOR_PINS.values():
            GPIO.output(motor['dir'], GPIO.HIGH)
            GPIO.output(motor['step'], GPIO.HIGH)
        time.sleep(delay)
        for motor in MOTOR_PINS.values():
            GPIO.output(motor['step'], GPIO.LOW)
        time.sleep(delay)
        
    #increase the steps for the motors
    clockwise_steps_motor1 += steps
    clockwise_steps_motor2 += steps
    clockwise_steps_motor3 += steps
    print(f"All motors moved CW {steps} steps")

def move_all_motors_ccw(steps, delay):
    # Move all motors counterclockwise
    for _ in range(steps):
        for motor in MOTOR_PINS.values():
            GPIO.output(motor['dir'], GPIO.LOW)
            GPIO.output(motor['step'], GPIO.HIGH)
        time.sleep(delay)
        for motor in MOTOR_PINS.values():
            GPIO.output(motor['step'], GPIO.LOW)
        time.sleep(delay)
        
    #decrease the steps for the motors
    clockwise_steps_motor1 -= steps
    clockwise_steps_motor2 -= steps
    clockwise_steps_motor3 -= steps
    print(f"All motors moved CCW {steps} steps")

def calculate_motor_steps(ball_x, ball_y):
    steps_x = int(pid_x(ball_x))
    steps_y = int(pid_y(ball_y))
    motor_steps = {}
    for motor, pos in MOTOR_POSITIONS.items():
        motor_error_x = CENTER_X - pos[0]
        motor_error_y = CENTER_Y - pos[1]
        clockwise_x = steps_x > 0 if motor_error_x > 0 else steps_x < 0
        clockwise_y = steps_y > 0 if motor_error_y > 0 else steps_y < 0
        steps = abs(steps_x) + abs(steps_y)
        clockwise = clockwise_x if abs(steps_x) > abs(steps_y) else clockwise_y
        motor_steps[motor] = (steps, clockwise)
    return motor_steps

def balance_ball():
    try:
        for x, y in read_touch_coordinates():
            motor_steps = calculate_motor_steps(x, y)
            for motor, (steps, clockwise) in motor_steps.items():
                move_motor(motor, steps, clockwise)
            time.sleep(0.1)
    except KeyboardInterrupt:
        #move motors back to original position
        move_all_motors_ccw(abs(clockwise_steps_motor1), 0.001)
        #then do the cleanup
        GPIO.cleanup()

if __name__ == "__main__":
    try:
        move_all_motors_cw(100, 0.01)
        balance_ball()
    except KeyboardInterrupt:
        print("Ball balancing interrupted.")
