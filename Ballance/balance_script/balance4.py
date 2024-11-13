import RPi.GPIO as GPIO
import time
import evdev
import math

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

# Set up GPIO
GPIO.setmode(GPIO.BCM)
for motor in MOTOR_PINS.values():
    GPIO.setup(motor['step'], GPIO.OUT)
    GPIO.setup(motor['dir'], GPIO.OUT)

# Define a function to control a single motor
def move_motor(motor, steps, clockwise):
    GPIO.output(MOTOR_PINS[motor]['dir'], GPIO.HIGH if clockwise else GPIO.LOW)
    for _ in range(steps):
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.HIGH)
        time.sleep(0.001)  # Control speed with sleep duration
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.LOW)
        time.sleep(0.001)

# Read touch coordinates from the touchscreen
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def read_touch_coordinates(device_path='/dev/input/event4'):
    device = evdev.InputDevice(device_path)
    x, y = None, None
    for event in device.read_loop():
        if event.type == evdev.ecodes.EV_ABS:
            if event.code == evdev.ecodes.ABS_X or event.code == evdev.ecodes.ABS_MT_POSITION_X:
                x = event.value
            elif event.code == evdev.ecodes.ABS_Y or event.code == evdev.ecodes.ABS_MT_POSITION_Y:
                y = event.value
            if x is not None and y is not None:
                return Point(x, y)

# Calculate distance from ball to each motor corner
def calculate_motor_steps(ball_x, ball_y):
    target_x, target_y = CENTER_X, CENTER_Y
    error_x = target_x - ball_x
    error_y = target_y - ball_y
    distance_to_center = math.sqrt(error_x**2 + error_y**2)

    # Scale steps based on distance error, defining max step limit for each control cycle
    max_steps = 10  # adjust as needed
    scale_factor = min(1, distance_to_center / 1000)  # scales within [0, 1]
    steps = int(max_steps * scale_factor)

    # Calculate angles for each motor
    motor_steps = {}
    for motor, pos in MOTOR_POSITIONS.items():
        motor_error_x = target_x - pos[0]
        motor_error_y = target_y - pos[1]
        distance = math.sqrt(motor_error_x**2 + motor_error_y**2)
        
        # Determine direction based on the sign of the x and y error projections
        clockwise = error_x * motor_error_x + error_y * motor_error_y > 0
        motor_steps[motor] = (steps, clockwise)

    return motor_steps

# Main loop
def balance_ball():
    try:
        while True:
            point = read_touch_coordinates()
            ball_x, ball_y = point.x, point.y
            motor_steps = calculate_motor_steps(ball_x, ball_y)

            # Move each motor according to the calculated steps
            for motor, (steps, clockwise) in motor_steps.items():
                move_motor(motor, steps, clockwise)
            time.sleep(0.1)
    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == "__main__":
    balance_ball()