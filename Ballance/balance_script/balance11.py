import RPi.GPIO as GPIO
import time
import threading
from touchScreenBasicCoordOutput import read_touch_coordinates

# --------------------------------------------------------------------------------------------
# GPIO setup for stepper motors
MOTOR_PINS = {
    'motor1': {'step': 23, 'dir': 24},  # (3670, 210)
    'motor2': {'step': 20, 'dir': 21},  # (397, 2005)
    'motor3': {'step': 5, 'dir': 6}     # (3670, 3800)
}

# Center position of the platform
CENTER_X, CENTER_Y = 2025, 2045
STEP_DELAY = 0.001  # 1 ms delay for microstepping

# GPIO Setup
GPIO.setmode(GPIO.BCM)
for motor in MOTOR_PINS.values():
    GPIO.setup(motor['step'], GPIO.OUT)
    GPIO.setup(motor['dir'], GPIO.OUT)

# --------------------------------------------------------------------------------------------
def move_motor(motor, steps, clockwise):
    """
    Moves a single motor a specified number of steps in a specified direction.
    """
    GPIO.output(MOTOR_PINS[motor]['dir'], GPIO.HIGH if clockwise else GPIO.LOW)
    for _ in range(abs(steps)):
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.HIGH)
        time.sleep(STEP_DELAY)
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.LOW)
        time.sleep(STEP_DELAY)

def move_motors_concurrently(motor_steps):
    """
    Moves the motors concurrently using threading.
    """
    threads = []
    for motor, (steps, clockwise) in motor_steps.items():
        if steps > 0:
            t = threading.Thread(target=move_motor, args=(motor, steps, clockwise))
            threads.append(t)
            t.start()
    for t in threads:
        t.join()

# --------------------------------------------------------------------------------------------
def balance_ball():
    """
    Main loop to balance the ball based on quadrant position.
    """
    try:
        while True:
            point = read_touch_coordinates()
            if point is None:
                continue

            ball_x, ball_y = point.x, point.y

            # Determine quadrant
            if ball_x >= CENTER_X and ball_y <= CENTER_Y:  # Q1
                print(ball_x, ball_y)
                motor_steps = {
                    'motor1': (100, False),  # Motor1 tilts down (CWW)
                    'motor2': (100, True),  # Motor2 tilts up (CW)
                    'motor3': (50, True)    # Motor3 tilts up (CW)
                }
            elif ball_x <= CENTER_X and ball_y <= CENTER_Y:  # Q2
                print(ball_x, ball_y)
                motor_steps = {
                    'motor1': (100, True),  # Motor1 tilts up (CW)
                    'motor2': (50, True),  # Motor2 tilts up (CW)
                    'motor3': (100, False) # Motor3 tilts down (CWW)
                }
            elif ball_x <= CENTER_X and ball_y >= CENTER_Y:  # Q3
                print(ball_x, ball_y)
                motor_steps = {
                    'motor1': (50, True),   # Motor1 tilts up (CW)
                    'motor2': (100, False), # Motor2 tilts down (CWW)
                    'motor3': (100, True)   # Motor3 tilts up (CW)
                }
            elif ball_x >= CENTER_X and ball_y >= CENTER_Y:  # Q4
                print(ball_x, ball_y)
                motor_steps = {
                    'motor1': (100, False), # Motor1 tilts down (CWW)
                    'motor2': (50, True),   # Motor2 tilts up (CW)
                    'motor3': (100, True)   # Motor3 tilts up (CW)
                }
            else:
                # Centered; no movement needed
                motor_steps = {
                    'motor1': (0, True),
                    'motor2': (0, True),
                    'motor3': (0, True)
                }

            # Move the motors according to the determined steps
            move_motors_concurrently(motor_steps)

            time.sleep(0.02)  # Delay for next update
    except KeyboardInterrupt:
        print("Exiting program...")
    finally:
        GPIO.cleanup()

# --------------------------------------------------------------------------------------------
if __name__ == "__main__":
    print("Starting balance loop...")
    balance_ball()
