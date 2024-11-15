import RPi.GPIO as GPIO
import time
import math
from touchScreenBasicCoordOutput import read_touch_coordinates, Point
import threading

# GPIO setup for stepper motors
MOTOR_PINS = {
    'motor1': {'step': 23, 'dir': 24},
    'motor2': {'step': 20, 'dir': 21},
    'motor3': {'step': 5, 'dir': 6}
}

# Motor corner positions
MOTOR_POSITIONS = {
    'motor1': (3670, 210),
    'motor2': (397, 2033.5),
    'motor3': (3670, 3800)
}

# Center position of the touchscreen
CENTER_X, CENTER_Y = 2005, 2033.5

# Set up GPIO
GPIO.setmode(GPIO.BCM)
for motor in MOTOR_PINS.keys():
    GPIO.setup(MOTOR_PINS[motor]['step'], GPIO.OUT)
    GPIO.setup(MOTOR_PINS[motor]['dir'], GPIO.OUT)

# Function to move all motors clockwise
def move_all_motors_cw(steps, delay):
    for _ in range(steps):
        for motor in MOTOR_PINS.keys():
            GPIO.output(MOTOR_PINS[motor]['dir'], GPIO.HIGH)
            GPIO.output(MOTOR_PINS[motor]['step'], GPIO.HIGH)
        time.sleep(delay)
        for motor in MOTOR_PINS.keys():
            GPIO.output(MOTOR_PINS[motor]['step'], GPIO.LOW)
        time.sleep(delay)

# Define a function to control a single motor
def move_motor(motor, steps, clockwise, delay=0, step_delay=0.002):
    time.sleep(delay)
    GPIO.output(MOTOR_PINS[motor]['dir'], GPIO.HIGH if clockwise else GPIO.LOW)
    for _ in range(abs(steps)):
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.HIGH)
        time.sleep(step_delay)
        GPIO.output(MOTOR_PINS[motor]['step'], GPIO.LOW)
        time.sleep(step_delay)

# Main loop
def balance_ball():
    try:
        while True:
            point = read_touch_coordinates()
            if point is not None:
                ball_x, ball_y = point.x, point.y
                error_x = CENTER_X - ball_x
                error_y = CENTER_Y - ball_y

                # Calculate steps proportional to the error magnitude
                steps_x = int(abs(error_x) * 0.1)  # Adjust the scaling factor as needed

                # Determine which motors to move based on the ball's position
                if error_x > 0:
                    t1 = threading.Thread(target=move_motor, args=('motor1', steps_x, False))
                    t2 = threading.Thread(target=move_motor, args=('motor3', steps_x, False))
                    t3 = threading.Thread(target=move_motor, args=('motor2', steps_x+50, False, 0.08, .001))
                    t1.start()
                    t2.start()
                    t3.start()
                    t1.join()
                    t2.join()
                    t3.join()
                    
                elif error_x < 0:
                    t1 = threading.Thread(target=move_motor, args=('motor1', steps_x, True))
                    t2 = threading.Thread(target=move_motor, args=('motor3', steps_x, True))
                    t3 = threading.Thread(target=move_motor, args=('motor2', steps_x+50, True, 0.08, .001))
                    t1.start()
                    t2.start()
                    t3.start()
                    t1.join()
                    t2.join()
                    t3.join()
            time.sleep(0.1)  # Update cycle delay
    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == "__main__":
    # Move all motors 100 Steps CW
    move_all_motors_cw(100, 0.001)

    # Begin Balance 
    balance_ball()