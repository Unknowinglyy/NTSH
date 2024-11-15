import class_MotorControl
import class_PID
import time
import threading
from touchScreenBasicCoordOutput import read_touch_coordinates, Point

# Motor pins setup
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

# PID coefficients
K_PID = [1.2, 0.1, 0.05]

# Initialize MotorControl and PID
motor_control = class_MotorControl.MotorControl(MOTOR_PINS)
pid_x = class_PID.PID(K_PID, setpoint=CENTER_X)
pid_y = class_PID.PID(K_PID, setpoint=CENTER_Y)

# Configure sample time (update frequency) and output limits
pid_x.sample_time = 0.1  # 100 ms update rate
pid_y.sample_time = 0.1
pid_x.output_limits = (-10, 10)  # Limiting output to max ±10 steps
pid_y.output_limits = (-10, 10)

# Function to check if the ball is near a specific motor
def is_ball_near_motor(ball_x, ball_y, motor_pos, threshold=100):
    distance = ((ball_x - motor_pos[0])**2 + (ball_y - motor_pos[1])**2)**0.5
    return distance < threshold

# Main loop
def balance_ball():
    try:
        while True:
            point = read_touch_coordinates()
            if point is not None:
                ball_x, ball_y = point.x, point.y

                # Check if the ball is near any motor and move that motor
                for motor, pos in MOTOR_POSITIONS.items():
                    if is_ball_near_motor(ball_x, ball_y, pos):
                        motor_control.move_motor(motor, 100, True)  # Move 100 steps clockwise
                        break  # Only move one motor at a time
            time.sleep(0.1)  # Update cycle delay
    except KeyboardInterrupt:
        motor_control.cleanup()

if __name__ == "__main__":
    # Start balancing the ball
    balance_ball()