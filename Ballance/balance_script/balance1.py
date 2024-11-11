import time
import math
from gpiozero import OutputDevice
from touchScreenBasicCoordOutput import read_touch_coordinates

# Motor Pins
step_pin_A = 23  # STEP pin for Motor A
dir_pin_A = 24   # DIR pin for Motor A
step_pin_B = 20  # STEP pin for Motor B
dir_pin_B = 21   # DIR pin for Motor B
step_pin_C = 5   # STEP pin for Motor C
dir_pin_C = 6    # DIR pin for Motor C

# Motor Step and Direction Controls
stepperA = OutputDevice(step_pin_A)
directionA = OutputDevice(dir_pin_A)
stepperB = OutputDevice(step_pin_B)
directionB = OutputDevice(dir_pin_B)
stepperC = OutputDevice(step_pin_C)
directionC = OutputDevice(dir_pin_C)

# Target (center) coordinates for the ball
TARGET_X = 2025
TARGET_Y = 2045

# PID Controller constants
KP = 0.001
KI = 0.0001
KD = 0.005

# PID state variables
error_prev_x = 0
error_prev_y = 0
integral_x = 0
integral_y = 0

# Conversion factor: steps per degree for the motors (may need tuning)
STEPS_PER_DEGREE = 3200 / 360

def move_motor(motor_step, motor_dir, steps):
    """Moves the motor a specific number of steps in a given direction."""
    if steps > 0:
        motor_dir.on()  # Set direction to clockwise
    else:
        motor_dir.off()  # Set direction to counter-clockwise
    for _ in range(abs(steps)):
        motor_step.on()
        time.sleep(0.0005)  # Adjust step delay as necessary
        motor_step.off()
        time.sleep(0.0005)

def pid_control(target_x, target_y, current_x, current_y):
    """Calculates PID outputs for both X and Y axes to balance the ball."""
    global error_prev_x, error_prev_y, integral_x, integral_y

    # Calculate errors
    error_x = target_x - current_x
    error_y = target_y - current_y

    # Proportional terms
    p_term_x = KP * error_x
    p_term_y = KP * error_y

    # Integral terms
    integral_x += error_x
    integral_y += error_y
    i_term_x = KI * integral_x
    i_term_y = KI * integral_y

    # Derivative terms
    derivative_x = error_x - error_prev_x
    derivative_y = error_y - error_prev_y
    d_term_x = KD * derivative_x
    d_term_y = KD * derivative_y

    # Update previous errors
    error_prev_x = error_x
    error_prev_y = error_y

    # PID output
    output_x = p_term_x + i_term_x + d_term_x
    output_y = p_term_y + i_term_y + d_term_y

    # Return the calculated outputs
    return output_x, output_y

def control_platform(output_x, output_y):
    """Moves the motors based on the PID outputs for X and Y directions."""
    # Convert PID outputs to motor steps (needs tuning for real hardware)
    steps_A = int(output_x * STEPS_PER_DEGREE)
    steps_B = int(output_y * STEPS_PER_DEGREE)
    steps_C = int((output_x + output_y) * STEPS_PER_DEGREE / 2)

    # Move motors based on the calculated steps
    move_motor(stepperA, directionA, steps_A)
    move_motor(stepperB, directionB, steps_B)
    move_motor(stepperC, directionC, steps_C)

def main():
    try:
        while True:
            # Read the current position of the ball
            ball_position = read_touch_coordinates()
            
            # If the ball position is valid, proceed with balancing
            if ball_position and ball_position.x is not None and ball_position.y is not None:
                current_x, current_y = ball_position.x, ball_position.y

                # Calculate PID control output for X and Y
                output_x, output_y = pid_control(TARGET_X, TARGET_Y, current_x, current_y)

                # Move the platform according to the PID output
                control_platform(output_x, output_y)

                # Print diagnostic information
                print(f"Ball Position: ({current_x}, {current_y}), PID Output: ({output_x}, {output_y})")
            else:
                print("Ball not detected.")

            # Run at a consistent rate
            time.sleep(0.02)  # 20 ms loop delay
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        # Ensure all motors are stopped
        stepperA.close()
        directionA.close()
        stepperB.close()
        directionB.close()
        stepperC.close()
        directionC.close()

if __name__ == "__main__":
    main()
