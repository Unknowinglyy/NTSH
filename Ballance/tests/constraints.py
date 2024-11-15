import time
from gpiozero import OutputDevice

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
test_steps = 50
delay_time = 0.01

# Step counters
clockwise_steps_motor1 = 0
clockwise_steps_motor2 = 0
clockwise_steps_motor3 = 0

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

if __name__ == "__main__":
    try:
        print("Starting motor test...")

        # Example movements
        move_motor(step, test_steps, direction, True)  # Motor 1 clockwise
        time.sleep(1)
        move_motor(step2, test_steps, direction2, True)  # Motor 2 clockwise
        time.sleep(1)
        move_motor(step3, test_steps, direction3, True)  # Motor 3 clockwise
        time.sleep(1)

        move_motor(step, test_steps, direction, True)  # Motor 1 clockwise
        time.sleep(1)
        move_motor(step2, test_steps, direction2, True)  # Motor 2 clockwise
        time.sleep(1)
        move_motor(step3, test_steps, direction3, True)  # Motor 3 clockwise
        time.sleep(1)

        print(f"Clockwise steps - Motor 1: {clockwise_steps_motor1}")
        print(f"Clockwise steps - Motor 2: {clockwise_steps_motor2}")
        print(f"Clockwise steps - Motor 3: {clockwise_steps_motor3}")

        time.sleep(3)

        #testing how much the motors can move from their current position
        move_motor(step, 250, direction, True)  # Motor 1 clockwise
        time.sleep(1)


        

        # print("Now moving back to the starting position...")
        # move_motor(step, clockwise_steps_motor1, direction, False)  # Motor 1 counterclockwise
        # time.sleep(1)
        # move_motor(step2, clockwise_steps_motor2, direction2, False)  # Motor 2 counterclockwise
        # time.sleep(1)
        # move_motor(step3, clockwise_steps_motor3, direction3, False)  # Motor 3 counterclockwise

        print("Motor test completed.")

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