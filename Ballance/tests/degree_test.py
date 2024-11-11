from gpiozero import OutputDevice
import time

# Pin definitions
step_pin = 23  # Pin connected to STEP on TMC2208
dir_pin = 24   # Pin connected to DIR on TMC2208

step_pin2 = 20  # Pin connected to STEP on 2nd TMC2208
dir_pin2 = 21   # Pin connected to DIR on 2nd TMC2208

step_pin3 = 5  # Pin connected to STEP on 3rd TMC2208
dir_pin3 = 6   # Pin connected to DIR on 3rd TMC2208

enable_pin = 4  # Pin connected to EN on TMC2208

# Motor movement parameters
steps_per_revolution = 500  # Number of steps per full revolution (adjust as needed)
delay_time = 0.005  # Delay in seconds between steps

# Setup GPIO
step = OutputDevice(step_pin)
direction = OutputDevice(dir_pin)

step2 = OutputDevice(step_pin2)
direction2 = OutputDevice(dir_pin2)

step3 = OutputDevice(step_pin3)
direction3 = OutputDevice(dir_pin3)

enable = OutputDevice(enable_pin, initial_value=False)  # Set initial value to LOW (enabled)

def move_motor(step, steps, delay):
    for _ in range(steps):
        step.on()
        time.sleep(delay)
        step.off()
        time.sleep(delay)

def test_motor(step, direction, steps_per_revolution, delay):
    # Move motor clockwise
    direction.on()
    move_motor(step, steps_per_revolution, delay)
    print(f"Moved {steps_per_revolution} steps clockwise")

    # Move motor counterclockwise
    direction.off()
    move_motor(step, steps_per_revolution, delay)
    print(f"Moved {steps_per_revolution} steps counterclockwise")

def main():
    try:
        print("Starting motor test...")
        enable.on()  # Enable the motor driver (set to LOW)

        # Test each motor
        print("Testing Motor 1")
        test_motor(step, direction, steps_per_revolution, delay_time)

        print("Testing Motor 2")
        test_motor(step2, direction2, steps_per_revolution, delay_time)

        print("Testing Motor 3")
        test_motor(step3, direction3, steps_per_revolution, delay_time)

    except KeyboardInterrupt:
        print("Motor test interrupted.")

    finally:
        enable.off()  # Disable the motor driver (set to HIGH)
        step.close()
        direction.close()
        step2.close()
        direction2.close()
        step3.close()
        direction3.close()
        print("GPIO cleaned up.")

if __name__ == "__main__":
    main()