from gpiozero import OutputDevice
import time

enable_pin = 4 # Pin connected to EN on TMC2208

# Motor movement parameters
test_steps = 200              # Number of steps to move in each direction
delay_time = 0.005          # Delay in seconds between steps

enable = OutputDevice(enable_pin, initial_value=False)

try:
    print("Starting enable test...")
    while True:
        print("enabling")
        enable.on()
        time.sleep(10)

except KeyboardInterrupt:
    print("enable test interrupted.")

finally:
    enable.off()
    print("exiting.")
