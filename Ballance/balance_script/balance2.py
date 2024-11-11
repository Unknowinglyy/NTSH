import evdev
from gpiozero import OutputDevice
import time

# Pin definitions
step_pin = 23  # Pin connected to STEP on TMC2208
dir_pin = 24   # Pin connected to DIR on TMC2208

step_pin2 = 20  # Pin connected to STEP on 2nd TMC2208
dir_pin2 = 21   # Pin connected to DIR on 2nd TMC2208

step_pin3 = 5 # Pin connected to STEP on 3rd TMC2208
dir_pin3 = 6  # Pin connected to DIR on 3rd TMC2208

enable_pin = 4 # Pin connected to EN on TMC2208

# Setup GPIO
step = OutputDevice(step_pin)
direction = OutputDevice(dir_pin)

step2 = OutputDevice(step_pin2)
direction2 = OutputDevice(dir_pin2)

step3 = OutputDevice(step_pin3)
direction3 = OutputDevice(dir_pin3)

enable = OutputDevice(enable_pin, initial_value=False)

# PID controller parameters
Kp = 1.0
Ki = 0.1
Kd = 0.05

# PID controller variables
integral = 0
previous_error = 0

def pid_control(setpoint, measured_value):
    global integral, previous_error
    error = setpoint - measured_value
    integral += error
    derivative = error - previous_error
    output = Kp * error + Ki * integral + Kd * derivative
    previous_error = error
    return output

def read_touch_coordinates(device_path='/dev/input/event4'):
    device = evdev.InputDevice(device_path)

    print(f"Device: {device.name}")
    print(f"Listening for touch events on {device.path}...")

    x, y = None, None

    # Read touch events in a loop
    for event in device.read_loop():
        # Only process single-touch absolute X and Y events (ignore multitouch)
        if event.type == evdev.ecodes.EV_ABS:
            if event.code == evdev.ecodes.ABS_X or event.code == evdev.ecodes.ABS_MT_POSITION_X:
                x = event.value
            elif event.code == evdev.ecodes.ABS_Y or event.code == evdev.ecodes.ABS_MT_POSITION_Y:
                y = event.value
            # Print coordinates when both X and Y are captured
            if x is not None and y is not None:
                yield (x, y)

if __name__ == "__main__":
    setpoint_x = 2025  # Desired X coordinate
    setpoint_y = 2045  # Desired Y coordinate

    for x, y in read_touch_coordinates():
        print(f"X: {x}, Y: {y}")

        # Calculate PID output for X and Y
        output_x = pid_control(setpoint_x, x)
        output_y = pid_control(setpoint_y, y)

        # Adjust motor positions based on PID output
        if output_x > 0:
            direction.on()
        else:
            direction.off()
        for _ in range(abs(int(output_x))):
            step.on()
            time.sleep(0.001)
            step.off()
            time.sleep(0.001)

        if output_y > 0:
            direction2.on()
        else:
            direction2.off()
        for _ in range(abs(int(output_y))):
            step2.on()
            time.sleep(0.001)
            step2.off()
            time.sleep(0.001)

        # Move the third motor based on the average of X and Y PID outputs
        output_z = (output_x + output_y) / 2
        if output_z > 0:
            direction3.on()
        else:
            direction3.off()
        for _ in range(abs(int(output_z))):
            step3.on()
            time.sleep(0.001)
            step3.off()
            time.sleep(0.001)
        
        print("Motors moved based on PID output")