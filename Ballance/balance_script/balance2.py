import evdev
import time
from gpiozero import OutputDevice
from simple_pid import PID

# Motor pins
step_pin = 23
dir_pin = 24
step_pin2 = 20
dir_pin2 = 21
step_pin3 = 5
dir_pin3 = 6
enable_pin = 4

# Motor setup
step = OutputDevice(step_pin)
direction = OutputDevice(dir_pin)
step2 = OutputDevice(step_pin2)
direction2 = OutputDevice(dir_pin2)
step3 = OutputDevice(step_pin3)
direction3 = OutputDevice(dir_pin3)
enable = OutputDevice(enable_pin, initial_value=False)

# PID setup
pid_x = PID(1, 0.1, 0.05, setpoint=2025)
pid_y = PID(1, 0.1, 0.05, setpoint=2045)
pid_x.output_limits = (-100, 100)
pid_y.output_limits = (-100, 100)

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
                yield (x, y)

def move_motor(motor, steps, direction_pin, direction):
    direction_pin.value = direction
    for _ in range(abs(steps)):
        motor.on()
        time.sleep(0.001)
        motor.off()
        time.sleep(0.001)

if __name__ == "__main__":
    enable.on()
    try:
        for x, y in read_touch_coordinates():
            error_x = pid_x(x)
            error_y = pid_y(y)

            if error_x > 0:
                move_motor(step2, error_x, direction2, True)
            else:
                move_motor(step3, -error_x, direction3, False)

            if error_y > 0:
                move_motor(step, error_y, direction, True)
            else:
                move_motor(step3, -error_y, direction3, False)

            print(f"X: {x}, Y: {y}, Error X: {error_x}, Error Y: {error_y}")

    except KeyboardInterrupt:
        enable.off()