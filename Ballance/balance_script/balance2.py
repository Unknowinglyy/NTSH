import evdev
import time
from gpiozero import OutputDevice
from simple_pid import PID

#motor pins
step_pin = 23
dir_pin = 24
step_pin2 = 20
dir_pin2 = 21
step_pin3 = 5
dir_pin3 = 6

#motor setup
#motor 1
step = OutputDevice(step_pin)
direction = OutputDevice(dir_pin)

#motor 2
step2 = OutputDevice(step_pin2)
direction2 = OutputDevice(dir_pin2)

#motor 3
step3 = OutputDevice(step_pin3)
direction3 = OutputDevice(dir_pin3)

#pid setup using simple_pid module
pid_x = PID(1, 0.1, 0.05, setpoint=2025)
pid_y = PID(1, 0.1, 0.05, setpoint=2045)
pid_x.output_limits = (-100, 100)
pid_y.output_limits = (-100, 100)

#reading the coordinates of the ball (from previous implementation)
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
            elif event.type == evdev.ecodes.EV_KEY:
                yield (x, y)

#moving of motor function
def move_motor(motor, steps, direction_pin, direction):
    direction_pin.value = direction
    for _ in range(int(abs(steps))):
        motor.on()
        time.sleep(0.01)
        motor.off()
        time.sleep(0.01)

if __name__ == "__main__":
    try:
        for x, y in read_touch_coordinates():
            error_x = pid_x(x)
            error_y = pid_y(y)

            #change the direction of the motor based on the error
            if error_x > 0:
                move_motor(step2, error_x, direction2, True)
            else:
                move_motor(step2, -error_x, direction2, False)

            if error_y > 0:
                move_motor(step, error_y, direction, False)
            else:
                move_motor(step, -error_y, direction, True)

            if error_x < 0 and error_y < 0:
                move_motor(step3, -error_x, direction3, True)
            elif error_x > 0 and error_y > 0:
                move_motor(step3, error_x, direction3, False)

            print(f"X: {x}, Y: {y}, Error X: {error_x}, Error Y: {error_y}")

    except KeyboardInterrupt:
        print("Motor control interrupted.")

    finally:
        #clean up the GPIO pins
        step.close()
        step2.close()
        step3.close()
        direction.close()
        direction2.close()
        direction3.close()
        print("GPIO cleaned up.")