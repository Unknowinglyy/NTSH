import evdev
import time
from collections import deque

def read_touch_coordinates(device_path='/dev/input/event3'):
    device = evdev.InputDevice(device_path)

    print(f"Device: {device.name}")
    print(f"Listening for touch events on {device.path}...")
    
    x, y = None, None

    for event in device.read_loop():
        if event.type == evdev.ecodes.EV_ABS:
            if event.code == evdev.ecodes.ABS_X or event.code == evdev.ecodes.ABS_MT_POSITION_X:
                x = event.value
            elif event.code == evdev.ecodes.ABS_Y or event.code == evdev.ecodes.ABS_MT_POSITION_Y:
                y = event.value
            if x is not None and y is not None:
                yield (x, y, time.time())

def calculate_speed_and_acceleration(positions, times):
    if len(positions) < 2:
        return None, None

    # Calculate speed
    dx = positions[-1][0] - positions[-2][0]
    dy = positions[-1][1] - positions[-2][1]
    dt = times[-1] - times[-2]
    speed_x = dx / dt
    speed_y = dy / dt

    if len(positions) < 3:
        return (speed_x, speed_y), None

    # Calculate acceleration
    prev_speed_x = (positions[-2][0] - positions[-3][0]) / (times[-2] - times[-3])
    prev_speed_y = (positions[-2][1] - positions[-3][1]) / (times[-2] - times[-3])
    acceleration_x = (speed_x - prev_speed_x) / dt
    acceleration_y = (speed_y - prev_speed_y) / dt

    return (speed_x, speed_y), (acceleration_x, acceleration_y)

if __name__ == "__main__":
    positions = deque(maxlen=3)
    times = deque(maxlen=3)

    try:
        for x, y, t in read_touch_coordinates():
            positions.append((x, y))
            times.append(t)

            speed, acceleration = calculate_speed_and_acceleration(positions, times)

            if speed is not None:
                print(f"Speed: X: {speed[0]:.2f}, Y: {speed[1]:.2f}")
            if acceleration is not None:
                print(f"Acceleration: X: {acceleration[0]:.2f}, Y: {acceleration[1]:.2f}")

    except KeyboardInterrupt:
        print("Tracking interrupted.")