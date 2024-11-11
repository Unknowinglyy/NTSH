import evdev


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

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
                yield Point(x, y)
            elif event.type == evdev.ecodes.EV_KEY:
                yield Point(x, y)


if __name__ == "__main__":
    for x, y in read_touch_coordinates():
        print(f"X: {x}, Y: {y}")
        if x is None or y is None:
            break