import evdev
import time
# Replace 'eventX' with your actual event device for the touchscreen
device_path = '/dev/input/event6'
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
            print(f"Touch at X: {x}, Y: {y}")
    elif event.type == evdev.ecodes.EV_KEY:
            print(f"Touch at X: {x}, Y: {y}")
            time.sleep(2)