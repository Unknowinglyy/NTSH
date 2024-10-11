import evdev

# Replace 'eventX' with your actual event device for the touchscreen
device_path = '/dev/input/event4'
device = evdev.InputDevice(device_path)

print(f"Device: {device.name}")
print(f"Listening for touch events on {device.path}...")

# Read touch events in a loop
for event in device.read_loop():
    if event.type == evdev.ecodes.EV_ABS:
        if event.code == evdev.ecodes.ABS_X:
            x = event.value
            print(f"X: {x}")
        if event.code == evdev.ecodes.ABS_Y:
            y = event.value
            print(f"Y: {y}")
