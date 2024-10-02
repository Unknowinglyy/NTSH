import evdev

# Replace 'eventX' with your actual event device for the touchscreen
device_path = '/dev/input/event4'
device = evdev.InputDevice(device_path)

print(f"Device: {device.name}")
print(f"Listening for touch events on {device.path}...")

# Read touch events in a loop
for event in device.read_loop():
    if event.type == evdev.ecodes.EV_ABS:
            x = event.value
            y = event.value
            print(f"X Coordinate: {x}")
            print(f"Y Coordinate: {y}")
