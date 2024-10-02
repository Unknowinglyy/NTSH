import evdev

device_path = '/dev/input/event4'

device = evdev.InputDevice(device_path)

print(f"Listening for events from {device_path}...")

try:
    for event in device.read_loop():
        if event.type == evdev.encodes.EV_ABS:
            absevent = evdev.categorize(event)
            print(f"Absolute event: code = {absevent.event.code}, value = {absevent.event.value}")
        elif event.type == evdev.encodes.EV_KEY:
            keyevent = evdev.categorize(event)
            print(f"Key event: code = {keyevent.event.code}, value = {keyevent.event.value}")
        elif event.type == evdev.encodes.EV_SYN:
            print("Sync event")
except KeyboardInterrupt:
    print("Exiting...")
finally:
    device.close()
    print("Input device closed")