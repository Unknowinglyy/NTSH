import serial

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)

try:
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line:
            print(f"Received: {line}")

except KeyboardInterrupt:
    print("Exiting Program")
finally:
    ser.close()
    print("Serial port closed")