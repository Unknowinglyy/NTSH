import serial
import time

serial_port = 'ttyAMA0' # '/dev/ttyUSB0'
baud_rates = [9600, 19200, 38400, 57600, 115200]

for baud_rate in baud_rates:
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"Testing baud rate: {baud_rate}")
        time.sleep(2)

        ser.write(b'Test')

        line = ser.readline().decode('utf-8').strip()
        if line:
            print(f"Received: {baud_rate} : {line}")
        ser.close()

    except Exception as e:
        print(f"failed to read at {baud_rate} baud rate: {e}")
