import serial

serial_port = 'COM3'
baud_rate = 9600

ser = serial.Serial(serial_port, baud_rate, timeout=1)

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