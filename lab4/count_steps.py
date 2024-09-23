import board
import busio
import adafruit_mpu6050

from time import sleep, perf_counter

i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

step_threshold = 3.0
step_interval = 0.3

last_step_time = 0
step_count = 0

def detect_step(accel_z, current_time):
    global last_step_time, step_count
    if accel_z > step_threshold and (current_time - last_step_time) > step_interval:
        step_count += 1
        last_step_time = current_time
        print(f"Step detected! Total steps: {step_count}")

def main():
    global step_count
    print("Starting step counter...")
    while True:
        _, _, accel_z = mpu.acceleration
        current_time = perf_counter()
        detect_step(accel_z, current_time)
        sleep(0.01)

if __name__ == "__main__":
    main()