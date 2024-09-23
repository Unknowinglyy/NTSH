import board
import busio
import adafruit_mpu6050
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from time import sleep, perf_counter

i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

step_threshold = 10.0
step_interval = 0.5

last_step_time = 0
step_count = 0

time_data = []
accel_x_data = []
accel_y_data = []
accel_z_data = []

step_time_data = []
step_accel_x_data = []
step_accel_y_data = []
step_accel_z_data = []

def detect_step(accel_z, current_time):
    global last_step_time, step_count
    if accel_z > step_threshold and (current_time - last_step_time) > step_interval:
        step_count += 1
        last_step_time = current_time
        print(f"Step detected! Total steps: {step_count}")
    
    step_time_data.append(current_time)
    step_accel_z_data.append(accel_x_data[-1])
    step_accel_y_data.append(accel_y_data[-1])
    step_accel_z_data.append(accel_z_data[-1])

    if len(step_time_data) > 50:
        step_time_data.pop(0)
        step_accel_x_data.pop(0)
        step_accel_y_data.pop(0)
        step_accel_z_data.pop(0)

def main():
    global step_count
    print("Starting step counter...")

    try:
        while True:
            accel_x, accel_y, accel_z = mpu.acceleration
            current_time = perf_counter()

            time_data.append(current_time)
            accel_x_data.append(accel_x)
            accel_y_data.append(accel_y)
            accel_z_data.append(accel_z)

            detect_step(accel_z, current_time)
            sleep(0.01)
    except KeyboardInterrupt:
        print(f"Total steps: {step_count}")
        plot_steps()

def plot_steps():
    plt.figure()
    plt.plot(step_time_data, step_accel_x_data, label='Step Accel X')
    plt.plot(step_time_data, step_accel_y_data, label='Step Accel Y')
    plt.plot(step_time_data, step_accel_z_data, label='Step Accel Z')

    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s^2)')
    plt.legend(loc='upper right')
    plt.title('Acceleration Data for the Last 50 Steps')
    plt.show()

if __name__ == "__main__":
    main()
