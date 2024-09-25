import board
import busio
import adafruit_mpu6050
import matplotlib.pyplot as plt
from time import perf_counter, sleep
from math import sqrt

# initialize I2C and MPU6050
i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

# parameters for step detection
step_threshold = 10.0
# minimum time between steps in seconds
step_interval = 0.5

# variables to keep track of steps
last_step_time = 0
step_count = 0

# lists to store data for plotting
time_data = []
accel_x_data = []
accel_y_data = []
accel_z_data = []

# lists to store step data for plotting
step_time_data = []
step_accel_x_data = []
step_accel_y_data = []
step_accel_z_data = []

start_time = perf_counter()

def detect_step(accel_x ,accel_y, accel_z, current_time):
    global last_step_time, step_count
    total_accel = sqrt(accel_x**2 + accel_y**2 + accel_z**2)
    if total_accel > step_threshold and (current_time - last_step_time) > step_interval:
        step_count += 1
        last_step_time = current_time
        print(f"Step detected! Total steps: {step_count}")

        # store step data
        step_time_data.append(current_time - start_time)
        step_accel_x_data.append(accel_x_data[-1])
        step_accel_y_data.append(accel_y_data[-1])
        step_accel_z_data.append(accel_z_data[-1])

        # limit the size of the lists to the last 50 steps
        if len(step_time_data) > 50:
            step_time_data.pop(0)
            step_accel_x_data.pop(0)
            step_accel_y_data.pop(0)
            step_accel_z_data.pop(0)

def main():
    global step_count, start_time
    print("Starting step counter...")

    try:
        while True:
            accel_x, accel_y, accel_z = mpu.acceleration
            current_time = perf_counter()

            time_data.append(current_time - start_time)
            accel_x_data.append(accel_x)
            accel_y_data.append(accel_y)
            accel_z_data.append(accel_z)

            detect_step(accel_x, accel_y, accel_z, current_time)
            sleep(0.01)
    except KeyboardInterrupt:
        print(f"Total steps: {step_count}")
        plot_steps()

def plot_steps():
    # plot the acceleration data for the last 50 steps
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