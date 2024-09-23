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

def detect_step(accel_z, current_time):
    global last_step_time, step_count
    if accel_z > step_threshold and (current_time - last_step_time) > step_interval:
        step_count += 1
        last_step_time = current_time
        print(f"Step detected! Total steps: {step_count}")

def update_plot(frame):
    global step_count
    accel_x, accel_y, accel_z = mpu.acceleration
    current_time = perf_counter()

    time_data.append(current_time)
    accel_x_data.append(accel_x)
    accel_y_data.append(accel_y)
    accel_z_data.append(accel_z)

    if len(time_data) > 100:
        time_data.pop(0)
        accel_x_data.pop(0)
        accel_y_data.pop(0)
        accel_z_data.pop(0)

    detect_step(accel_z, current_time)
    
    ax.clear()
    ax.plot(time_data, accel_x_data, lalel='Accel X')
    ax.plot(time_data, accel_y_data, label='Accel Y')
    ax.plot(time_data, accel_z_data, label='Accel Z')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Acceleration (m/s^2)')
    ax.legend(loc='upper right')

def main():
    global step_count
    print("Starting step counter...")

    # Set up the plot
    fig, ax = plt.subplots()
    ani = animation.FuncAnimation(fig, update_plot, interval=100)

    try:
        plt.show()
    except KeyboardInterrupt:
        print(f"Total steps detected: {step_count}")

if __name__ == "__main__":
    main()
