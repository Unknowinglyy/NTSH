import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import numpy as np  # Import numpy for standard deviation calculation

# Create SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Create CS (chip select)
cs = digitalio.DigitalInOut(board.D25)

# Create MCP object
mcp = MCP.MCP3008(spi, cs)

# Create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

def measure_voltage(sample_rate=100):
    previous_voltage = None
    voltage_changes = []  # List to store changes in voltage
    samples_per_batch = sample_rate  # Number of samples after which to print a line

    while True:
        for sample_number in range(sample_rate):  # Loop for the number of samples per second
            # Measure the voltage
            voltage = chan0.voltage
            current_time = time.time()
            
            # Calculate the change in voltage
            if previous_voltage is not None:
                voltage_change = voltage - previous_voltage
                voltage_changes.append(voltage_change)  # Store the change
            else:
                voltage_change = 0
            
            # Print the measured voltage alongside the time it was measured
            print(f"Time: {current_time:.2f}, Voltage: {voltage:.2f} V (Change: {voltage_change:.2f} V)")
            
            # Update the previous voltage
            previous_voltage = voltage
            
            # Wait for the next sample
            time.sleep(1 / sample_rate)

            # Print a dashed line after every 20 samples
            if (sample_number + 1) % samples_per_batch == 0:
                print("-" * 40)

        # Calculate the standard deviation of the voltage changes after each batch
        if len(voltage_changes) > 1:  # Ensure there's enough data to compute std deviation
            std_dev_changes = np.std(voltage_changes)
            print(f"Standard Deviation of Voltage Changes: {std_dev_changes:.4f} V")

def main():
    measure_voltage(sample_rate=2000)  # 2000 samples per second

if __name__ == "__main__":
    main()
