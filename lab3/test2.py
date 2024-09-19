import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import numpy as np  # Import numpy for RMS calculation

# Create SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Create CS (chip select)
cs = digitalio.DigitalInOut(board.D25)

# Create MCP object
mcp = MCP.MCP3008(spi, cs)

# Create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

def measure_voltage(sample_rate=1000):
    previous_voltage = None
    voltage_changes = []  # List to store changes in voltage
    voltages = []  # List to store raw voltage readings

    while True:
        # Measure the voltage
        voltage = chan0.voltage
        current_time = time.time()
        
        # Calculate the change in voltage
        if previous_voltage is not None:
            voltage_change = voltage - previous_voltage
            voltage_changes.append(voltage_change)  # Store the change
        else:
            voltage_change = 0
        
        voltages.append(voltage)  # Store the voltage reading
        previous_voltage = voltage
        
        # Print the measured voltage alongside the time it was measured
        print(f"Time: {current_time:.2f}, Voltage: {voltage:.2f} V (Change: {voltage_change:.2f} V)")

        # Wait for the next sample
        time.sleep(1 / sample_rate)

        # Print a dashed line after every 20 samples
        if len(voltages) % 20 == 0:
            print("-" * 40)

        # Calculate RMS and standard deviation every 100 samples
        if len(voltages) % 100 == 0:
            std_dev_changes = np.std(voltage_changes) if len(voltage_changes) > 1 else 0
            rms_value = np.sqrt(np.mean(np.square(voltages)))  # Calculate RMS

            print(f"Standard Deviation of Voltage Changes: {std_dev_changes:.4f} V")
            print(f"RMS Voltage: {rms_value:.4f} V")

def main():
    measure_voltage(sample_rate=2000)  # 2000 samples per second

if __name__ == "__main__":
    main()
