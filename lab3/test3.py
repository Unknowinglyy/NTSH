import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import numpy as np  # Import numpy for calculations

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
    voltages = []  # List to store raw voltage readings
    slopes = []    # List to store slopes between samples

    while True:
        for _ in range(sample_rate):  # Collect 1000 samples
            voltage = chan0.voltage
            voltages.append(voltage)

            # Calculate the slope
            if previous_voltage is not None:
                slope = voltage - previous_voltage
                slopes.append(slope)
            
            previous_voltage = voltage
            
            # Wait for the next sample
            time.sleep(1 / sample_rate)

        # After collecting samples, calculate stats
        std_dev_slopes = np.std(slopes) if len(slopes) > 1 else 0
        max_voltage = np.max(voltages) if voltages else 0

        # Calculate expected RMS value based on max voltage
        expected_rms_max = max_voltage / np.sqrt(2) if max_voltage > 0 else 0
        
        # Calculate actual RMS value from the sample voltages
        actual_rms = np.sqrt(np.mean(np.square(voltages))) if voltages else 0

        # Print the results
        print(f"Standard Deviation of Slopes: {std_dev_slopes:.4f} V/s")
        print(f"Maximum Voltage: {max_voltage:.4f} V")
        print(f"Expected RMS Voltage (based on max): {expected_rms_max:.4f} V")
        print(f"Actual RMS Voltage (based on samples): {actual_rms:.4f} V")
        print("-" * 40)  # Separator for clarity

        # Clear the lists for the next batch of samples
        slopes.clear()
        voltages.clear()

def main():
    measure_voltage(sample_rate=1000)  # 1000 samples per second

if __name__ == "__main__":
    main()
