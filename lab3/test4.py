import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import numpy as np  # Import numpy for calculations
from scipy.stats import linregress  # Import for linear regression analysis
from scipy.fft import fft

# Create SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Create CS (chip select)
cs = digitalio.DigitalInOut(board.D25)

# Create MCP object
mcp = MCP.MCP3008(spi, cs)

# Create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

import numpy as np
import numpy as np

def analyze_waveform(voltages):
    max_voltage = np.max(voltages)
    rms_value = np.sqrt(np.mean(np.square(voltages)))
    average_value = np.mean(voltages)

    # Calculate shape factor
    shape_factor = rms_value / average_value if average_value != 0 else 0

    # Calculate skewness and kurtosis
    skewness = (((np.mean((voltages - average_value)**3)) / 
                (np.std(voltages)**3)) if np.std(voltages) != 0 else 0)
    kurtosis = (((np.mean((voltages - average_value)**4)) / 
                (np.std(voltages)**4)) if np.std(voltages) != 0 else 0)
    
    print(f"Shape Factor: {shape_factor:.4f}, Skewness: {skewness:.4f}, Kurtosis: {kurtosis:.4f}")

    # Criteria for differentiation
    if shape_factor > 1.0 and skewness < 0.5:
        print("The waveform is likely a Sine wave.")
    elif shape_factor < 0.6 and skewness > 0.5:
        print("The waveform is likely a Triangle wave.")
    else:
        print("The waveform is ambiguous.")

def measure_voltage(sample_rate=2000, num_samples=2000):
    previous_voltage = None
    voltages = []  # List to store raw voltage readings
    slopes = []    # List to store slopes between samples
    indices = []   # List to store index values for linearity analysis

    while True:
        for i in range(num_samples):  # Collect specified number of samples
            voltage = chan0.voltage
            voltages.append(voltage)
            indices.append(i)  # Store the index for regression analysis

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

        # Linear regression analysis for linearity
        if len(voltages) > 1:
            slope, intercept, r_value, p_value, std_err = linregress(indices, voltages)
            linearity = r_value**2
        else:
            linearity = 0

        # Print the results
        print(f"Linearity (R-squared): {linearity:.4f}")
        print(f"Standard Deviation of Slopes: {std_dev_slopes:.4f} V/s")
        print(f"Maximum Voltage: {max_voltage:.4f} V")
        print(f"Expected RMS Voltage (based on max): {expected_rms_max:.4f} V")
        print(f"Actual RMS Voltage (based on samples): {actual_rms:.4f} V")
        print(f"Expected RMS - Actual RMS: {expected_rms_max - actual_rms:.4f} V")
        print("-" * 40)  # Separator for clarity
        analyze_waveform(voltages)

        # Clear the lists for the next batch of samples
        slopes.clear()
        voltages.clear()
        indices.clear()

def main():
    measure_voltage(sample_rate=1000, num_samples=1000)  # 1000 samples per second

if __name__ == "__main__":
    main()
