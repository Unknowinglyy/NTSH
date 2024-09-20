import numpy as np
import spidev
import time
from scipy.stats import linregress

# MCP3008 setup
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0
spi.max_speed_hz = 1350000  # Set speed to 1.35MHz

# Parameters
num_samples = 1000  # Number of samples to analyze
voltage_readings = []

try:
    while True:
        # Read voltage from MCP3008 (channel 0)
        adc_value = spi.xfer2([1, (8 << 4), 0])  # Start bit + channel
        voltage = ((adc_value[1] & 3) << 8) + adc_value[2]
        voltage = voltage * (3.3 / 1023)  # Convert to voltage (assuming 3.3V reference)

        # Store the reading
        voltage_readings.append(voltage)

        # Once we have enough samples, perform analysis
        if len(voltage_readings) >= num_samples:
            # Analyze linearity (using linear regression)
            indices = np.arange(len(voltage_readings))
            slope, intercept, r_value, p_value, std_err = linregress(indices, voltage_readings)

            # Calculate standard deviation
            std_dev = np.std(voltage_readings)

            # Calculate linearity as R-squared value
            linearity = r_value**2

            # Output results
            print(f"Linearity (R-squared): {linearity:.4f}")
            print(f"Standard Deviation: {std_dev:.4f} V")

            # Clear the readings for the next batch
            voltage_readings = []

        # Sleep for a short period to control the reading frequency
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopping the script.")

finally:
    spi.close()
