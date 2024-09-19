import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import numpy as np

# Create SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Create CS (chip select)
cs = digitalio.DigitalInOut(board.D25)

# Create MCP object
mcp = MCP.MCP3008(spi, cs)

# Create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

def find_waveform_shape(sample_rate=1000, duration=1):
    num_samples = sample_rate * duration
    samples = []
    timestamps = []

    # Calculate time between samples
    sample_interval = 1 / sample_rate
    start_time = time.time()
    
    for i in range(num_samples):
        sample_value = chan0.value
        current_time = time.time() - start_time
        
        samples.append(sample_value)
        timestamps.append(current_time)

        # Print the sample value, timestamp, change in voltage, and change in time
        if i > 0:
            change_in_voltage = samples[i] - samples[i - 1]
            change_in_time = timestamps[i] - timestamps[i - 1]
            #print(f"Sample {i}: Value = {sample_value}, Time = {current_time:.6f}s, "
            #      f"Change in Voltage = {change_in_voltage}, Change in Time = {change_in_time:.6f}s")
        else:
            #print(f"Sample {i}: Value = {sample_value}, Time = {current_time:.6f}s, "
            #      f"Change in Voltage = N/A, Change in Time = N/A")

        # Calculate elapsed time and adjust sleep duration
        elapsed_time = time.time() - start_time
        sleep_time = sample_interval - (elapsed_time - (i * sample_interval))
        if sleep_time > 0:
            time.sleep(sleep_time)

    # Convert samples to numpy array for analysis
    samples = np.array(samples)

    # Normalize samples to range [0, 1] for positive-only waveforms
    samples_normalized = (samples - np.min(samples)) / (np.max(samples) - np.min(samples))

    # Calculate slopes between consecutive normalized samples
    slopes = np.diff(samples_normalized)

    # Compute the standard deviation of slopes
    std_dev_slopes = np.std(slopes)

    print(f"Slope std dev: {std_dev_slopes}")
    if std_dev_slopes < 0.1:
        return "Triangle", None

    return "Unknown", None

def main():
    while True:
        wave_type, _ = find_waveform_shape()
        print(f"Waveform Type: {wave_type}")
        time.sleep(1)

if __name__ == "__main__":
    main()
