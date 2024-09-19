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

    # Calculate time between samples
    sample_interval = 1 / sample_rate
    start_time = time.time()
    
    for i in range(num_samples):
        sample_value = chan0.value
        samples.append(sample_value)

        # Calculate elapsed time and adjust sleep duration
        elapsed_time = time.time() - start_time
        sleep_time = sample_interval - elapsed_time + (i * sample_interval)
        if sleep_time > 0:
            time.sleep(sleep_time)

    # Convert samples to numpy array for analysis
    samples = np.array(samples)

    # Normalize samples to range [0, 1]
    samples_normalized = (samples - np.min(samples)) / (np.max(samples) - np.min(samples))

    # Calculate slopes between consecutive normalized samples
    slopes = np.diff(samples_normalized)

    # Compute the standard deviation and mean of slopes
    std_dev_slopes = np.std(slopes)
    mean_slopes = np.mean(np.abs(slopes))

    print(f"Slope std dev: {std_dev_slopes}, Mean slopes: {mean_slopes}")

    # Square Wave Detection
    if np.any(np.abs(slopes) > 0.99):
        return "Square Wave", None

    # Triangle Wave Detection
    if std_dev_slopes < 0.1:
        return "Triangle Wave", None

    # Sine Wave Detection: Check curvature
    second_derivative = np.diff(slopes)
    std_dev_curvature = np.std(second_derivative)

    if std_dev_curvature > 0.05:  # Adjust this threshold as needed
        return "Sine Wave", None

    return "Unknown", None

def main():
    while True:
        wave_type, _ = find_waveform_shape()
        print(f"Waveform Type: {wave_type}")
        time.sleep(1)

if __name__ == "__main__":
    main()
