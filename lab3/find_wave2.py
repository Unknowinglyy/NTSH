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

    # Collect samples
    start_time = time.time()
    while len(samples) < num_samples:
        samples.append(chan0.value)
        time.sleep(1 / sample_rate)

    # Convert samples to numpy array
    samples = np.array(samples)

    # Normalize samples to range [0, 1] for positive-only waveforms
    samples = (samples - np.min(samples)) / (np.max(samples) - np.min(samples))

    # Calculate the change in voltage between consecutive samples
    change_in_voltage = np.diff(samples)
    std_dev_change_in_voltage = np.std(change_in_voltage)

    # Check for square wave: distinct high and low levels
    unique_vals = np.unique(samples)
    if len(unique_vals) <= 3:
        return "Square", None

    # Check if the waveform is a triangle wave
    if std_dev_change_in_voltage < 0.1:  # Threshold for slope consistency
        rising_edge_count = np.sum(np.diff(samples) > 0)
        falling_edge_count = np.sum(np.diff(samples) < 0)
        if abs(rising_edge_count - falling_edge_count) < 0.1 * num_samples:
            return "Triangle", None

    # For sine wave, check smoothness and oscillation pattern
    mean_change = np.mean(np.abs(np.diff(samples)))
    if mean_change < 0.2:  # Adjust threshold based on waveform characteristics
        return "Sine", None

    return "Unknown", None

def main():
    while True:
        wave_type, _ = find_waveform_shape()
        print(f"Waveform Type: {wave_type}")
        time.sleep(1)  # Adjust the sleep time if necessary

if __name__ == "__main__":
    main()
