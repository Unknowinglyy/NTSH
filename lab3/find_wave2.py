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

    # Normalize samples to range [0, 1] and shift to [-0.5, 0.5]
    samples = (samples - np.min(samples)) / (np.max(samples) - np.min(samples))
    samples = samples - 0.5  # Shift to [-0.5, 0.5]

    # Calculate the difference between successive samples
    diffs = np.diff(samples)

    # Detect peaks and troughs
    peaks = np.where((samples[:-2] < samples[1:-1]) & (samples[1:-1] > samples[2:]))[0]
    troughs = np.where((samples[:-2] > samples[1:-1]) & (samples[1:-1] < samples[2:]))[0]

    num_peaks = len(peaks)
    num_troughs = len(troughs)

    # Classify based on shape characteristics
    if num_peaks > 2 and num_troughs > 2:
        # Check for square wave: many rapid transitions with nearly equal peaks and troughs
        if abs(num_peaks - num_troughs) < 2 and num_peaks > 5:
            return "Square", None
        # Check for triangle wave: periodic linear rise and fall with fewer peaks and troughs
        elif abs(num_peaks - num_troughs) < 2 and num_peaks <= 5:
            return "Triangle", None
    # Otherwise, consider it as sine wave if it's smooth and has few peaks and troughs
    if np.all(np.abs(diffs) < 0.1):
        return "Sine", None

    return "Unknown", None

def main():
    while True:
        wave_type, _ = find_waveform_shape()
        print(f"Waveform Type: {wave_type}")
        time.sleep(1)  # Adjust the sleep time if necessary

if __name__ == "__main__":
    main()
