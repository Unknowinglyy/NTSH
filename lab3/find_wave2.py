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

    # Calculate slopes between consecutive samples
    slopes = np.diff(samples)

    # Compute the standard deviation of slopes
    std_dev_slopes = np.std(slopes)

    # Check if the waveform is a triangle wave
    if std_dev_slopes < 0.1:  # This threshold is for illustration; adjust as needed
        return "Triangle", None

    # Check periodicity and symmetry for triangle waves
    peaks = np.where((samples[:-2] < samples[1:-1]) & (samples[1:-1] > samples[2:]))[0]
    troughs = np.where((samples[:-2] > samples[1:-1]) & (samples[1:-1] < samples[2:]))[0]
    
    if len(peaks) >= 2 and len(troughs) >= 2:
        peak_to_peak_distances = np.diff(peaks)
        trough_to_trough_distances = np.diff(troughs)
        
        # Check if the distances between peaks and troughs are consistent
        if np.std(peak_to_peak_distances) < 0.1 and np.std(trough_to_trough_distances) < 0.1:
            return "Triangle", None
    elif len(np.unique(samples)) <= 10:
        return "Square", None
    # If not a triangle wave, classify based on other criteria
    return "Unknown", None

def main():
    while True:
        wave_type, _ = find_waveform_shape()
        print(f"Waveform Type: {wave_type}")
        time.sleep(1)  # Adjust the sleep time if necessary

if __name__ == "__main__":
    main()
