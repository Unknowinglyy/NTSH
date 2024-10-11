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
    std_dev_slopes = np.std(slopes)

    print(f"Slope std dev: {std_dev_slopes}")

    # Square Wave Detection
    if np.any(np.abs(slopes) > 0.99):
        return "Square Wave", None

    # Harmonics Analysis
    fft_values = np.fft.fft(samples_normalized)
    magnitudes = np.abs(fft_values)
    
    # Determine number of significant harmonics with stricter threshold
    harmonic_threshold = 0.2 * np.max(magnitudes)  # Adjust this as needed
    
    
    # Determine number of significant harmonics
    fundamental_freq_index = np.argmax(magnitudes[1:]) + 1
    harmonics_count = np.count_nonzero(magnitudes[2:] > harmonic_threshold)

    print(f"Fundamental Frequency Index: {fundamental_freq_index}, Number of Significant Harmonics: {harmonics_count}")

    # Triangle Wave Detection
    if std_dev_slopes < 0.1 and harmonics_count > 2:  # Triangle waves have harmonics
        return "Triangle Wave", None

    # Sine Wave Detection
    if harmonics_count <= 2:  # Sine waves have only the fundamental frequency
        return "Sine Wave", None

    return "Unknown", None

def main():
    while True:
        wave_type, _ = find_waveform_shape()
        print(f"Waveform Type: {wave_type}")
        time.sleep(1)

if __name__ == "__main__":
    main()
