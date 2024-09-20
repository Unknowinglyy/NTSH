import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import numpy as np

# Set print options for higher precision
np.set_printoptions(precision=10, suppress=True)

# Create SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Create CS (chip select)
cs = digitalio.DigitalInOut(board.D25)

# Create MCP object
mcp = MCP.MCP3008(spi, cs)

# Create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

def find_frequency(sample_rate=1000, duration=4):
    num_samples = sample_rate * duration
    samples = []

    # Collect samples with precise timing
    start_time = time.perf_counter()
    for _ in range(num_samples):
        samples.append(chan0.voltage)
        while (time.perf_counter() - start_time) < (len(samples) / sample_rate):
            pass

    # Convert samples to numpy array
    samples = np.array(samples, dtype=np.float64)

    # Remove DC offset by subtracting mean
    samples -= np.mean(samples)

    # Apply a window function to reduce spectral leakage
    window = np.hanning(len(samples))
    windowed_samples = samples * window

    # Zero padding to increase FFT resolution
    padded_samples = np.pad(windowed_samples, (0, num_samples), 'constant')

    # Perform FFT
    fft_result = np.fft.fft(padded_samples)
    freqs = np.fft.fftfreq(len(fft_result), 1/sample_rate)

    # Find the peak frequency index
    peak_freq_index = np.argmax(np.abs(fft_result))
    peak_freq = freqs[peak_freq_index]

    # Interpolation to get a more accurate frequency
    if peak_freq_index > 0 and peak_freq_index < len(fft_result) - 1:
        # Quadratic interpolation around the peak
        y0 = np.abs(fft_result[peak_freq_index - 1])
        y1 = np.abs(fft_result[peak_freq_index])
        y2 = np.abs(fft_result[peak_freq_index + 1])
        numerator = (y2 - y0) * 0.5
        denominator = (y2 - 2 * y1 + y0)
        if denominator != 0:
            offset = numerator / denominator
            peak_freq += offset * (freqs[1] - freqs[0])  # Adjust by the frequency spacing

    # Print debug information
    # print(f"fft_result = {fft_result}\n")
    # print(f"freqs = {freqs}\n")
    # print(f"peak_freq_index = {peak_freq_index}\n")
    # print(f"peak_freq = {peak_freq:.10f}\n")  # Adjusted for higher precision

    return abs(peak_freq)

def main():
    while True:
        freq = find_frequency()
        print(f"Frequency: {freq:.10f} Hz\n")  # Adjusted for higher precision
        time.sleep(0.1)

if __name__ == "__main__":
    main()
