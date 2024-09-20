import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import numpy as np

def find_frequency(sample_rate=1000, duration=4):
    num_samples = sample_rate * duration
    samples = []

    # Collect samples with precise timing
    start_time = time.perf_counter()
    for _ in range(num_samples):
        samples.append(chan0.voltage)
        while (time.perf_counter() - start_time) < (len(samples) / sample_rate):
            pass

    # Convert to numpy array and remove DC offset
    samples = np.array(samples, dtype=np.float64)
    samples -= np.mean(samples)

    # Apply a window function
    window = np.hanning(len(samples))
    windowed_samples = samples * window

    # Perform FFT
    fft_result = np.fft.fft(windowed_samples)
    freqs = np.fft.fftfreq(len(fft_result), 1/sample_rate)

    # Consider only the positive half of the spectrum
    positive_freqs = freqs[freqs >= 0]
    positive_fft_result = np.abs(fft_result[freqs >= 0])

    # Find the peak frequency index
    peak_freq_index = np.argmax(positive_fft_result)
    peak_freq = positive_freqs[peak_freq_index]

    # Print debug information
    print(f"Peak Frequency Index: {peak_freq_index}")
    print(f"Detected Peak Frequency: {peak_freq:.10f} Hz")

    return abs(peak_freq)
