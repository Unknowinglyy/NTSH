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

def find_frequency(sample_rate=1000, duration=2):
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
    #print(f"Windowed Samples: {windowed_samples}")

    # Zero padding to increase FFT resolution
    padded_samples = np.pad(windowed_samples, (0, num_samples), 'constant')
    
    # Perform FFT
    fft_result = np.fft.fft(padded_samples, 3)
    print(f"fft_result = {fft_result}\n")
    freqs = np.fft.fftfreq(len(fft_result), 1/sample_rate)
    print(f"freqs = {freqs}\n")

    # Find the peak frequency
    peak_freq = freqs[np.argmax(np.abs(fft_result))]
    print(f"peak_freq = {peak_freq}\n")

    return abs(peak_freq)

def main():
    while True:
        freq = find_frequency()
        print(f"Frequency: {freq:.6f} Hz\n")
        time.sleep(0.1)

if __name__ == "__main__":
    main()