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

def find_frequency(sample_rate=1000, duration=1):
    num_samples = sample_rate * duration
    samples = []

    # Collect samples
    start_time = time.time()
    while len(samples) < num_samples:
        samples.append(chan0.value)
        time.sleep(1 / sample_rate)

    # Convert samples to numpy array
    samples = np.array(samples)

    # Normalize samples: scale to the range [0, 1] and then shift to [-0.5, 0.5]
    samples = (samples - np.min(samples)) / (np.max(samples) - np.min(samples))
    samples = samples - 0.5  # Shift to [-0.5, 0.5]

    # Perform FFT
    fft_result = np.fft.fft(samples)
    freqs = np.fft.fftfreq(len(fft_result), 1 / sample_rate)

    # Compute magnitude spectrum
    magnitude = np.abs(fft_result)

    # Find the peak frequency in the positive half of the spectrum
    half_range = len(freqs) // 2
    peak_freq = freqs[:half_range][np.argmax(magnitude[:half_range])]

    return abs(peak_freq)

def main():
    while True:
        freq = find_frequency()
        print(f"Frequency: {freq} Hz")
        time.sleep(1)  # Adjust the sleep time if necessary

if __name__ == "__main__":
    main()
