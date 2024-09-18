import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import numpy as np
import keyboard

# Create SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Create CS (chip select)
cs = digitalio.DigitalInOut(board.D25)

# Create MCP object
mcp = MCP.MCP3008(spi, cs)

# Create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

def find_frequency(sample_rate = 1000, duration = 1):
    num_samples = sample_rate * duration
    samples = []

    #collect samples
    start_time = time.perf_counter()
    for _ in range(num_samples):
        samples.append(chan0.voltage)
        while(time.perf_counter() - start_time) < (len(samples) / sample_rate): 
            pass

    #convert samples to numpy array
    samples = np.array(samples)

    #remove DC offset by subtracting mean
    samples -= np.mean(samples)

    #perform fft
    fft_result = np.fft.fft(samples)
    print(f"fft_result = {fft_result}\n")
    freqs = np.fft.fftfreq(len(fft_result), 1/sample_rate)
    print(f"freqs = {freqs}\n")

    #find the peak frequency
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
