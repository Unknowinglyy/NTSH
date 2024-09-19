import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import numpy as np
import keyboard
from find_freq import find_frequency

# Create SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Create CS (chip select)
cs = digitalio.DigitalInOut(board.D25)

# Create MCP object
mcp = MCP.MCP3008(spi, cs)

# Create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

def sample_signal(sample_rate = 1000, duration = 1):
    num_samples = sample_rate * duration
    samples = []

    #collect samples
    start_time = time.time()
    while len(samples) < num_samples:
        samples.append(chan0.value)
        time.sleep(1/sample_rate)
    
    #convert samples to numpy array
    samples = np.array(samples, dtype=np.float64)

    #remove DC offset by subtracting mean
    samples -= np.mean(samples)

    return samples

def normalize_signal(samples):
    return (samples - np.min(samples)) / (np.max(samples) - np.min(samples))

def identify_wave(samples, sample_rate):
    normalized_samples = normalize_signal(samples)
    derivative = np.diff(normalized_samples)
    absolute_derivative = np.abs(derivative)
    derivative_range = np.ptp(derivative)
    frequency = find_frequency(sample_rate)

    print(f"Derivative: {derivative}\n")

    print(f"Frequency: {frequency}\n")

    print(f"Derivative Range: {derivative_range}\n")

    #check if square wave (sharp edges)
    if np.any(absolute_derivative > 0.99):
        return "Square Wave"
    
    # Check if triangle wave (linear segments) or sine wave (smooth curve)
    if (frequency < 50):
        if np.all(absolute_derivative < 0.6) and np.any(absolute_derivative > 0.01):
            return "Sine Wave"
        elif np.all(absolute_derivative < 0.6):
            return "Triangle Wave"
    else:
        return "Unknown Wave"
    
def main():
    while True:
        samples = sample_signal()
        wave = identify_wave(samples, 1000)
        print(f"Wave: {wave}\n")
        time.sleep(0.1)

if __name__ == "__main__":
    main()