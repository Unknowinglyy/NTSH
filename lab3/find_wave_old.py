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
    derivative_range = np.max(np.abs(derivative)) - np.min(np.abs(derivative))

    #check if square wave (sharp edges)
    if np.any(np.abs(derivative) > 0.99):
        return "Square Wave"
    
    frequency = find_frequency(sample_rate)

    if frequency >= 1 and frequency <= 10:
        if(derivative_range >= 0.003 and derivative_range <= 0.016):
            return "Triangle Wave"
        elif(derivative_range >= 0 and derivative_range <= 0.004):
            return "Sine Wave"
    
    if frequency >= 10 and frequency <= 20:
        if(derivative_range >= 0.003 and derivative_range <= 0.016):
            return "Triangle Wave"
        elif(derivative_range >= 0 and derivative_range <= 0.019):
            return "Sine Wave"
        
    if frequency >= 20 and frequency <= 50:
        if(derivative_range >= 0.003 and derivative_range <= 0.16):
            return "Triangle Wave"
        elif(derivative_range >= 0.018 and derivative_range <= 0.21):
            return "Sine Wave"
    
    return "Unknown Wave"
    
def main():
    while True:
        samples = sample_signal()
        wave = identify_wave(samples, 1000)
        print(f"Wave: {wave}\n")
        time.sleep(0.1)

if __name__ == "__main__":
    main()