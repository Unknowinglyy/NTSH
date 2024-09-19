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

# Create an analog input channel on pin 1
chan1 = AnalogIn(mcp, MCP.P1)

def sample_signal(sample_rate=1000, duration=1):
    num_samples = sample_rate * duration
    samples = []

    # Collect samples
    start_time = time.time()
    while len(samples) < num_samples:
        # Read the value from channel 1
        samples.append(chan1.value)  # Read raw value
        time.sleep(1 / sample_rate)
    
    # Convert samples to numpy array
    samples = np.array(samples, dtype=np.float64)

    # Normalize to the MCP3008 range (0-1023)
    samples = (samples / 1023.0) * 3.3  # Assuming a 3.3V reference

    return samples

def normalize_signal(samples):
    return (samples - np.min(samples)) / (np.max(samples) - np.min(samples))

def identify_wave(samples):
    normalized_samples = normalize_signal(samples)
    derivative = np.diff(normalized_samples)

    # Check if square wave (sharp edges)
    if len(np.unique(samples)) <= 10:
        return "Square Wave"
    
    # Check if triangle wave (linear segments)
    if np.all(np.abs(derivative) < 0.62):
        return "Triangle Wave"
    
    # Check if sine wave (smooth curve)
    if np.all(np.abs(derivative) < 0.99) and np.any(np.abs(derivative) > 0.01):
        return "Sine Wave"
    
    return "Unknown Wave"

def main():
    while True:
        samples = sample_signal()
        wave = identify_wave(samples)
        print(f"Wave: {wave}\n")
        time.sleep(0.1)

if __name__ == "__main__":
    main()
