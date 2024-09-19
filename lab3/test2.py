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

def measure_voltage(sample_rate=100, duration=1):
    num_samples = sample_rate * duration
    samples = []
    
    previous_voltage = None

    for sample_number in range(num_samples):
        # Measure the voltage
        voltage = chan0.voltage
        current_time = time.time()
        
        # Calculate the change in voltage
        if previous_voltage is not None:
            voltage_change = voltage - previous_voltage
        else:
            voltage_change = 0
        
        # Store the voltage sample
        samples.append(voltage)
        
        # Update the previous voltage
        previous_voltage = voltage
        
        # Wait for the next sample
        time.sleep(1 / sample_rate)

        # Print a dashed line after every 20 samples
        if (sample_number + 1) % 20 == 0:
            print(f"Sample {sample_number + 1}: Voltage = {voltage:.2f} V (Change: {voltage_change:.2f} V)")
            print("-" * 40)

    return np.array(samples)

def analyze_waveform(samples):
    # Normalize samples to range [0, 1]
    samples_normalized = (samples - np.min(samples)) / (np.max(samples) - np.min(samples))

    # Calculate slopes between consecutive samples
    slopes = np.diff(samples_normalized)
    std_dev_slopes = np.std(slopes)
    
    # Harmonics Analysis
    fft_values = np.fft.fft(samples_normalized)
    magnitudes = np.abs(fft_values)
    
    # Count significant harmonics (set a threshold)
    harmonic_threshold = 0.1 * np.max(magnitudes)
    harmonics_count = np.count_nonzero(magnitudes[2:] > harmonic_threshold)

    # Waveform classification
    if np.any(np.abs(slopes) > 0.99):
        return "Square Wave"
    elif std_dev_slopes < 0.1 and harmonics_count > 2:
        return "Triangle Wave"
    elif harmonics_count <= 2:
        return "Sine Wave"
    
    return "Unknown"

def main():
    while True:
        samples = measure_voltage(sample_rate=100, duration=1)  # 100 samples for 1 second
        wave_type = analyze_waveform(samples)
        print(f"Waveform Type: {wave_type}")
        time.sleep(1)  # Wait before the next measurement cycle

if __name__ == "__main__":
    main()
