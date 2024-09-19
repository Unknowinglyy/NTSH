import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import numpy as np  # Import numpy for RMS calculation

# Create SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Create CS (chip select)
cs = digitalio.DigitalInOut(board.D25)

# Create MCP object
mcp = MCP.MCP3008(spi, cs)

# Create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

def measure_voltage(sample_rate=1000):
    previous_voltage = None
    voltage_changes = []  # List to store changes in voltage
    voltages = []  # List to store raw voltage readings

    while True:
        for _ in range(sample_rate):  # Collect samples
            voltage = chan0.voltage
            current_time = time.time()
            
            # Calculate the change in voltage
            if previous_voltage is not None:
                voltage_change = voltage - previous_voltage
                voltage_changes.append(voltage_change)
            else:
                voltage_change = 0
            
            voltages.append(voltage)
            previous_voltage = voltage
            
            # Wait for the next sample
            time.sleep(1 / sample_rate)

        # After collecting samples, calculate stats
        std_dev_changes = np.std(voltage_changes) if len(voltage_changes) > 1 else 0
        rms_value = np.sqrt(np.mean(np.square(voltages)))  # Calculate RMS

        # Print the results
        print(f"Standard Deviation of Voltage Changes: {std_dev_changes:.4f} V")
        print(f"RMS Voltage: {rms_value:.4f} V")

        # Determine the wave type
        if len(np.unique(samples)) <= 10:
            wave_type = "Square Wave"
        elif rms_value <= 1.2:
            wave_type = "Triangle Wave"
        else:
            wave_type = "Sine Wave"

        print(f"Detected Wave Type: {wave_type}")
        print("-" * 40)  # Separator for clarity

        # Clear the lists for the next batch of samples
        voltage_changes.clear()
        voltages.clear()

def main():
    measure_voltage(sample_rate=1000)  # 1000 samples per second

if __name__ == "__main__":
    main()
