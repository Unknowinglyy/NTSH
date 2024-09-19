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
    last_wave_type = None  # Variable to track the last detected wave type

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

        # Determine the wave type
        if np.any(np.abs(np.diff(voltage_changes)) > 0.99):
            wave_type = "SQUARE WAVE"
        elif rms_value <= 1.2 and rms_value > 1.0:
            wave_type = "TRIANGLE WAVE"
        elif rms_value > 1.2 and np.mean(voltages) > 0.005:
            wave_type = "SINE WAVE"
        else:
            wave_type = "UNKNOWN WAVE / TRANSITION BETWEEN TWO WAVES OCCURING"

        # Print the detected wave type only if it has changed
        if wave_type != last_wave_type:
            print(f"Detected Wave Type: {wave_type}")
            last_wave_type = wave_type  # Update last_wave_type

        print(f"Standard Deviation of Voltage Changes: {std_dev_changes:.4f} V")
        print(f"RMS Voltage: {rms_value:.4f} V")
        print(f"Avg Voltage: {np.mean(np.square(voltages))} V")
        print("-" * 40)  # Separator for clarity

        # Clear the lists for the next batch of samples
        voltage_changes.clear()
        voltages.clear()

def main():
    measure_voltage(sample_rate=1000)  # 1000 samples per second

if __name__ == "__main__":
    main()
