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

def measure_voltage(sample_rate=1000, duration=5):
    previous_voltage = None
    voltage_changes = []
    voltages = []

    start_time = time.time()

    while time.time() - start_time < duration:
        voltage = chan0.voltage
        if previous_voltage is not None:
            voltage_change = voltage - previous_voltage
            voltage_changes.append(voltage_change)
        else:
            voltage_change = 0
        
        voltages.append(voltage)
        previous_voltage = voltage

        time.sleep(1 / sample_rate)

    return np.array(voltages), np.array(voltage_changes)

def analyze_waveform(voltages, voltage_changes):
    std_dev_changes = np.std(voltage_changes)
    peak_to_peak = np.max(voltages) - np.min(voltages)
    rms_value = np.sqrt(np.mean(voltages**2))  # Calculate RMS

    zero_crossings = np.where(np.diff(np.sign(voltages)))[0].size

    print(f"Standard Deviation of Voltage Changes: {std_dev_changes:.4f} V")
    print(f"Peak-to-Peak Voltage: {peak_to_peak:.4f} V")
    print(f"RMS Voltage: {rms_value:.4f} V")
    print(f"Zero Crossings Count: {zero_crossings}")

    # Heuristic for classification
    if zero_crossings > 10:
        return "Sine Wave"
    elif std_dev_changes < 0.02 and peak_to_peak < 1:
        return "Triangle Wave"
    else:
        return "Square Wave"

def main():
    voltages, voltage_changes = measure_voltage(sample_rate=2000, duration=5)
    wave_type = analyze_waveform(voltages, voltage_changes)
    print(f"Identified Waveform Type: {wave_type}")

if __name__ == "__main__":
    main()
