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

def measure_voltage(sample_rate=10):
    previous_voltage = None  # Initialize previous voltage
    count = 0  # Initialize count for readings
    total_change = 0.0  # Initialize total change

    while True:
        voltage = chan0.voltage
        change = voltage - previous_voltage if previous_voltage is not None else 0.0
        change_mag = np.fabs(change)

        # Add |change| to total change
        total_change += change_mag
        
        print(f"Voltage: {voltage:.2f} V, |Change|: {change_mag:.2f} V")
        
        previous_voltage = voltage  # Update previous voltage
        count += 1
        
        if count % 10 == 0:  # Every 10 readings
            average_change = total_change / 10  # Calculate average change
            if average_change == 0 or average_change <= 0.05:
                print("NO WAVE")
            print(f"Average Change (last 10 readings): {average_change:.2f} V")
            print("-" * 40)  # Output a line of dashes
            total_change = 0.0  # Reset total change for the next 10 readings

        time.sleep(1 / sample_rate)  # Wait for the next sample

def main():
    measure_voltage(sample_rate=10)  # 10 samples per second

if __name__ == "__main__":
    main()
