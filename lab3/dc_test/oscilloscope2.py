import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import numpy as np
from scipy import stats  # Import stats from scipy

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
    voltageChangeArr = []
    voltageChangeArr_B = []

    while True:
        voltage = chan0.voltage
        change = voltage - previous_voltage if previous_voltage is not None else 0.0
        change_mag = np.fabs(change)

        # Store |change|
        voltageChangeArr.append(np.round(change_mag, 2))
        voltageChangeArr_B.append(np.round(change_mag, 0))

        # Add |change| to total change
        total_change += change_mag
        
        #print(f"Voltage: {voltage:.2f} V, |Change|: {change_mag:.2f} V")
        
        previous_voltage = voltage  # Update previous voltage
        count += 1
        
        if count % 20 == 0:  # Every 20 readings
            average_change = total_change / sample_rate  # Calculate average change
            print(f"Average Change (last 20 readings): {average_change:.2f} V")
            print(voltageChangeArr)
            print(voltageChangeArr_B)
            print(np.unique(voltageChangeArr))
            
            # Calculate mode
            mode_change = stats.mode(voltageChangeArr)
            mode_value = mode_change.mode[0]
            mode_count = mode_change.count[0]
            print(f"Mode of Changes: {mode_value:.2f} V, Count: {mode_count}")

            if (np.round(average_change, 0) == 0) and (mode_value == 0) and (np.round((np.sum(np.unique(voltageChangeArr))), 0) == 0) and ((20 - mode_count) <= 3):
                print("NO WAVE")
            if (len(np.unique(voltageChangeArr)) >= 2) and (average_change > 0):
                print("SQUARE WAVE")
            if (average_change > 0 and mode_count >= 5) and (len(np.unique(voltageChangeArr)) > 3):
                print("TRIANGLE WAVE")
            if (len(np.unique(voltageChangeArr)) > 4) and average_change > 0 and mode_count < 4:
                print("SINE WAVE")
            print("-" * 40)  # Output a line of dashes

            total_change = 0.0  # Reset total change for the next 20 readings
            voltageChangeArr.clear()
            voltageChangeArr_B.clear()
        
        time.sleep(1 / 50)  # Wait for the next sample [I CHANGED THE VALUE FROM 1/sample_rate to 50]

def main():
    measure_voltage(sample_rate=20)  # 20 samples per second

if __name__ == "__main__":
    main()
