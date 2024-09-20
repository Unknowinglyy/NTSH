import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import numpy as np
from scipy import stats  # Import stats from scipy
from find_freq2 import find_frequency

# Create SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Create CS (chip select)
cs = digitalio.DigitalInOut(board.D25)

# Create MCP object
mcp = MCP.MCP3008(spi, cs)

# Create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

def measure_voltage(sample_rate=10):
    sample_count = sample_rate
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
        voltageChangeArr.append(np.round(change_mag, 2))  # Use list for appending
        voltageChangeArr_B.append(np.round(change_mag, 0))

        # Add |change| to total change
        total_change += change_mag
        
        previous_voltage = voltage  # Update previous voltage
        count += 1
        
        if count % sample_count == 0:  # Every 50 readings
            average_change = total_change / sample_count  # Calculate average change
            print(f"Average Change (last {sample_count} readings): {average_change:.2f} V")
            
            # Convert to NumPy arrays for analysis
            voltageChangeArr_np = np.array(voltageChangeArr)
            voltageChangeArr_B_np = np.array(voltageChangeArr_B)


            print(voltageChangeArr)  # Print as NumPy array
            print(np.unique(voltageChangeArr_np))
            
            # Calculate mode
            mode_change = stats.mode(voltageChangeArr_np)
            mode_value = mode_change.mode[0]
            mode_count = mode_change.count[0]
            print(f"Mode of Changes: {mode_value:.2f} V, Count: {mode_count}")

            print(f"Max: {np.max(voltageChangeArr)}")

            if (np.round(average_change, 2) == 0) and (mode_value == 0) and (np.round((np.sum(np.unique(voltageChangeArr_np))), 0) == 0):
                print("NO WAVE")
            if (len(np.unique(voltageChangeArr_np)) >= 2 and len(np.unique(voltageChangeArr_np)) <= 7) and (average_change > 0) and np.any(voltageChangeArr_np > 1) and (np.round(mode_value, 2) == 0 or ((np.max(voltageChangeArr)) - mode_value <= 0.12)) :
                if (mode_count > 10):
                    print("SQUARE WAVE")
            if (average_change > 0 and mode_count > 8) and (len(np.unique(voltageChangeArr_np)) > 7) and (mode_value != 0):
                print("TRIANGLE WAVE")
            if (len(np.unique(voltageChangeArr_np)) > 4) and average_change > 0 and mode_count <= 9:
                if (mode_value == 0) and (mode_count < 9):
                    print("SINE WAVE")
                if (mode_value != 0):
                    print("SINE WAVE")
            find_frequency(sample_rate=75, duration=2)
            print("-" * 40)  # Output a line of dashes

            total_change = 0.0  # Reset total change for the next set of readings
            voltageChangeArr.clear()  # Clear list for the next readings
            voltageChangeArr_B.clear()
        
        time.sleep(1 / 50)  # Wait for the next sample

def main():
    measure_voltage(sample_rate=75)  # 50 samples per second

if __name__ == "__main__":
    main()
