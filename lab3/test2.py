import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# Create SPI bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# Create CS (chip select)
cs = digitalio.DigitalInOut(board.D25)

# Create MCP object
mcp = MCP.MCP3008(spi, cs)

# Create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

def measure_voltage(sample_rate=1):
    previous_voltage = None

    while True:
        # Measure the voltage
        voltage = chan0.voltage
        current_time = time.time()
        
        # Calculate the change in voltage
        if previous_voltage is not None:
            voltage_change = voltage - previous_voltage
        else:
            voltage_change = 0  # No change for the first sample
        
        # Print the measured voltage alongside the time it was measured
        print(f"Time: {current_time:.2f}, Voltage: {voltage:.2f} V (Change: {voltage_change:.2f} V)")
        
        # Update the previous voltage
        previous_voltage = voltage
        
        # Wait for the next sample
        time.sleep(1 / sample_rate)
    print("-------------------------------------------------------------------")
def main():
    measure_voltage(sample_rate=10)  # 10 samples per second

if __name__ == "__main__":
    main()
