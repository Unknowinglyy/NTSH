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

def measure_voltage(sample_rate=10):
    count = 0  # Initialize count for readings
    while True:
        voltage = chan0.voltage
        print(f"Voltage: {voltage:.4f} V")
        
        count += 1
        if count % 10 == 0:  # Every 10 readings
            print("-" * 40)  # Output a line of dashes

        time.sleep(1 / sample_rate)  # Wait for the next sample

def main():
    measure_voltage(sample_rate=10)  # 10 samples per second

if __name__ == "__main__":
    main()
