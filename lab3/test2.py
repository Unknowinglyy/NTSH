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

def measure_voltage(sample_rate=1, duration=10):
    num_samples = sample_rate * duration

    for _ in range(num_samples):
        # Measure the voltage
        voltage = chan0.voltage
        current_time = time.time()
        
        # Print the measured voltage alongside the time it was measured
        print(f"Time: {current_time:.2f}, Voltage: {voltage:.2f} V")
        
        # Wait for the next sample
        time.sleep(1 / sample_rate)

def main():
    measure_voltage(sample_rate=10, duration=1)  # 10 samples in 1 second

if __name__ == "__main__":
    main()
