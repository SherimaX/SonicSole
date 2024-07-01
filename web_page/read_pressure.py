import spidev
import time
import RPi.GPIO as GPIO

class SonicSole:
    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)  # Open SPI bus 0, device 0
        self.spi.max_speed_hz = 1000000
        self.cs_pin = 8  # Chip Select pin (use appropriate pin for your setup)
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.cs_pin, GPIO.OUT)
        GPIO.output(self.cs_pin, GPIO.HIGH)

    def read_adc(self, channel):
        GPIO.output(self.cs_pin, GPIO.LOW)
        adc = self.spi.xfer2([1, (8 + channel) << 4, 0])
        GPIO.output(self.cs_pin, GPIO.HIGH)
        data = ((adc[1] & 3) << 8) + adc[2]
        return data

    def update_pressure(self):
        currHeelPressure = self.read_adc(0)  # Assuming heel sensor is on channel 0
        print("currHeelPressure: {}".format(currHeelPressure))

    def main_loop(self):
        try:
            while True:
                self.update_pressure()
                time.sleep(0.1)  # Delay to simulate processing time
        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            self.spi.close()
            GPIO.cleanup()

if __name__ == "__main__":
    sole = SonicSole()
    sole.main_loop()

