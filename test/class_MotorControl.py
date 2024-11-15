import RPi.GPIO as GPIO
import time

class MotorControl:
    def __init__(self, motor_pins):
        self.motor_pins = motor_pins
        GPIO.setmode(GPIO.BCM)
        for motor in self.motor_pins.values():
            GPIO.setup(motor['step'], GPIO.OUT)
            GPIO.setup(motor['dir'], GPIO.OUT)

    def move_motor(self, motor, steps, clockwise):
        GPIO.output(self.motor_pins[motor]['dir'], GPIO.HIGH if clockwise else GPIO.LOW)
        for _ in range(abs(steps)):
            GPIO.output(self.motor_pins[motor]['step'], GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(self.motor_pins[motor]['step'], GPIO.LOW)
            time.sleep(0.001)

    def cleanup(self):
        GPIO.cleanup()