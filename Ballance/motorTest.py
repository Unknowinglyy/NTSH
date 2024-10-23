import time
import RPi.GPIO as GPIO

CW = 1
CCW = 0
Dir = 23
Stp = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(Dir, GPIO.OUT)
GPIO.setup(Stp, GPIO.OUT)

GPIO.output(Dir, CW)
while True:
	GPIO.output(Stp, GPIO.HIGH)
	time.sleep(0.01)
	GPIO.output(Stp, GPIO.LOW)
	time.sleep(0.01)
