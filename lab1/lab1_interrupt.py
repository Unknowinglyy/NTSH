from gpiozero import RGBLED, Button, LEDCharDisplay
from time import sleep  
import threading
    
#create a led object with the correct pins
led1 = RGBLED(red=9, green=10, blue=11)

led2 = RGBLED(red=7, green=8, blue=15)

button = Button(2)

display = LEDCharDisplay(a=3, b=4, c=17, d=18, e=14, f=22, g=23, dp=24)

def change_display():
    #count down from 9 to 0
    for char in '9876543210':
        display.value = char
        sleep(1)
    #display turns off
    display.off()

#led1 should be red at first
led1.color = (1, 0, 0)
#led2 should be green at first
led2.color = (0, 1, 0)


while True:
    #when button is pressed, led2 turns blue, blinks 3 times, then turns red
    #using wait_for_press which is an interrupt
    button.wait_for_press()
    led2.color = (0, 0, 1)
    led2.blink(on_time=0.5, off_time=0.5, n=3)
    led2.color = (1, 0, 0)
    #turn led1 red
    led1.color = (0, 1, 0)
    #make the display count down
    threading.Thread(target=change_display).start()
    #when countdown reaches 4 led1 turns blue and blinks until countdown reaches 0
    sleep(5)
    led1.color = (0, 0, 1)
    led1.blink(on_time=0.5, off_time=0.5, n=5)
    #turn led1 red
    led1.color = (1, 0, 0)
    #turn led2 green
    led2.color = (0, 1, 0)
    #wait for 20 seconds before button can be pressed again
    sleep(7)