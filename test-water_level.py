###
import RPi.GPIO as GPIO
import time

water_level = 17 #pin GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(water_level, GPIO.IN)

def read_water_level():
	water = GPIO.input(water_level)
    
	return water

while True:
    print("WATER LEVEL %d" %(read_water_level()))
    time.sleep(1)