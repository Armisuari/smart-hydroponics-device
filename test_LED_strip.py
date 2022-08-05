###
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

LED_strip = [8, 23, 24]

for pin in LED_strip:
    GPIO.setup(pin, GPIO.OUT)

for pin in LED_strip:
    GPIO.output(pin, GPIO.HIGH)


