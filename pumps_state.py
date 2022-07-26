#pumps state
import RPi.GPIO as GPIO
import json


water_pump = 16
alkaline_pump = 13
acid_pump = 19
nutrient_a = 18
nutrient_b = 12

led_pin = {
            23: False,
            24: False,
            8: False
          }

pumps_state = {
                water_pump: False,
                alkaline_pump: False,
                acid_pump: False,
                nutrient_a: False,
                nutrient_b: False 
              }

pumps_info = {
    'water pump': False,
    'alkaline pump': False,
    'acid pump': False,
    'nutrient A': False,
    'nutrient B': False,
}

led_state = {'led_strip': False}

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for pin in pumps_state:
    GPIO.setup(pin, GPIO.OUT)

for pin in led_pin:
    GPIO.setup(pin, GPIO.OUT)

def set_water(state):
    GPIO.output(water_pump, GPIO.HIGH if state == True else GPIO.LOW)
    print("WATER PUMP IS ON") if state == True else print("WATER PUMP IS OFF")
    pumps_state[water_pump] = state
    pumps_info['water pump'] = state

def set_alkaline(state):
    GPIO.output(alkaline_pump, GPIO.HIGH if state == True else GPIO.LOW)
    print("ALKALINE PUMP IS ON") if state == True else print("ALKALINE PUMP IS OFF")
    pumps_state[alkaline_pump] = state
    pumps_info['alkaline pump'] = state

def set_acid(state):
    GPIO.output(acid_pump, GPIO.HIGH if state == True else GPIO.LOW)
    print("ACID PUMP IS ON") if state == True else print("ACID PUMP IS OFF")
    pumps_state[acid_pump] = state
    pumps_info['acid pump'] = state

def set_nutrient_a(state):
    GPIO.output(nutrient_a, GPIO.HIGH if state == True else GPIO.LOW)
    print("NUTRIENT A PUMP IS ON") if state == True else print("NUTRIENT A PUMP IS OFF")
    pumps_state[nutrient_a] = state
    pumps_info['nutrient A'] = state

def set_nutrient_b(state):
    GPIO.output(nutrient_b, GPIO.HIGH if state == True else GPIO.LOW)
    print("NUTRIENT B PUMP IS ON") if state == True else print("NUTRIENT B PUMP IS OFF")
    pumps_state[nutrient_b] = state
    pumps_info['nutrient B'] = state

def get_pumps():
    return json.dumps(pumps_state)

def get_led():
    return json.dumps(led_state)

def set_led(state):
    if state == True:
        GPIO.output(23, GPIO.HIGH)
        GPIO.output(24, GPIO.HIGH)
        GPIO.output(8, GPIO.HIGH)
    else:
        GPIO.output(23, GPIO.LOW)
        GPIO.output(24, GPIO.LOW)
        GPIO.output(8, GPIO.LOW)

    print("LED IS ON") if state == True else print("LED IS OFF")
    led_state['led_strip'] = state
