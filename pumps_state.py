#pumps state
import RPi.GPIO as GPIO
import json


water_pump = 16
alkaline_pump = 13
acid_pump = 19
nutrient_a = 18
nutrient_b = 12

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

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for pin in pumps_state:
    GPIO.setup(pin, GPIO.OUT)

def set_water(state):
    GPIO.output(water_pump, GPIO.HIGH if state == True else GPIO.LOW)
    print("WATER PUMP IS ON") if state == True else print("WATER PUMP IS OFF")
    pumps_state[water_pump] = state
    pumps_info['water pump'] = state
def get_pumps():
    return json.dumps(pumps_state)