#this is the main code file
#snvwevnwehrh
#newvscode

import I2C_LCD_driver
import time
import sys
import RPi.GPIO as GPIO
import datetime
import json
import logging
import os
import glob
import paho.mqtt.client as mqtt

THINGSBOARD_HOST = 'demo.thingsboard.io'
ACCES_TOKEN = 'ZiyVSGT5blbSNIqEvF8h'

sys.path.insert(0,'../libs/DFRobot_ADS1115/RaspberryPi/Python/')
sys.path.insert(0,'../libs/GreenPonik_EC_Python_industrial_probes/src/')
sys.path.insert(0,'../src/')

ADS1115_REG_CONFIG_PGA_6_144V        = 0x00 # 6.144V range = Gain 2/3
ADS1115_REG_CONFIG_PGA_4_096V        = 0x02 # 4.096V range = Gain 1
ADS1115_REG_CONFIG_PGA_2_048V        = 0x04 # 2.048V range = Gain 2 (default)
ADS1115_REG_CONFIG_PGA_1_024V        = 0x06 # 1.024V range = Gain 4
ADS1115_REG_CONFIG_PGA_0_512V        = 0x08 # 0.512V range = Gain 8
ADS1115_REG_CONFIG_PGA_0_256V        = 0x0A # 0.256V range = Gain 16

from DFRobot_ADS1115 import ADS1115
from GreenPonik_EC import GreenPonik_EC
from GreenPonik_PH import GreenPonik_PH

Water_Pump = 16
Alkaline_Pump = 13
Acid_Pump = 19
Nutrient_A = 18
Nutrient_B = 12

Pumps = {Water_Pump: False, Alkaline_Pump: False, Acid_Pump: False, Nutrient_A: False, Nutrient_B: False}

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for y in Pumps:
    GPIO.setup(y, GPIO.OUT)

GPIO.output(Nutrient_A, 0)
GPIO.output(Nutrient_B, 0)

ads1115 = ADS1115()
ec      = GreenPonik_EC()
ph      = GreenPonik_PH()

mylcd = I2C_LCD_driver.lcd()

ec.begin()
ph.begin()

PH = 0.0
EC = 0.0

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

mylcd.lcd_clear()
mylcd.lcd_display_string('Smart Hydroponic IoT', 1, 0)
mylcd.lcd_display_string('By Macca Lab', 2, 4)
mylcd.lcd_display_string('--------------------', 3, 0)
mylcd.lcd_display_string('User: Mr.Yoki Ramdan', 4, 0)
time.sleep(5)
mylcd.lcd_clear()

wp_state = False

desired_EC = 0.0
desired_PH = 0.0

AB_pump = False
Al_pump = False
Ac_pump = False
state_low = False
state_high= False

state_manual_water = False
state_manual = False
state_manual_AB = False
state_manual_al = False
state_manual_ac = False

def simulasi():
    global wp_state, EC, PH, state_manual_water, desired_EC, desired_PH
    global AB_pump, Al_pump, Ac_pump, state_low, state_high, state_manual
    global state_manual_AB, state_manual_al, state_manual_ac

    if wp_state == True:
        EC = EC - 1
        PH = PH - 0.1
        
    if PH<0:
        PH = 0
    if EC<0:
        EC = 0
        
    if state_ec == False and wp_state == True:
        EC = EC + 5
        AB_pump = True
    else:
        AB_pump = False

    if state_ph == False and wp_state == True:
        if state_low == True and state_high == False:
            PH = PH + 0.5
            Al_pump = True
            Ac_pump = False
        elif state_low == False and state_high == True:
            PH = PH - 0.5
            Al_pump = False
            Ac_pump = True
    else:
        Al_pump = False
        Ac_pump = False

    print("state_low =", state_low)
        
    if state_ec == True:
        EC = EC - 0.1
        print('EC in evaporation...............................')
    if state_ph == True:
        PH = PH - 0.01
        print('PH in evaporation...............................')

    j = open('state-manual.txt', 'r')
    manual_but = int(j.read())
    if manual_but == 1:
        state_manual = True
    else:
        state_manual = False

    print('state manual = ', state_manual)

    g = open('manual-water.txt','r')
    manual_but1 = int(g.read())
    if manual_but1 == 1:
        state_manual_water = True
    else:
        state_manual_water = False

    k = open('manual-AB.txt','r')
    manual_but2 = int(k.read())
    if manual_but2 == 1:
        state_manual_AB = True
    else:
        state_manual_AB = False
    
    l = open('manual-al.txt','r')
    manual_but3 = int(l.read())
    if manual_but3 == 1:
        state_manual_al = True
    else:
        state_manual_al = False
    
    m = open('manual-ac.txt','r')
    manual_but4 = int(m.read())
    if manual_but4 == 1:
        state_manual_ac = True
    else:
        state_manual_ac = False
    
    h = open('desired_EC.txt','r')
    desired_EC = float(h.read())
    
    i = open('desired_PH.txt','r')
    desired_PH = float(i.read())
    
    f=open('ec-sim.txt','w')
    e=open('ph-sim.txt','w')
    f.write(str(EC))
    e.write(str(PH))
    f.close()
    e.close()

def water_pump(state):
    global EC, PH, wp_state, state_manual_water, Water_Pump
    print('state water = ', state)
    if state == 'on':
        wp_state = True
        GPIO.output(Water_Pump, 1)
        print('Water Pump On............ wp_state =', wp_state )        
    else:   
        wp_state = False
        GPIO.output(Water_Pump, 0)
        print('Water Pump On............wp_state =', wp_state )

    mylcd.lcd_display_string("WP:%.f " %(wp_state), 1, 8)
    
def Nut_pump():
    global EC
    EC=EC+3.0
    f = open('ec-sim.txt', 'w')
    f.write(str(EC))
    f.close

def ph_pump(status):
    global PH
    
    if status == 'up':
        PH=PH+1
    else:
        PH=PH-1
        
    f = open('ph-sim.txt','w')
    f.write(str(PH))
    f.close
    
state_ec = False
state_ph = False
state_over = False
state_manual_water = False

def millis():
    return time.time() * 1000
    
a = 'off'
b = 0
c = 0
d = 0
previousMillis = 0
previousMillis_ec = 0
previousMillis_al = 0
previousMillis_ac = 0
previous_minute = 0
td = datetime.datetime.today()

def manual():
    global state_manual_water, state_manual_AB, state_manual_al, state_manual_ac

    if state_manual_water == True:
        water_pump('on')
    else:
        water_pump('off')

    if state_manual_AB == True:
        GPIO.output(Nutrient_A, 1)
        GPIO.output(Nutrient_B, 1)
    else:
        GPIO.output(Nutrient_A, 0)
        GPIO.output(Nutrient_B, 0)
        
    if state_manual_al == True:
        GPIO.output(Alkaline_Pump, 1)
    else:
        GPIO.output(Alkaline_Pump, 0)
        
    if state_manual_ac == True:
        GPIO.output(Acid_Pump, 1)
    else:
        GPIO.output(Acid_Pump, 0)
        
waktu = 0
previous_minute2 = td.minute

def auto():
    global PH, EC, state_ec, state_ph, a, previousMillis, wp_state, state_over
    global desired_EC, previousMillis_ec, b, c, d, previousMillis_al, previousMillis_ac
    global state_low, state_high, Al_pump, Ac_pump, previous_minute, td, state_manual_water
    global waktu, previous_minute2

    #desired_PH = 7
    #desired_EC = 250
    range_PH = 0.5
    range_EC = 50
    wp_interval = 1 # 1 minute
    pump_interval = 15000
    
    currentMillis = millis()
    
    if state_ec == False or state_ph == False or state_over == True:
        if td.minute - previous_minute >= wp_interval:
            previous_minute = td.minute
            if a == 'off':
                a = 'on'
            else:
                a = 'off'
            water_pump(a)
    #water_pump('on')
    elif state_ec == True and state_ph == True:
        if td.minute - previous_minute2 >= 1:
            previous_minute2 = td.minute
            if a == 'on':
                print('Desired EC & PH Achieved, turn off water pump after 5 mins')
                a = 'off'
                water_pump(a)
    
    print("previous_minute =", previous_minute)

    if AB_pump == True:
        if currentMillis - previousMillis_ec >= pump_interval:
            previousMillis_ec = currentMillis
            GPIO.output(Nutrient_A, 1)
            GPIO.output(Nutrient_B, 1)
            time.sleep(1)
            GPIO.output(Nutrient_A, 0)
            GPIO.output(Nutrient_B, 0)
    else:
        GPIO.output(Nutrient_A, 0)
        GPIO.output(Nutrient_B, 0)
     
    if Al_pump == True:
        if currentMillis - previousMillis_al >= pump_interval:
            previousMillis_al = currentMillis
            GPIO.output(Alkaline_Pump, 1)
            time.sleep(1)
            GPIO.output(Alkaline_Pump, 0)
    else:
        GPIO.output(Alkaline_Pump, 0)

    if Ac_pump == True:
        if currentMillis - previousMillis_ac >= pump_interval:
            previousMillis_ac = currentMillis
            GPIO.output(Acid_Pump, 1)
            time.sleep(1)
            GPIO.output(Acid_Pump, 0)
    else:
        GPIO.output(Acid_Pump, 0)
    
    error_PH = round(PH - desired_PH, 1)
    error_EC = round((EC - desired_EC),0)
    
    mylcd.lcd_display_string("E:%.1f " %(error_PH), 1, 14)
    mylcd.lcd_display_string("E:%.f " %(error_EC), 2, 14)

    
    if error_PH>=(-range_PH) and error_PH<=range_PH:
        print('PH Stable', error_PH)
        mylcd.lcd_display_string(">>PH:Stable...     ", 3, 0)
        state_ph = True
        state_low = False
        state_high = False
    elif error_PH>range_PH:
        print('Calibrating PH | acid pump active ', error_PH)
        mylcd.lcd_display_string(">>PH:Calibrating...", 3, 0)
        state_ph = False
        state_low = False
        state_high = True
    elif error_PH<(-range_PH):
        print('>>Calibrating PH | alkaline pump active', error_PH)
        mylcd.lcd_display_string(">>PH:Calibrating...", 3, 0)
        state_ph = False
        state_low = True
        state_high = False
  
    if error_EC>=(-range_EC) and error_EC<=range_EC:
        print('EC Stable', error_EC)
        mylcd.lcd_display_string(">>EC:Stable...     ", 4, 0)
        state_ec = True
    elif error_EC>range_EC:
        print('>>Calibrating EC | Water pump active ', error_EC)
        mylcd.lcd_display_string(">>EC:Over...       ", 4, 0)
        state_over = True
        state_ec = True
    elif error_EC<(-range_EC):
        print('>>Calibrating EC | AB Mix pump active ', error_EC)
        mylcd.lcd_display_string(">>EC:Calibrating...", 4, 0)
        state_ec = False

def get_temp():
    file = open(device_file, 'r')
    lines = file.readlines()
    file.close()
    trimmed_data = lines[1].find('t=')
    
    if trimmed_data != -1:
        temp_string = lines[1][trimmed_data+2:]
        temp_c = float(temp_string) / 1000.0
        return temp_c

def read_sensor():
	global ads1115, wp_state
	global ec, EC
	global ph, PH
	
	ph_sim = open('ph-sim.txt', 'r')
	ec_sim = open('ec-sim.txt', 'r')
	temperature = round(get_temp(), 0) #25 # or make your own temperature read process
	#Set the IIC address
	ads1115.setAddr_ADS1115(0x48)
	#Sets the gain and input voltage range.
	ads1115.setGain(ADS1115_REG_CONFIG_PGA_6_144V)
	#Get the Digital Value of Analog of selected channel
	adc0 = ads1115.readVoltage(0)
	adc1 = ads1115.readVoltage(1)
	#Convert voltage to EC with temperature compensation
	#EC = ec.readEC(adc0['r'],temperature)
	#PH = ph.readPH(adc1['r'])
	EC = float(ec_sim.read())
	PH = float(ph_sim.read())
	print("Temperature:%.1f ^C EC:%.2f ms/cm PH:%.2f " %(temperature,EC,PH))
	#mylcd.lcd_clear()
	mylcd.lcd_display_string("Temp:%.f " %(temperature), 1, 0)
	mylcd.lcd_display_string("EC:%.f " %(EC), 2, 0)
	mylcd.lcd_display_string("PH:%.1f " %(PH), 2, 7)
	return temperature, EC, PH

previous_second = 0

if __name__ == "__main__":
    try:
        while True:
            td = datetime.datetime.today()
            previous_second = td.second
            read_sensor()
            simulasi()
            if state_manual == True:
                manual()
            else:
                auto()
            time.sleep(0.2)
            
    except KeyboardInterrupt:
        pass