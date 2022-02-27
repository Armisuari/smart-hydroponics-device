#!/usr/bin/python
#editing

from __future__ import print_function
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
import os
from wifi import Cell, Scheme
import subprocess
from DFRobot_ADS1115 import ADS1115
from GreenPonik_EC import GreenPonik_EC
from GreenPonik_PH import GreenPonik_PH
import urllib.request


SSID = ''
PSK = ''

#allSSID = []


#wifiscan()
#CreateWifiConfig(ssid, psk)
#print("rebooting system...")
#time.sleep(2)
#os.system("sudo reboot")

THINGSBOARD_HOST = 'demo.thingsboard.io'
ACCES_TOKEN = 'YnWgABN7LrZ1nsAJvxYT'

url = 'https://demo.thingsboard.io'

mylcd = I2C_LCD_driver.lcd()

mylcd.lcd_clear()
mylcd.lcd_display_string('Smart Hydroponic IoT', 1, 0)
mylcd.lcd_display_string('By Macca Lab', 2, 4)
mylcd.lcd_display_string('--------------------', 3, 0)
mylcd.lcd_display_string('User: Mr.Yoki Ramdan', 4, 0)
time.sleep(2)
mylcd.lcd_clear()
mylcd.lcd_display_string('Preparing all system', 1, 0)
time.sleep(2)
mylcd.lcd_display_string('>>>>Please Wait!<<<<', 2, 0)
time.sleep(20)
mylcd.lcd_display_string('     Almost done    ', 3, 0)
mylcd.lcd_display_string('~~~~~~~~~~~~~~~~~~~~', 4, 0)
time.sleep(20)
mylcd.lcd_clear()

def millis():
    return time.time() * 1000

try:
    allSSID = list(Cell.all('wlan0'))
except:
    allSSID = []

def wifiscan():
    global SSID, allSSID

    print(allSSID)
    myssid = 'Cell(ssid='+SSID+')'
    print(myssid)

    for i in range(len(allSSID)):
        if str(allSSID[i]) == myssid:
            a = i
            myssid = allSSID[a]
            print(a)
            break

        else:
            print('.')

def CreateWifiConfig(ssid, psk):
    config_lines = [
        '\n',
        'network={',
        '\tssid="{}"'.format(ssid),
        '\tpsk="{}"'.format(psk),
        '\tkey_mgmt=WPA-PSK',
        '}'
    ]
    
    config ='\n'.join(config_lines)
    print(config)

    with open("/etc/wpa_supplicant/wpa_supplicant.conf", "a+") as wifi:
        wifi.write(config)
    print("Wifi config added")

try:
    print('Checking Internet Connection...')
    mylcd.lcd_display_string('Checking Internet', 1, 1)
    mylcd.lcd_display_string('Connection...', 2, 3)
    urllib.request.urlopen(url, timeout=1)
    print("Connected to the Internet")
    mylcd.lcd_display_string('~CONNECTED~', 4, 4)
    time.sleep(3)
    mylcd.lcd_clear()
except urllib.request.URLError as err:
    print('No internet connection')
    mylcd.lcd_display_string('!FAIL!', 4, 7)
    time.sleep(3)
    mylcd.lcd_clear()
    mylcd.lcd_display_string('Please set up your', 1, 1)
    mylcd.lcd_display_string('mobile hotspot :', 2, 2)
    mylcd.lcd_display_string('(SSID: admin)', 3, 3)
    mylcd.lcd_display_string('(Password: admin123)', 4, 0)
    time.sleep(30)
    mylcd.lcd_clear()

sys.path.insert(0,'../libs/DFRobot_ADS1115/RaspberryPi/Python/')
sys.path.insert(0,'../libs/GreenPonik_EC_Python_industrial_probes/src/')
sys.path.insert(0,'../src/')

ADS1115_REG_CONFIG_PGA_6_144V        = 0x00 # 6.144V range = Gain 2/3
ADS1115_REG_CONFIG_PGA_4_096V        = 0x02 # 4.096V range = Gain 1
ADS1115_REG_CONFIG_PGA_2_048V        = 0x04 # 2.048V range = Gain 2 (default)
ADS1115_REG_CONFIG_PGA_1_024V        = 0x06 # 1.024V range = Gain 4
ADS1115_REG_CONFIG_PGA_0_512V        = 0x08 # 0.512V range = Gain 8
ADS1115_REG_CONFIG_PGA_0_256V        = 0x0A # 0.256V range = Gain 16

Water_Pump = 16
Alkaline_Pump = 13
Acid_Pump = 19
Nutrient_A = 18
Nutrient_B = 12
Water_Level = 17

Pumps = {Water_Pump: False, Alkaline_Pump: False, Acid_Pump: False, Nutrient_A: False, Nutrient_B: False}

sensor_data = {'pH Value': 0, 'Temp Value': 0, 'Tds Value': 0, 'Ec Value': 0, 'Water State': False}

j = open('state-manual.txt', 'r')
Manual_Switch = int(j.read())

manual_button ={'Manual Switch': Manual_Switch, 'Manual Water': False, 'Manual Alkaline': False, 'Manual Acid': False, 'Manual Nutrient': False}


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for x in Pumps:
    GPIO.setup(x, GPIO.OUT)

GPIO.setup(Water_Level, GPIO.IN)

GPIO.output(Nutrient_A, 0)
GPIO.output(Nutrient_B, 0)

ads1115 = ADS1115()
ec      = GreenPonik_EC()
ph      = GreenPonik_PH()

ec.begin()
ph.begin()

PH = 0.0
EC = 0.0

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

try:
    base_dir = '/sys/bus/w1/devices/'
    device_folder = glob.glob(base_dir + '28*')[0]
    device_file = device_folder + '/w1_slave'
except:
    pass

wp_state = False

w = open('desired_EC.txt', 'r')
z = open('desired_PH.txt', 'r')

desired_EC = float(w.read())
desired_PH = float(z.read())

SSID_PSK = ''

list_wifi = str(allSSID)
format_wifi = '[]=()'
new_list_wifi = list_wifi
for character in format_wifi:
    new_list_wifi = new_list_wifi.replace(character, '')
new_list_wifi_1 = new_list_wifi.replace('Cell', '')
new_list_wifi_2 = new_list_wifi_1.replace('ssid', '')
new_list_wifi_3 = new_list_wifi_1.replace(',', ' |')

wifi = {'wifi': new_list_wifi_2, 'pass': ' '}

desired_value = {'desired_EC': desired_EC, 'desired_PH': desired_PH}

AB_pump = False
Al_pump = False
Ac_pump = False
state_low = False
state_high= False

state_manual_water = False
state_manual = Manual_Switch
state_manual_AB = False
state_manual_al = False
state_manual_ac = False

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, rc, *extra_params):
    global manual_button, Manual_Switch, allSSID, wifi

    print('Connected with result code ' + str(rc))
    # Subscribing to receive RPC requests
    client.subscribe('v1/devices/me/rpc/request/+')
    #client.subscribe('v1/devices/me/rpc/response/+')
    client.subscribe('v1/devices/me/attributes')
    # Sending current GPIO status
    client.publish('v1/devices/me/attributes', pumps_status(), 1)
    client.publish('v1/devices/me/attributes', json.dumps(manual_button), 1)
    #client.publish('v1/devices/me/attributes', json.dumps(wifi_input), 1)
    client.publish('v1/devices/me/attributes', json.dumps(desired_value), 1)
    client.publish('v1/devices/me/attributes', json.dumps(wifi), 1)
    print('pushhhhhhhhh')
    time.sleep(5)


reboot_state = False
reboot_state_now = False
shutdown_state = False

def on_message(client, userdata, msg):
    global state_manual, manual_button, state_manual_water, state_manual_AB
    global state_manual_al, state_manual_ac, desired_EC, desired_PH
    global SSID_PSK, reboot_state, reboot_state_now, shutdown_state

    print ('Topic: ' + msg.topic + '\nMessage: ' + str(msg.payload))
    # Decode JSON request
    data = json.loads(msg.payload)
    #print('isi data:', data)
    list_data_wifi = []

    if msg.topic == 'v1/devices/me/attributes':
        data_after = list(data)
        if data_after[0] == 'desired_EC':
            desired_EC = data['desired_EC']
        elif data_after[0] == 'desired_PH':
            desired_PH = data['desired_PH']
        
        elif data_after[0] == 'pass':
            SSID_PSK=data['pass']
            list_data_wifi = SSID_PSK.split(', ')
            SSID = list_data_wifi[0]
            PSK = list_data_wifi[1]
            print(SSID, PSK)
            wifiscan()
            CreateWifiConfig(SSID, PSK)
            print("rebooting system...")
            reboot_state = True

            #SSID = data['SSID']
        """    
        elif data_after[0] == 'PSK':
            PSK = data['PSK']
            ip_address = wifi_connect(SSID, PSK)
        """

        j = open('desired_PH.txt', 'w')
        i = open('desired_EC.txt','w')
        j.write(str(desired_PH))
        i.write(str(desired_EC))
        j.close()
        i.close()

    else:
        if data['method'] == 'setValue_ms':
            if data['params'] == True:
                Manual_Switch = 1
                state_manual = True
            else:
                state_manual = False
                Manual_Switch = 0
            f = open('state-manual.txt', 'w')
            f.write(str(Manual_Switch))
            f.close

        if data['method'] == 'setValue_water':
            if data['params'] == True:
                state_manual_water = True
            else:
                state_manual_water = False
        if data['method'] == 'setValue_ab':
            if data['params'] == True:
                state_manual_AB = True
            else:
                state_manual_AB = False
        if data['method'] == 'setValue_up':
            if data['params'] == True:
                state_manual_al = True
            else:
                state_manual_al = False
        if data['method'] == 'setValue_down':
            if data['params'] == True:
                state_manual_ac = True
            else:
                state_manual_ac = False

        if data['method'] == 'rpcCommand':
            reboot_state_now = True
        if data['method'] == 'rpc_shut':
            shutdown_state = True

def pumps_status():
    # Encode GPIOs state to json
    return json.dumps(Pumps)
    
def set_gpio_status(pin, status):
    GPIO.output(pin, GPIO.HIGH if status else GPIO.LOW)
    Pumps[pin] = status

client = mqtt.Client()
#Register pconnect callback
client.on_connect = on_connect
#Registed publish message callback
client.on_message = on_message
#set acces token
client.username_pw_set(ACCES_TOKEN)
#Connect to Thingsboard using default MQTT port
try:
    client.connect(THINGSBOARD_HOST, 1883, 60)
    """
    mylcd.lcd_clear()
    mylcd.lcd_display_string('>>Connected to the<<', 1, 0)
    mylcd.lcd_display_string('>>>>>>>SERVER<<<<<<<', 2, 0)
    time.sleep(2)
    mylcd.lcd_clear()
    """
except:
    pass

client.loop_start()

def simulasi():
    global wp_state, EC, PH, state_manual_water, desired_EC, desired_PH
    global AB_pump, Al_pump, Ac_pump, state_low, state_high, state_manual
    global state_manual_AB, state_manual_al, state_manual_ac, TDS

    #if wp_state == True:
        #EC = EC - 1
    #    PH = PH - 0.1
        
    #if PH<0:
    #    PH = 0
    #if EC<0:
    #    EC = 0
        
    if state_ec == False and wp_state == True:
        #EC = EC + 5
        AB_pump = True
    else:
        AB_pump = False

    if state_ph == False and wp_state == True:
        if state_low == True and state_high == False:
            #PH = PH + 0.5
            Al_pump = True
            Ac_pump = False
        elif state_low == False and state_high == True:
            #PH = PH - 0.5
            Al_pump = False
            Ac_pump = True
    else:
        Al_pump = False
        Ac_pump = False

    print('AB_pump: ', AB_pump)
        
    #if state_ec == True:
    #    EC = EC - 0.1
        #print('EC in evaporation...............................')
    #if state_ph == True:
    #    PH = PH - 0.01
        #print('PH in evaporation...............................')

    print('state manual = ', state_manual)
    
    """
    h = open('desired_EC.txt','r')
    desired_EC = float(h.read())
    
    i = open('desired_PH.txt','r')
    desired_PH = float(i.read())
    """
    
    #f=open('ec-sim.txt','w')
    #e=open('ph-sim.txt','w')
    #f.write(str(EC))
    #e.write(str(PH))
    #f.close()
    #e.close()

def water_pump(state):
    global EC, PH, wp_state, state_manual_water, Water_Pump
    #print('state water = ', state)
    if state == 'on':
        wp_state = True
        GPIO.output(Water_Pump, 1)
        print('Water Pump On............ wp_state =', wp_state )        
    else:   
        wp_state = False
        GPIO.output(Water_Pump, 0)
        print('Water Pump Off............wp_state =', wp_state )

    mylcd.lcd_display_string("Water Pump:%.f " %(wp_state), 1, 8)
    
state_ec = False
state_ph = False
state_over = False
state_manual_water = False
    
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
    
    mylcd.lcd_display_string('--------------------', 3, 0)
    mylcd.lcd_display_string('>>>>Manual Mode<<<<<', 4, 0)

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
    global waktu, previous_minute2, currentMillis, TDS2

    #desired_PH = 7
    #desired_EC = 250
    range_PH = 0.5
    range_EC = 100
    wp_interval = 1 # 1 minute
    pump_interval = 15000
    
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
    
    #print("previous_minute =", previous_minute)

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
    error_EC = round((TDS2 - desired_EC),0)
    
    #mylcd.lcd_display_string("E:%.1f " %(error_PH), 1, 14)
    #mylcd.lcd_display_string("E:%.f " %(error_EC), 2, 14)

    
    if error_PH>=(-range_PH) and error_PH<=range_PH:
        print('PH Stable', error_PH)
        mylcd.lcd_display_string(">>PH:Stable...      ", 3, 0)
        state_ph = True
        state_low = False
        state_high = False
    elif error_PH>range_PH:
        print('Calibrating PH | acid pump active ', error_PH)
        mylcd.lcd_display_string(">>PH:Calibrating... ", 3, 0)
        state_ph = False
        state_low = False
        state_high = True
    elif error_PH<(-range_PH):
        print('>>Calibrating PH | alkaline pump active', error_PH)
        mylcd.lcd_display_string(">>PH:Calibrating... ", 3, 0)
        state_ph = False
        state_low = True
        state_high = False
  
    if error_EC>=(-range_EC) and error_EC<=range_EC:
        print('EC Stable', error_EC)
        mylcd.lcd_display_string(">>EC:Stable...      ", 4, 0)
        state_ec = True
    elif error_EC>range_EC:
        print('>>Calibrating EC | Water pump active ', error_EC)
        mylcd.lcd_display_string(">>EC:Over...        ", 4, 0)
        state_over = True
        state_ec = True
    elif error_EC<(-range_EC):
        print('>>Calibrating EC | AB Mix pump active ', error_EC)
        mylcd.lcd_display_string(">>EC:Calibrating... ", 4, 0)
        state_ec = False

def get_temp():
    try:
        file = open(device_file, 'r')
        lines = file.readlines()
        file.close()
        trimmed_data = lines[1].find('t=')
    
        if trimmed_data != -1:
            temp_string = lines[1][trimmed_data+2:]
            temp_c = float(temp_string) / 1000.0
            return temp_c
    except:
        pass

PH = 0.0
temperature = 0.0
EC = 0.0
PH2 = 0.0
TDS2 = 0.0

def read_sensor():
    global ads1115, wp_state, temperature, PH2
    global ec, EC, ph, PH, sensor_data, wifi_input, TDS2
    
    ph_sim = open('ph-sim.txt', 'r')
    ec_sim = open('ec-sim.txt', 'r')
    ws_sim = open('ws-sim.txt', 'r')
    temperature = get_temp()
    #Set the IIC address
    ads1115.setAddr_ADS1115(0x48)
    #Sets the gain and input voltage range.
    ads1115.setGain(ADS1115_REG_CONFIG_PGA_6_144V)
    #Get the Digital Value of Analog of selected channel
    adc0 = ads1115.readVoltage(0)
    adc1 = ads1115.readVoltage(1)
    #Convert voltage to EC with temperature compensation
    EC = ec.readEC(adc1['r'],temperature)
    PH = ph.readPH(adc0['r'])
    TDS = EC * 500 # 1.0 ms/cm = 500 ppm
    TDS2 = 0.1723*TDS+3.679
    """
    PH3 = 2.430*PH-10.30
	PH2 = 0.6438*PH3-0.9918
	"""

    if WS == 1:
        sensor_data['Water State'] = True
    else:
        sensor_data['Water State'] = False

    client.publish('v1/devices/me/telemetry', json.dumps(sensor_data), 1)

    if EC <= 0.5:
        EC = 0

    if PH > 10:
        PH = 0

    #EC = float(ec_sim.read())
    #PH = float(ph_sim.read())
    WS = int(ws_sim.read())
    #print("Water Level:", GPIO.input(Water_Level))
    #print("Temperature:%.1f ^C EC:%.2f ms/cm PH:%.2f TDS:%.f " %(temperature,EC, PH, TDS))
    """
    mylcd.lcd_display_string("Temp:%.f " %(temperature), 1, 0)
    mylcd.lcd_display_string('      ', 2, 4)
    mylcd.lcd_display_string('       ', 2, 13)
    mylcd.lcd_display_string("TDS:%.f " %(TDS), 2, 0)
    mylcd.lcd_display_string("PH:%.1f " %(PH), 2, 10)
    """
    return temperature, EC, PH, TDS2

def rata_rata():
    global PH, EC, TDS2, temperature, PH2
    data_PH =[]
    data_TDS=[]
    jum = 0
    jum1 = 0

    for i in range(0, 5):
        PH1 = PH
        data_PH.append(PH1)
        jum += data_PH[i]
        rata2 = jum / 5

    for j in range(0, 5):
        TDS1 = TDS2
        data_TDS.append(TDS1)
        jum += data_TDS[j]
        rata21 = jum1 / 5

    sensor_data['Temp Value'] = temperature
    sensor_data['Ec Value'] = TDS2
    sensor_data['pH Value'] = PH

    client.publish('v1/devices/me/telemetry', json.dumps(sensor_data), 1)

    print("Temperature:%.1f ^C EC:%.2f ms/cm PH:%.2f TDS:%.f " %(temperature,EC, PH1, TDS1))
    mylcd.lcd_display_string("Temp:%.f " %(temperature), 1, 0)
    mylcd.lcd_display_string('      ', 2, 4)
    mylcd.lcd_display_string('       ', 2, 13)
    mylcd.lcd_display_string("TDS:%.f " %(TDS1), 2, 0)
    mylcd.lcd_display_string("PH:%.2f " %(PH1), 2, 10)


previous_second = 0
previous_second1 = 0

def reboot():
    mylcd.lcd_clear()
    mylcd.lcd_display_string('>Adding WiFi Succes<', 1, 0)
    mylcd.lcd_display_string('--------------------', 2, 0)
    time.sleep(2)
    mylcd.lcd_display_string('>>Rebooting system<<', 4, 0)
    os.system("sudo reboot")

def reboot_now():
    mylcd.lcd_clear()
    mylcd.lcd_display_string('>>Rebooting system<<', 1, 0)
    mylcd.lcd_display_string('>>>>Please Wait!<<<<', 2, 0)
    time.sleep(2)
    mylcd.lcd_display_string('--------------------', 3, 0)
    os.system("sudo reboot")

def shutdown():
    mylcd.lcd_clear()
    mylcd.lcd_display_string('>>Shutdown system<<', 1, 0)
    mylcd.lcd_display_string('>>>>Please Wait!<<<<', 2, 0)
    time.sleep(2)
    mylcd.lcd_display_string('--------------------', 3, 0)
    os.system("sudo shutdown -now")

if __name__ == "__main__":
    try:
        while True:
            currentMillis = millis()
            td = datetime.datetime.today()
            if reboot_state == True:
                reboot()
            if reboot_state_now == True:
                reboot_now()
            if shutdown_state == True:
                shutdown()

            if currentMillis - previous_second1 >= 1000:
                previous_second1 = currentMillis 
                try:
                    read_sensor()
                except:
                    pass
                

            if currentMillis - previous_second >= 5000:
                previous_second = currentMillis 
                print('>>>>>>>>>>>>>Desried EC:%.1f, desired_PH:%.1f ' %(desired_EC, desired_PH))
                rata_rata()
                simulasi()                

            if state_manual == True:
                manual()
            else:
                auto()
            time.sleep(1)
            
    except KeyboardInterrupt:
        pass
        
client.loop_stop()
client.disconnect()
GPIO.cleanup()