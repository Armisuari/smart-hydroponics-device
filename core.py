import time
import sys
import I2C_LCD_driver
import json
import os
import glob

mylcd = I2C_LCD_driver.lcd() #lcd i2c

# Opening JSON file
#f = open(path_url+'config.json')  #for asus
f = open('config.json') #for dev
data = json.load(f)
delay_device = (data['config']['delay']) 
connection_mode = data['config']['connection_mode']
param_date_device = data['config']['param_date_device']
param_name_1 = data['config']['param_name_1']
param_name_2 = data['config']['param_name_2']
param_name_3 = data['config']['param_name_3']
param_name_4 = data['config']['param_name_4']
param_status_1 = data['config']['param_status_1']
param_status_2 = data['config']['param_status_2']
param_status_3 = data['config']['param_status_3']
param_status_4 = data['config']['param_status_4']
sensor_type = data['config']['sensor_type']
calibrate_ec = data['config']['calibrate_ec']
print("delay device="+delay_device)
print("connection mode="+connection_mode)
print("param name 1="+param_name_1)
print("param name 2="+param_name_1)
print("param name 3="+param_name_1)
print("param name 4="+param_name_1)
print("Sensor Type="+sensor_type+"\n")
f.close()

#ADS1115 initialize
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
ads1115 = ADS1115()
ec      = GreenPonik_EC()
ph      = GreenPonik_PH()
ec.begin()
ph.begin()

#DS18b20 Initialize
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

def get_temp():
    file = open(device_file, 'r')
    lines = file.readlines()
    file.close()
    trimmed_data = lines[1].find('t=')
    
    if trimmed_data != -1:
        temp_string = lines[1][trimmed_data+2:]
        temp_c = float(temp_string) / 1000.0
        return temp_c

def read_ph_ec():
	global ads1115, calibrate_ec
	global ec
	global ph
	temperature = get_temp() #25 # or make your own temperature read process
	#Set the IIC address
	ads1115.set_addr_ADS1115(0x48)
	#Sets the gain and input voltage range.
	ads1115.set_gain(ADS1115_REG_CONFIG_PGA_6_144V)
	#Get the Digital Value of Analog of selected channel
	adc0 = ads1115.read_voltage(0)
	adc1 = ads1115.read_voltage(1)
	#Convert voltage to EC with temperature compensation
	EC = ec.readEC(adc1['r'],temperature) - float(calibrate_ec)
	PH = ph.readPH(adc0['r'])
    
	mylcd.lcd_display_string('PH: ', 1,0)
	mylcd.lcd_display_string(str('%.1f' % PH), 1,3)
	mylcd.lcd_display_string('EC: ', 1,8)
	mylcd.lcd_display_string(str('%.1f' % EC), 1,11)
	mylcd.lcd_display_string('ms/cm', 1,14)
	mylcd.lcd_display_string('Temp: ', 2,0)
	mylcd.lcd_display_string(str('%.1f' % temperature), 2,5)
    
	print("Temperature:%.1f ^C EC:%.2f ms/cm PH:%.2f " %(temperature,EC,PH))
	return temperature, EC, PH


try:
    while True:
        f = open('config.json')
        data = json.load(f)
        delay_device = data['config']['delay']
        calibrate_ec = data['config']['calibrate_ec']

        read_ph_ec()
        time.sleep(int(delay_device))
except KeyboardInterrupt:
	pass
