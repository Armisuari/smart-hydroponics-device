import sys
import I2C_LCD_driver
import os
import glob
from multiprocessing import connection

mylcd = I2C_LCD_driver.lcd() #lcd i2c

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

#Set the IIC address
ads1115.set_addr_ADS1115(0x48)
#Sets the gain and input voltage range.
ads1115.set_gain(ADS1115_REG_CONFIG_PGA_6_144V)

#DS18b20 Initialize
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

try:
	base_dir = '/sys/bus/w1/devices/'
	device_folder = glob.glob(base_dir + '28*')[0]
	device_file = device_folder + '/w1_slave'
except Exception:
	pass

def get_temp():
    file = open(device_file, 'r')
    lines = file.readlines()
    file.close()
    trimmed_data = lines[1].find('t=')

    if trimmed_data != -1:
        temp_string = lines[1][trimmed_data+2:]
        temp_c = float(temp_string) / 1000.0
        return temp_c


def read_ph():
	adc0 = ads1115.read_voltage(0)
	PH = ph.readPH(adc0['r'])
	# print("PH: %.1f" %(PH))
	# mylcd.lcd_display_string('PH: ', 1,0)
	# mylcd.lcd_display_string(str('%.1f' % PH), 1,3)
	return PH

def read_ec():
	temperature = 25 # or make your own temperature read process
	#Get the Digital Value of Analog of selected channel
	adc1 = ads1115.read_voltage(1)
	#Convert voltage to EC with temperature compensation
	EC = ec.readEC(adc1['r'],temperature)
	# mylcd.lcd_display_string('EC: ', 1,8)
	# mylcd.lcd_display_string(str('%.1f' % EC), 1,11)
	# mylcd.lcd_display_string('ms/cm', 1,15)
	# mylcd.lcd_display_string('Temp: ', 2,0)
	# mylcd.lcd_display_string(str('%.1f' % temperature), 2,5)
	# print("Temperature:%.1f ^C EC:%.2f ms/cm " %(temperature,EC))
	return EC


# try:
#     while True:
#         f = open('config.json')
#         data = json.load(f)
#         delay_device = data['config']['delay']
#         calibrate_ec = data['config']['calibrate_ec']

#         read_ph_ec()
#         time.sleep(int(delay_device))
# except KeyboardInterrupt:
# 	pass
