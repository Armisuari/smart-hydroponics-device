import time
import sys
import I2C_LCD_driver

mylcd = I2C_LCD_driver.lcd()

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

#mylcd.lcd_display_string(' Smart Hydroponics! ', 1,0)
V_array = [0.0, 0.0, 0.0, 0.0, 0.0]

def read_ph_ec():
	global ads1115
	global ec
	global ph
	temperature = 25 # or make your own temperature read process
	#Set the IIC address
	ads1115.set_addr_ADS1115(0x48)
	#Sets the gain and input voltage range.
	ads1115.set_gain(ADS1115_REG_CONFIG_PGA_6_144V)
	#Get the Digital Value of Analog of selected channel
	for i in range(0,5):
		adc0 = ads1115.read_voltage(0)
		V_array[i] = adc0['r']
		time.sleep(0.3)

	adc1 = ads1115.read_voltage(1)
	#Convert voltage to EC with temperature compensation
	voltage = (V_array[0] + V_array[1] + V_array[2] + V_array[3] + V_array[4]) / 5

	EC = ec.readEC(adc1['r'],temperature) * 1000
	# PH = ph.readPH(adc0['r'])
	PH = -0.004944*voltage + 20.29
    
	mylcd.lcd_display_string('PH:' + str(round(PH,1)) + ' EC:' + str(int(EC)) + 'us/cm', 1,0)
    
	print("Temperature:%.1f ^C EC:%.f ms/cm PH:%.1f " %(temperature,EC,PH))


if __name__ == "__main__":
    while True:
        read_ph_ec()
        time.sleep(1)
