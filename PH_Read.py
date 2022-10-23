import time
import sys
sys.path.insert(0,'../libs/DFRobot_ADS1115/RaspberryPi/Python/')
sys.path.insert(0,'../src/')


from DFRobot_ADS1115 import ADS1115
from GreenPonik_PH import GreenPonik_PH

ADS1115_REG_CONFIG_PGA_6_144V = 0x00  # 6.144V range = Gain 2/3
ADS1115_REG_CONFIG_PGA_4_096V = 0x02  # 4.096V range = Gain 1
ADS1115_REG_CONFIG_PGA_2_048V = 0x04  # 2.048V range = Gain 2 (default)
ADS1115_REG_CONFIG_PGA_1_024V = 0x06  # 1.024V range = Gain 4
ADS1115_REG_CONFIG_PGA_0_512V = 0x08  # 0.512V range = Gain 8
ADS1115_REG_CONFIG_PGA_0_256V = 0x0A  # 0.256V range = Gain 16

ads1115 = ADS1115()
ph = GreenPonik_PH()
ph.begin()

PH_array = [0.0, 0.0, 0.0, 0.0, 0.0]
V_array = [0.0, 0.0, 0.0, 0.0, 0.0]

def read_ph():
    global ads1115
    global ph
    # Set the IIC address
    #ads1115.setAddr_ADS1115(0x48)
    ads1115.set_addr_ADS1115(0x48)
    # Sets the gain and input voltage range.
    ads1115.set_gain(ADS1115_REG_CONFIG_PGA_6_144V)
    # Get the Digital Value of Analog of selected channel
    #adc0 = ads1115.readVoltage(0)
    # Convert voltage to pH
    # adc0 = ads1115.read_voltage(0)
    # PH = ph.readPH(adc0['r'])
    for i in range(0,5):
        adc0 = ads1115.read_voltage(0)
        V_array[i] = adc0['r']
        # PH_array[i] = ph.readPH(adc0['r'])
        # print(PH_array[i])
        time.sleep(0.3)

    print(V_array)

    voltage = (V_array[0] + V_array[1] + V_array[2] + V_array[3] + V_array[4]) / 5
    # PH = (PH_array[0] + PH_array[1] + PH_array[2] + PH_array[3] + PH_array[4]) / 5

    PH = -0.004944*voltage + 20.29
    # PH = ph.readPH(voltage)

    print("PH:%.1f Voltage:%2.f " % (PH, voltage))
    # return PH, adc0['r']


if __name__ == "__main__":
    while True:
        read_ph()
        time.sleep(1)