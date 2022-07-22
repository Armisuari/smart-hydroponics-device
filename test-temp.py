from __future__ import print_function
#import I2C_LCD_driver
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
import I2C_LCD_driver

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

mylcd = I2C_LCD_driver.lcd()

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

try:
    while True:
        temperature =  get_temp()
        print("Temperature:%.2f ^C" %(temperature))
        mylcd.lcd_display_string('Temp: ', 1,0)
        mylcd.lcd_display_string(str(temperature), 1,6)
        mylcd.lcd_display_string('^C', 1,10)

except KeyboardInterrupt:
    pass