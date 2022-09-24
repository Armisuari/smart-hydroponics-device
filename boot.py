import I2C_LCD_driver
import subprocess
import socket
import time

mylcd = I2C_LCD_driver.lcd()
hostname = socket.gethostname()

mylcd.lcd_display_string(' Smart Hydroponics! ', 1, 0)

def get_ip_address():
    ip_address = ''
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip_address = s.getsockname()[0]
    s.close()
    return ip_address

time.sleep(30)

try:
    ssid = subprocess.check_output(['sudo', 'iwgetid']).decode()
    print("Connected Wifi SSID: " + ssid.split('"')[1])
    print(get_ip_address())

    mylcd.lcd_display_string('Wifi: ' + ssid.split('"')[1], 3, 0)
    mylcd.lcd_display_string('IP: ' + get_ip_address(), 4, 0)
except:
    pass
