import time
import read_sensor
import I2C_LCD_driver

mylcd = I2C_LCD_driver.lcd()

PH_list = [0.0,0.0,0.0]

# time.sleep(60)

def main():
    while True:
        for i in range(0,3):
            PH_list[i] = round(read_sensor.read_ph()+1.5,2)
            time.sleep(0.3)

        _PH = (PH_list[0] + PH_list[1] + PH_list[2]) / 3
    
        EC = read_sensor.read_ec() 

        if EC < 30 : 
            PH = _PH
            EC = 0
            print('no EC')
        else:
            PH = _PH
            print('with EC')


        try:
            temp = read_sensor.get_temp()
        except:
            temp = 25.0

        print(PH_list)
        print('PH: %.2f EC: %.1f uS/cm Temp: %.2f^C ' 
                %(PH, EC, temp))

        mylcd.lcd_clear()
        mylcd.lcd_display_string('PH: '+str(round(PH,2)), 1, 0)
        mylcd.lcd_display_string('EC: '+str(round(EC, 1)) + ' us/cm  ', 2, 0)
        mylcd.lcd_display_string('Temp: '+str(round(temp,2)) + '^C', 3, 0)

        time.sleep(1)

main()
