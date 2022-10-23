#!/usr/bin/env python3
from asyncore import read
from distutils.log import error
from enum import Flag
import json
from telnetlib import EC
from tokenize import Triple
import paho.mqtt.client as mqtt
from threading import Thread
import _thread
import read_sensor
import time
import datetime
import os
import I2C_LCD_driver
import pumps_state
import logging
import sys

logger = logging.getLogger('')
logger.setLevel(logging.DEBUG)
fh = logging.FileHandler('/home/comitup/Smart-Hydroponics/log_info.log')
sh = logging.StreamHandler(sys.stdout)
formatter = logging.Formatter(
    '[%(asctime)s] %(levelname)s [%(filename)s.%(funcName)s:%(lineno)d] %(message)s', datefmt='%a, %d %b %Y %H:%M:%S')
fh.setFormatter(formatter)
sh.setFormatter(formatter)
logger.addHandler(fh)
logger.addHandler(sh)

mylcd = I2C_LCD_driver.lcd()  # lcd i2c

# Opening JSON file
# f = open(path_url+'config.json')  #for asus
f = open('/home/comitup/Smart-Hydroponics/config.json')  # for dev
data = json.load(f)
delay_device = (data['config']['delay_device'])
delay_update = data['config']['delay_update']
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
calibrate_ph = data['config']['calibrate_ph']
logging.debug("delay device="+delay_device)
logging.debug("connection mode="+connection_mode)
logging.debug("param name 1="+param_name_1)
logging.debug("param name 2="+param_name_1)
logging.debug("param name 3="+param_name_1)
logging.debug("param name 4="+param_name_1)
logging.debug("Sensor Type="+sensor_type+"\n")
f.close()

THINGSBOARD_HOST = 'demo.thingsboard.io'
ACCESS_TOKEN = 'E7yjSrlXEX8VkFJtVz4Q'

# Timeout to wait for connection
WAIT_CONNECTION_TIMEOUT = 10

# We assume that all GPIOs are LOW
# gpio_state = {7: False, 11: False, 12: False, 13: False, 15: False, 16: False, 18: False, 22: False, 29: False,
#   31: False, 32: False, 33: False, 35: False, 36: False, 37: False, 38: False, 40: False}

connected = False
TDS = 0

desired_val = {'pH': 7.0, 'EC': 1200.0}

log_attribute = {'PH': 0.0, 'EC': 0.0, 'TEMP': 0.0, 'prev_ec': 0, 'water': False,
                 'date_time': '', 'water_info': ''}

error_state = {'water state': False, 'ec error': 0.0, 'ph error': 0.0}


def get_desired_value():
    return json.dumps(desired_val)


def millis():
    return time.time() * 1000

# The callback for when the client receives a CONNACK response from the server.


def on_connect(client, userdata, rc, *extra_params):
    logging.debug('Connected with result code ' + str(rc))
    # Subscribing to receive RPC requests
    client.subscribe('v1/devices/me/rpc/request/+')
    client.subscribe('v1/devices/me/attributes/response/+')
    client.subscribe('v1/devices/me/attributes')
    # Sending current GPIO status
    #client.publish('v1/devices/me/attributes', get_gpio_status(), 1)
    client.publish('v1/devices/me/attributes/request/1',
                   '{"sharedKeys":"start_led,end_led"}')
    client.publish('v1/devices/me/attributes/request/1',
                   '{"sharedKeys":"pH,EC"}')
    client.publish('v1/devices/me/attributes', pumps_state.get_pumps(), 1)
    # client.publish('v1/devices/me/attributes', get_desired_value(), 1)
    # client.publish('v1/devices/me/attributes', pumps_state.get_led(), 1)

    global connected
    connected = True


def on_disconnect(unused_client, unused_userdata, rc):
    """Paho callback for when a device disconnects."""
    logging.debug(f"Disconnected with result code " + str(rc))
    logging.debug()

    global connected
    connected = False


def on_publish(client, userdata, mid):
    """Paho callback when a message is sent to the broker."""
    logging.debug('on_publish')
    logging.debug("userdata:" + str(userdata))
    logging.debug("mid:" + str(mid))
    logging.debug()

# The callback for when a PUBLISH message is received from the server.


def on_message(client, userdata, msg):
    logging.debug('\nTopic: ' + msg.topic + '\nMessage: ' + str(msg.payload))
    # Decode JSON request
    data_in = json.loads(msg.payload)
    # Check request method

    if msg.topic == 'v1/devices/me/attributes/response/1':
        logging.debug(data_in)
        try:
            if list(data_in['shared'])[0] == 'pH' or list(data_in['shared'])[0] == 'EC':
                desired_val['pH'] = data_in['shared']['pH']
                desired_val['EC'] = data_in['shared']['EC']
                logging.debug('store desired pH & EC from platform...')

            else:
                pumps_state.led_state['start_led'] = data_in['shared']['start_led']
                pumps_state.led_state['end_led'] = data_in['shared']['end_led']
        except:
            pass

    elif msg.topic == 'v1/devices/me/attributes':
        logging.debug("data atribut changed !")
        logging.debug(data_in)

        if list(data_in)[0] == 'EC':
            desired_val['EC'] = data_in['EC']
        elif list(data_in)[0] == 'pH':
            desired_val['pH'] = data_in['pH']
        else:
            pumps_state.led_state['start_led'] = data_in['start_led']
            pumps_state.led_state['end_led'] = data_in['end_led']

        logging.debug(desired_val)

    else:

        if data_in['method'] == 'set_water':
            pumps_state.set_water(data_in['params'])
        elif data_in['method'] == 'set_alkaline':
            pumps_state.set_alkaline(data_in['params'])
        elif data_in['method'] == 'set_acid':
            pumps_state.set_acid(data_in['params'])
        elif data_in['method'] == 'set_nutrient_a':
            pumps_state.set_nutrient_a(data_in['params'])
        elif data_in['method'] == 'set_nutrient_b':
            pumps_state.set_nutrient_b(data_in['params'])
        elif data_in['method'] == 'set_led':
            pumps_state.set_led(data_in['params'])
            # pumps_state.led_state['led_strip'] = data_in['params']
        elif data_in['method'] == 'set_ec_tds':
            read_sensor.set_ec_tds(data_in['params'])
        else:
            client.publish(msg.topic.replace(
                'request', 'response'), pumps_state.get_pumps(), 1)


client = mqtt.Client()
# Register connect callback
client.on_connect = on_connect
# Registed publish message callback
client.on_message = on_message
# Set access token
client.username_pw_set(ACCESS_TOKEN)
# Connect to ThingsBoard using default MQTT port and 60 seconds keepalive interval
client.connect(THINGSBOARD_HOST, 1883, 60)
client.loop_start()


def publish_events():
    global TDS

    """Publish an event."""
    logging.debug("Publish Events")

    read_sensor.sensor_data['last_update'] = log_attribute['date_time']
    read_sensor.sensor_data['PH_sensor'] = log_attribute['PH']

    if read_sensor.sensor_data["ec_tds"] == False:
        read_sensor.sensor_data['EC_sensor'] = log_attribute['EC']
    else:
        read_sensor.sensor_data['EC_sensor'] = TDS

    read_sensor.sensor_data['TEMP_sensor'] = log_attribute['TEMP']

    log_attribute['water_info'] = "OK" if log_attribute['water'] == True else "LOW"
    read_sensor.sensor_data['water_state'] = log_attribute['water_info']

    # Publish something
    logging.debug("Publishing to Cloud Dashboard")
    logging.debug("data " + str(read_sensor.sensor_data))
    logging.debug("Current delay update : %s second" % (delay_update))

    client.publish('v1/devices/me/attributes',
                   json.dumps(read_sensor.sensor_data), 1)
    client.publish('v1/devices/me/telemetry',
                   json.dumps(read_sensor.sensor_data), 1)


def LED_handle(threadName, delay):
    while(1):
        now = datetime.datetime.now()
        start_split = pumps_state.led_state['start_led'].split(":")
        end_split = pumps_state.led_state['end_led'].split(":")

        if now.hour == int(start_split[0]) and now.minute == int(start_split[1]) and now.second == 0:
            pumps_state.set_led(True)
            logging.debug('LED ON')

        if now.hour == int(end_split[0]) and now.minute == int(end_split[1]) and now.second == 0:
            pumps_state.set_led(False)
            logging.debug('LED OFF')

        time.sleep(delay)


def sensor_realtime():
    global TDS

    log_attribute['PH'] = read_sensor.read_ph()
    log_attribute['EC'] = round(read_sensor.read_ec(), 1)

    if log_attribute['PH'] > 20:
        log_attribute['PH'] = 0
    if log_attribute['EC'] < 30:
        log_attribute['EC'] = 0

    TDS = log_attribute['EC']*500
    try:
        log_attribute['TEMP'] = read_sensor.get_temp()
    except Exception:
        log_attribute['TEMP'] = 25
    # bool(read_sensor.read_water_level())
    log_attribute['water'] = True if read_sensor.read_water_level(
    ) == 0 else False

# def sensor_sim():
#     global TDS

#     if log_attribute['PH'] < 0:
#         log_attribute['PH'] = 0
#     if log_attribute['EC'] < 0:
#         log_attribute['EC'] = 0

#     TDS = log_attribute['EC'] * 0.5
#     log_attribute['TEMP'] = 25
#     log_attribute['water'] = True if read_sensor.read_water_level(
#     ) == 0 else False


def sensor_live(threadName, delay):
    while True:
        global delay_device, delay_update
        global calibrate_ec, calibrate_ph
        global TDS

        print('='*80)
        logging.debug('\n\n' + threadName)

        f = open('/home/comitup/Smart-Hydroponics/config.json')  # for dev
        data = json.load(f)
        delay_device = (data['config']['delay_device'])
        delay_update = data['config']['delay_update']
        calibrate_ec = data['config']['calibrate_ec']
        calibrate_ph = data['config']['calibrate_ph']
        f.close()

        sensor_realtime()
        # sensor_sim()

        logging.debug("running local...")
        logging.debug(pumps_state.pumps_info)
        # logging.debug(pumps_state.led_state)
        logging.debug(log_attribute)

        response = os.system("sudo ping -c 1 -W 3 " + THINGSBOARD_HOST)
        status = 'ONLINE ' if response == 0 else 'OFFLINE'
        mylcd.lcd_display_string('Stat:' + status, 1, 8)

        mylcd.lcd_display_string(
            'PH:' + str('%.1f' % log_attribute['PH']), 1, 0)

        if log_attribute['EC'] != log_attribute['prev_ec']:
            mylcd.lcd_display_string('                ', 3, 3)
        log_attribute['prev_ec'] = log_attribute['EC']
        mylcd.lcd_display_string('                ', 3, 3)
        if read_sensor.sensor_data['ec_tds'] == False:
            mylcd.lcd_display_string(
                'EC:' + str('%.1f' % log_attribute['EC']) + 'us/cm', 3, 0)
        else:
            mylcd.lcd_display_string('TDS: ' + str('%d' % TDS) + 'ppm', 3, 0)

        mylcd.lcd_display_string(
            'Temp:' + str('%.1f' % log_attribute['TEMP']) + ' Water: ', 2, 0)

        if log_attribute['water'] == True:
            mylcd.lcd_display_string("OK ", 2, 16)
        else:
            mylcd.lcd_display_string("LOW ", 2, 16)

        mylcd.lcd_display_string('pH='+str(desired_val['pH']) +
                                 ' EC='+str(desired_val['EC'])+'  ', 4, 0)
        time.sleep(delay)


def sensor_update(threadName, delay):
    while True:
        logging.debug(threadName)
        sensor_realtime()
        now = datetime.datetime.now()
        log_attribute['date_time'] = now.strftime("%Y-%m-%dT%H:%M:%S")
        logging.debug("Current date and time : ")
        logging.debug(log_attribute['date_time'])

        response = os.system("sudo ping -c 1 -W 3 " + THINGSBOARD_HOST)

        if response == 0:
            logging.debug(THINGSBOARD_HOST + ' is UP and reachable!')
            global connected
            connected = True
            if not publish_events():
                logging.debug("Succes send to dashboard" +
                              str(now) + " " + str(now.hour))
            else:
                logging.debug("Failed send to dashboard")

        elif response == 2 or 256 or 512:
            logging.debug(THINGSBOARD_HOST +
                          ' is DOWN and No response from Server!')
            connected = False
            now = datetime.datetime.now()
            log_attribute['date_time'] = now.strftime("%Y-%m-%dT%H:%M:%S")
            logging.debug("Current date and time : ")
            logging.debug(log_attribute['date_time'])
            logging.debug("Current delay : ")
            logging.debug(delay)
        else:
            logging.debug(THINGSBOARD_HOST, 'is DOWN and Host Unreachable!')
            connected = False
            now = datetime.datetime.now()
            log_attribute['date_time'] = now.strftime("%Y-%m-%dT%H:%M:%S")
            logging.debug("Current date and time : ")
            logging.debug(log_attribute['date_time'])
            logging.debug("Current delay : ")
            logging.debug(delay)

        time.sleep(int(delay))


def water_pump(threadName, delay):
    # water pump on/off interval
    logging.debug(threadName)
    while True:
        # cek water state
        error_state['water state'] = True if error_state['water state'] == False else False
        # turn on/off water pump
        pumps_state.set_water(error_state['water state'])
        logging.debug(error_state)

        # log_attribute['EC'] = log_attribute['EC'] - 10

        time.sleep(delay)


def ec_handle(threadName, delay1, delay2):
    logging.debug(threadName)
    while True:
        error_state['ec error'] = log_attribute['EC'] - desired_val['EC']
        if error_state['water state'] == True:
            if error_state['ec error'] >= -50:
                pumps_state.set_nutrient_a(False)
                pumps_state.set_nutrient_b(False)
                logging.debug(
                    'EC Value Achieved, Water Pump turned off after 5 mins left')
                time.sleep(delay1)
                pumps_state.set_water(False)
            else:
                pumps_state.set_nutrient_a(True)
                pumps_state.set_nutrient_b(True)

                # log_attribute['EC'] = log_attribute['EC'] + 20

                time.sleep(1)
                pumps_state.set_nutrient_a(False)
                pumps_state.set_nutrient_b(False)
                time.sleep(delay2)


def ph_handle(threadName, delay1, delay2):
    logging.debug(threadName)
    while True:
        error_state['ph error'] = log_attribute['PH'] - desired_val['pH']
        if error_state['water state'] == True:
            if error_state['ph error'] >= -0.5 and error_state['ph error'] <= 0.5:
                pumps_state.set_acid(False)
                pumps_state.set_alkaline(False)
                logging.debug(
                    'pH Value Achieved, Water Pump turned off after 5 mins left')
                time.sleep(delay1)
                pumps_state.set_water(False)

            if error_state['ph error'] < 0.5:
                pumps_state.set_alkaline(True)
                pumps_state.set_acid(False)

                # log_attribute['PH'] = log_attribute['PH'] + 0.25

                time.sleep(1)
                pumps_state.set_alkaline(False)
                pumps_state.set_acid(False)
                time.sleep(delay2)

            if error_state['ph error'] > -0.5:
                pumps_state.set_alkaline(False)
                pumps_state.set_acid(True)

                # log_attribute['PH'] = log_attribute['PH'] - 0.25

                time.sleep(1)
                pumps_state.set_alkaline(False)
                pumps_state.set_acid(False)
                time.sleep(delay2)

def main_():
    try:
        # time.sleep(30) #bug reading sensor
        _thread .start_new_thread(
            sensor_live, ("RUNNING Thread-sensor-live", int(delay_device), ))
        _thread .start_new_thread(
            sensor_update, ("Thread-sensor-update", int(delay_update), ))
        # _thread.start_new_thread(
        #     LED_handle, ("Thread-LED-handle", 1))
        _thread.start_new_thread(
            water_pump, ("Thread-water-pumps", 60*30))
        _thread.start_new_thread(
            ec_handle, ("RUNNIG THREAD-EC-PUMPS", 60*5, 60))
        _thread.start_new_thread(
            ph_handle, ("RUNNIG THREAD-PH-PUMPS", 60*5, 60))

    except KeyboardInterrupt:
        logging.debug("Error: unable to start thread")

    while 1:
        pass

client.loop_stop()
client.disconnect()
read_sensor.GPIO.cleanup()
