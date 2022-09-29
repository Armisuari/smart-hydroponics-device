#!/usr/bin/env python3

from asyncore import read
from enum import Flag
import json
from telnetlib import EC
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
ACCESS_TOKEN = '6AXNZUy7XiA6UGnMOPAy'

# Timeout to wait for connection
WAIT_CONNECTION_TIMEOUT = 10

# We assume that all GPIOs are LOW
# gpio_state = {7: False, 11: False, 12: False, 13: False, 15: False, 16: False, 18: False, 22: False, 29: False,
#   31: False, 32: False, 33: False, 35: False, 36: False, 37: False, 38: False, 40: False}

connected = False
TDS = 0

desired_val = {'pH': 0, 'EC': 0}


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
    # if data['method'] == 'getGpioStatus':
    #     # Reply with GPIO status
    #     client.publish(msg.topic.replace('request', 'response'), get_gpio_status(), 1)
    # elif data['method'] == 'setGpioStatus':
    #     # Update GPIO status and reply
    #     set_gpio_status(data['params']['pin'], data['params']['enabled'])
    #     client.publish(msg.topic.replace('request', 'response'), get_gpio_status(), 1)
    #     client.publish('v1/devices/me/attributes', get_gpio_status(), 1)

    # if data_in['method'] == 'getPumpsStatus':
    #     client.publish(msg.topic.replace('request', 'response'), get_pumps_status(), 1)
    # elif data_in['method'] == 'setPumpsStatus':
    #     set_pumps_status(data_in['params']['pin'], data_in['params']['enabled'])
    #     client.publish(msg.topic.replace('request', 'response'), get_pumps_status(), 1)
    #     client.publish('v1/devices/me/attributes', get_pumps_status(), 1)
    # logging.debug('////////////////////////////////////////////////////////')
    # logging.debug(data_in['client']['start_led'])
    # logging.debug('////////////////////////////////////////////////////////')

    if msg.topic == 'v1/devices/me/attributes/response/1':
        logging.debug(data_in)

        if list(data_in['shared'])[0] == 'pH' or list(data_in['shared'])[0] == 'EC':
            desired_val['pH'] = data_in['shared']['pH']
            desired_val['EC'] = data_in['shared']['EC']
            logging.debug('store desired pH & EC from platform...')
        # elif list(data_in['shared'])[0] == 'EC':
        #     logging.debug('store desired EC from platform...')
        else:
            pumps_state.led_state['start_led'] = data_in['shared']['start_led']
            pumps_state.led_state['end_led'] = data_in['shared']['end_led']
            # time.sleep(3)

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
        # mylcd.lcd_clear()

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

    #mylcd.lcd_clear()


# def get_gpio_status():
#     # Encode GPIOs state to json
#     return json.dumps(gpio_state)

# def set_gpio_status(pin, status):
#     # Output GPIOs state
#     GPIO.output(pin, GPIO.HIGH if status else GPIO.LOW)
#     # Update GPIOs state
#     gpio_state[pin] = status

# Using board GPIO layout
# GPIO.setmode(GPIO.BOARD)
# GPIO.setwarnings(False)
# for pin in gpio_state:
#     # Set output mode for all GPIO pins
#     GPIO.setup(pin, GPIO.OUT)

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
    # logging.debug()
    logging.debug("Publish Events")
    # logging.debug("================================================")
    # logging.debug()

    # client = get_client()

    # Publish to the events
    # mqtt_topic = f"/devices/{DEVICE_ID}/events"

    # payload = {"{}".format(param_date_device):date_time,
    #            "{}".format(param_name_1):data_1,
    #            "{}".format(param_name_2):data_2,
    #            "{}".format(param_name_3):data_3,
    #            "{}".format(param_name_4):data_4,}

    read_sensor.sensor_data['last_update'] = log_attribute['date_time']
    read_sensor.sensor_data['PH_sensor'] = log_attribute['PH']

    if read_sensor.sensor_data["ec_tds"] == False:
        read_sensor.sensor_data['EC_sensor'] = log_attribute['EC']*1000
    else:
        read_sensor.sensor_data['EC_sensor'] = TDS

    read_sensor.sensor_data['TEMP_sensor'] = log_attribute['TEMP']

    log_attribute['water_info'] = "OK" if log_attribute['water'] == True else "LOW"
    read_sensor.sensor_data['water_state'] = log_attribute['water_info']

    # Publish something
    logging.debug("Publishing to Cloud Dashboard")
    logging.debug("data " + str(read_sensor.sensor_data))
    logging.debug("Current delay update : %s second" % (delay_update))
    # logging.debug()
    # Publish "payload" to the MQTT topic. qos=1 means at least once
    # delivery. Cloud IoT Core also supports qos=0 for at most once
    # delivery.
    # message = client.publish(mqtt_topic, json.dumps(payload), qos=1)
    # message = client.publish('v1/devices/me/attributes', json.dumps(sensor_data), 1)
    client.publish('v1/devices/me/attributes',
                   json.dumps(read_sensor.sensor_data), 1)
    # if message.wait_for_publish():
    #     release_client(client)
    #     return False
    # else:
    #     release_client(client)


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

log_attribute = {'PH':0.0, 'EC':0.0, 'TEMP': 0.0,
                 'prev_ec':0, 'water':False,
                 'date_time':'', 'water_info':''}

def sensor_realtime():
    global TDS

    log_attribute['PH'] = read_sensor.read_ph() + float(calibrate_ph)
    log_attribute['EC'] = read_sensor.read_ec()

    if log_attribute['PH'] > 20: log_attribute['PH'] = 0
    if log_attribute['EC'] < 1: log_attribute['EC'] = 0

    TDS = log_attribute['EC']*500
    try:
        log_attribute['TEMP'] = read_sensor.get_temp()
    except Exception:
        log_attribute['TEMP'] = 25
    # bool(read_sensor.read_water_level())
    log_attribute['water'] = True if read_sensor.read_water_level() == 0 else False


def sensor_live(threadName, delay):
    while True:
        global delay_device, delay_update
        global calibrate_ec, calibrate_ph
        global TDS
        f = open('/home/comitup/Smart-Hydroponics/config.json')  # for dev
        data = json.load(f)
        delay_device = (data['config']['delay_device'])
        delay_update = data['config']['delay_update']
        calibrate_ec = data['config']['calibrate_ec']
        calibrate_ph = data['config']['calibrate_ph']
        f.close()

        sensor_realtime()

        logging.debug("running local...")
        # logging.debug(
        # '\n\n///////////////////////////////////////Pumps state/////////////////////////////////')
        logging.debug(pumps_state.pumps_info)
        logging.debug(pumps_state.led_state)
        # logging.debug(
        # '///////////////////////////////////////////////////////////////////////////////////////\n')

        response = os.system("sudo ping -c 1 -W 3 " + THINGSBOARD_HOST)
        if response == 0:
            stat = 'ONLINE '
        else:
            stat = 'OFFLINE'
        logging.debug("Current delay_device: " + delay_device)

        # mylcd.lcd_display_string('                ', 1,0)
        # mylcd.lcd_display_string('                ', 2,0)
        # mylcd.lcd_display_string('                ', 3,0)
        # mylcd.lcd_display_string('                ', 4,0)

        mylcd.lcd_display_string('PH: ', 1, 0)
        mylcd.lcd_display_string(str('%.1f' % log_attribute['PH']), 1, 3)
        
        EC_e=log_attribute['EC'] * 1000
        if EC_e != log_attribute['prev_ec']:
            mylcd.lcd_display_string('         ', 3, 3)

        log_attribute['prev_ec'] = EC_e

        mylcd.lcd_display_string('         ', 3, 3)
        if read_sensor.sensor_data["ec_tds"] == False:
            mylcd.lcd_display_string('EC: ', 3, 0)
            mylcd.lcd_display_string(str('%.1f' % EC_e), 3, 3)
            mylcd.lcd_display_string('us/cm', 3, 12)
        else:
            mylcd.lcd_display_string('TDS: ', 3, 0)
            mylcd.lcd_display_string(str('%d' % TDS), 3, 4)
            mylcd.lcd_display_string('ppm  ', 3, 12)

        mylcd.lcd_display_string('Temp: ', 2, 0)
        mylcd.lcd_display_string(str('%.1f' % log_attribute['TEMP']), 2, 5)
        mylcd.lcd_display_string('Water:', 2, 10)

        if log_attribute['water'] == True:
            mylcd.lcd_display_string("OK ", 2, 16)
        else:
            mylcd.lcd_display_string("LOW ", 2, 16)

        mylcd.lcd_display_string('Stat: ', 1, 8)
        mylcd.lcd_display_string(stat, 1, 13)

        mylcd.lcd_display_string('Des: ', 4, 0)
        mylcd.lcd_display_string('pH='+str(desired_val['pH']) +
                                 ' EC='+str(desired_val['EC'])+'  ', 4, 5)
        time.sleep(delay)


def sensor_update(threadName, delay):
    while True:
        # Read sensor
        sensor_realtime()
        now = datetime.datetime.now()
        log_attribute['date_time'] = now.strftime("%Y-%m-%dT%H:%M:%S")
        logging.debug("Current date and time : ")
        logging.debug(log_attribute['date_time'])

        response = os.system("sudo ping -c 1 -W 3 " + THINGSBOARD_HOST)

        if response == 0:
            # logging.debug(
            #     "********************************************************************")
            logging.debug(THINGSBOARD_HOST + ' is UP and reachable!')
            # logging.debug(
            #     "********************************************************************")
            # logging.debug("\n")
            global connected
            connected = True
            if not publish_events():
                logging.debug("Succes send to dashboard" +
                              str(now) + " " + str(now.hour))
            else:
                logging.debug("Failed send to dashboard")

        elif response == 2 or 256 or 512:
            # logging.debug(
            #     "********************************************************************")
            logging.debug(THINGSBOARD_HOST +
                          ' is DOWN and No response from Server!')
            # logging.debug(
            #     "********************************************************************")
            # logging.debug("\n")
            connected = False
            now = datetime.datetime.now()
            log_attribute['date_time'] = now.strftime("%Y-%m-%dT%H:%M:%S")
            logging.debug("Current date and time : ")
            logging.debug(log_attribute['date_time'])
            logging.debug("Current delay : ")
            logging.debug(delay)
        else:
            # logging.debug(
            #     "********************************************************************")
            logging.debug(THINGSBOARD_HOST, 'is DOWN and Host Unreachable!')
            # logging.debug(
            #     "********************************************************************")
            # logging.debug("\n")
            connected = False
            now = datetime.datetime.now()
            log_attribute['date_time'] = now.strftime("%Y-%m-%dT%H:%M:%S")
            logging.debug("Current date and time : ")
            logging.debug(log_attribute['date_time'])
            logging.debug("Current delay : ")
            logging.debug(delay)

        time.sleep(int(delay))

water_state = False
def water_pump(threadName, delay):
    logging.debug(threadName)
    global water_state

    while True:
        water_state = True if water_state == False else False
        pumps_state.set_water(water_state)
        logging.debug('water state: ' + str(water_state))
        time.sleep(delay*60)

def pH_handle(threadName, delay):
    logging.debug(threadName)
    global water_state



try:
    _thread .start_new_thread(
        sensor_live, ("Thread-sensor-live", int(delay_device), ))
    # time.sleep(1)  # fix bug lcd
    _thread .start_new_thread(
        sensor_update, ("Thread-sensor-update", int(delay_update), ))
    _thread.start_new_thread(
        LED_handle, ("Thread-LED-handle", 1))
    _thread.start_new_thread(
        water_pump, ("Thread-water-pumps", 1))

except KeyboardInterrupt:
    logging.debug("Error: unable to start thread")

while 1:
    pass

client.loop_stop()
client.disconnect()
read_sensor.GPIO.cleanup()
