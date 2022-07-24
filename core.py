#main code
from asyncore import read
import json
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
from threading import Thread
import _thread
import read_sensor
import time
import datetime
import os
import I2C_LCD_driver

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

THINGSBOARD_HOST = 'demo.thingsboard.io'
ACCESS_TOKEN = '6AXNZUy7XiA6UGnMOPAy'

# Timeout to wait for connection
WAIT_CONNECTION_TIMEOUT = 10

# We assume that all GPIOs are LOW
# gpio_state = {7: False, 11: False, 12: False, 13: False, 15: False, 16: False, 18: False, 22: False, 29: False,
            #   31: False, 32: False, 33: False, 35: False, 36: False, 37: False, 38: False, 40: False}

water_pump = 16
alkaline_pump = 13
acid_pump = 19
nutrient_a = 18
nutrient_b = 12
water_level = 17

connected = False

sensor_data = { 
                "PH_sensor" : 0, "EC_sensor" : 0, "TEMP_sensor" : 0,
                "water_state" : False, "last_update": ""
              }

pumps_data = {
                water_pump: False,
                alkaline_pump: False,
                acid_pump: False,
                nutrient_a: False,
                nutrient_b: False 
            }

def millis():
    return time.time() * 1000

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, rc, *extra_params):
    print('Connected with result code ' + str(rc))
    # Subscribing to receive RPC requests
    client.subscribe('v1/devices/me/rpc/request/+')
    # Sending current GPIO status
    #client.publish('v1/devices/me/attributes', get_gpio_status(), 1)
    client.publish('v1/devices/me/attributes', get_pumps_status(), 1)

    global connected
    connected = True

def on_disconnect(unused_client, unused_userdata, rc):
    """Paho callback for when a device disconnects."""
    print(f"Disconnected with result code " + str(rc))
    print()

    global connected
    connected = False

def on_publish(client, userdata, mid):
    """Paho callback when a message is sent to the broker."""
    print('on_publish')
    print("userdata:" + str(userdata))
    print("mid:" + str(mid))
    print()

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print ('Topic: ' + msg.topic + '\nMessage: ' + str(msg.payload))
    # Decode JSON request
    data = json.loads(msg.payload)
    # Check request method
    # if data['method'] == 'getGpioStatus':
    #     # Reply with GPIO status
    #     client.publish(msg.topic.replace('request', 'response'), get_gpio_status(), 1)
    # elif data['method'] == 'setGpioStatus':
    #     # Update GPIO status and reply
    #     set_gpio_status(data['params']['pin'], data['params']['enabled'])
    #     client.publish(msg.topic.replace('request', 'response'), get_gpio_status(), 1)
    #     client.publish('v1/devices/me/attributes', get_gpio_status(), 1)

    if data['method'] == 'getPumpsStatus':
        client.publish(msg.topic.replace('request', 'response'), get_pumps_status(), 1)
    elif data['method'] == 'setPumpsStatus':
        set_pumps_status(data['params']['pin'], data['params']['enabled'])
        client.publish(msg.topic.replace('request', 'response'), get_pumps_status(), 1)
        client.publish('v1/devices/me/attributes', get_pumps_status(), 1)


# def get_gpio_status():
#     # Encode GPIOs state to json
#     return json.dumps(gpio_state)

def get_pumps_status():
    return json.dumps(pumps_data)

# def set_gpio_status(pin, status):
#     # Output GPIOs state
#     GPIO.output(pin, GPIO.HIGH if status else GPIO.LOW)
#     # Update GPIOs state
#     gpio_state[pin] = status

def set_pumps_status(pin, status):
    GPIO.output(pin, GPIO.HIGH if status else GPIO.LOW)
    pumps_data[pin] = status

# Using board GPIO layout
# GPIO.setmode(GPIO.BOARD)
# GPIO.setwarnings(False)
# for pin in gpio_state:
#     # Set output mode for all GPIO pins
#     GPIO.setup(pin, GPIO.OUT)

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for pin in pumps_data:
    GPIO.setup(pin, GPIO.OUT)
GPIO.setup(water_level, GPIO.IN)

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

PH = 0.0
EC = 0.0
TEMP = 0.0
water = 0
date_time = ''

def publish_events():
    """Publish an event."""
    print()
    print("Publish Events")
    print("================================================")
    print()

    # client = get_client()

    # Publish to the events
    # mqtt_topic = f"/devices/{DEVICE_ID}/events"

    # payload = {"{}".format(param_date_device):date_time,
    #            "{}".format(param_name_1):data_1,
    #            "{}".format(param_name_2):data_2,
    #            "{}".format(param_name_3):data_3,
    #            "{}".format(param_name_4):data_4,}

    sensor_data['last_update'] = date_time
    sensor_data['PH_sensor'] = PH
    sensor_data['EC_sensor'] = EC
    sensor_data['TEMP_sensor'] = TEMP
    sensor_data['water_state'] = water

    # Publish something
    print("Publishing to Cloud Dashboard")
    print("data " + str(sensor_data))
    print("Current delay : %s second" %(delay_device))
    print()
    # Publish "payload" to the MQTT topic. qos=1 means at least once
    # delivery. Cloud IoT Core also supports qos=0 for at most once
    # delivery.
    # message = client.publish(mqtt_topic, json.dumps(payload), qos=1)   
    # message = client.publish('v1/devices/me/attributes', json.dumps(sensor_data), 1)
    client.publish('v1/devices/me/attributes', json.dumps(sensor_data), 1)
    # if message.wait_for_publish():
    #     release_client(client)
    #     return False
    # else:
    #     release_client(client)

def sensor_handle():
    global PH,EC,TEMP,water
    PH = read_sensor.read_ph()
    EC = read_sensor.read_ec()
    try:
        TEMP = read_sensor.get_temp()
    except Exception:
        TEMP = 25
    water = GPIO.input(water_level)

def sensor_live(threadName, delay):
    while True:
        sensor_handle()
        print("running local...")
        
        response = os.system("sudo ping -c 1 -W 3 " + THINGSBOARD_HOST)
        if response == 0:
            stat = 'ONLINE '
        else:
            stat = 'OFFLINE'
        
        mylcd.lcd_display_string('PH: ', 1,0)
        mylcd.lcd_display_string(str('%.1f' % PH), 1,3)
        mylcd.lcd_display_string('EC: ', 1,8)
        mylcd.lcd_display_string(str('%.1f' % EC), 1,11)
        mylcd.lcd_display_string('ms/cm', 1,15)
        mylcd.lcd_display_string('Temp: ', 2,0)
        mylcd.lcd_display_string(str('%.f' % TEMP), 2,5)
        mylcd.lcd_display_string('Stat: ', 4,7)
        mylcd.lcd_display_string(stat, 4,12)
        time.sleep(delay)

def sensor_update(threadName, delay):
    while True:
        #Read sensor
        sensor_handle()
        now = datetime.datetime.now()
        global date_time
        date_time = now.strftime("%Y-%m-%dT%H:%M:%S")
        print ("Current date and time : ")
        print (date_time)

        response = os.system("sudo ping -c 1 -W 3 " + THINGSBOARD_HOST)

        if response == 0:
            print ("********************************************************************")
            print(THINGSBOARD_HOST, 'is UP and reachable!')
            print ("********************************************************************")
            print ("\n")
            global connected
            connected = True
            if not publish_events():
                print("Succes send to dashboard")
            else:
                print("Failed send to dashboard")

        elif response == 2 or 256 or 512:
            print ("********************************************************************")
            print(THINGSBOARD_HOST, 'is DOWN and No response from Server!')
            print ("********************************************************************")
            print ("\n")
            connected = False
            now = datetime.datetime.now()
            date_time = now.strftime("%Y-%m-%dT%H:%M:%S")
            print ("Current date and time : ")
            print (date_time)
            print("Current delay : ")
            print(delay)
        else:
            print ("********************************************************************")
            print(THINGSBOARD_HOST, 'is DOWN and Host Unreachable!')
            print ("********************************************************************")
            print ("\n")
            connected = False
            now = datetime.datetime.now()
            date_time = now.strftime("%Y-%m-%dT%H:%M:%S")
            print ("Current date and time : ")
            print (date_time)
            print("Current delay : ")
            print(delay)

        time.sleep(int(delay))

try:
    # while True:
    #     task_sensor = Thread(target=sensor_hendle(delay_device))
    #     task_sensor.start()
    _thread .start_new_thread( sensor_live, ("Thread-sensor-live", 2, ) )
    time.sleep(1)
    _thread .start_new_thread( sensor_update, ("Thread-sensor-update", int(delay_device), ) )

except KeyboardInterrupt:
    print ("Error: unable to start thread")

while 1:
    pass

client.loop_stop()
client.disconnect()
GPIO.cleanup()