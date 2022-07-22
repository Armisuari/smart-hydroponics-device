# Opening JSON file
import json


#f = open(path_url+'config.json')  #for asus
f = open('config.json') #for dev
data = json.load(f)

delay_device = data['config']['delay'] 
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

print("delay device="+delay_device)
print("connection mode="+connection_mode)
print("param name 1="+param_name_1)
print("param name 2="+param_name_1)
print("param name 3="+param_name_1)
print("param name 4="+param_name_1)
print("Sensor Type="+sensor_type+"\n")

f.close()