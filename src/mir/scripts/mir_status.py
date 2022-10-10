#!/usr/bin/env python

import requests
import json
import yaml

with open("/home/axalta_ws/src/mir/scripts/config.yaml", "r") as stream:
            yaml_data = yaml.safe_load(stream)
            
def get_mir_state():
    
    url=yaml_data["host"]+"/status"
    payload={}
    headers={'Content-Type': 'application/json','Authorization': yaml_data["auth"]}

    response = requests.request("GET", url, headers=headers, data=payload, verify=False)
    jresponse=response.json()
    ##print(response.text)
    ##print(jresponse['battery_percentage'])

    return jresponse["state_text"]
    
def get_mir_battery():

    url=yaml_data["host"]+"/status"
    payload={}
    headers={'Content-Type': 'application/json','Authorization': yaml_data["auth"]}

    response = requests.request("GET", url, headers=headers, data=payload, verify=False)
    jresponse=response.json()
    ##print(response.text)
    ##print(jresponse['battery_percentage'])

    return jresponse['battery_percentage']

def get_mir_position():
    
    url=yaml_data["host"]+"/status"

    payload={}
    headers = {'Content-Type': 'application/json','Authorization': yaml_data["auth"]}

    response = requests.request("GET", url, headers=headers, data=payload, verify=False)
    jresponse=response.json()

    #print(jresponse['position']['x'])
    #print(jresponse['position']['y'])
    #print(jresponse['position']['orientation'])

    return jresponse['position']['x'],jresponse['position']['y'],jresponse['position']['orientation']
