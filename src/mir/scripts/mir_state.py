#!/usr/bin/env python

import requests
import json
import yaml

with open("/home/axalta_ws/src/mir/scripts/config.yaml", "r") as stream:
            yaml_data = yaml.safe_load(stream)


def pause_mir():

    url=yaml_data["host"]+"/status"
    payload=json.dumps({"state_id": 4})
    headers={'Content-Type': 'application/json','Authorization': yaml_data["auth"]}

    response = requests.request("PUT", url, headers=headers, data=payload, verify=False)
    jresponse=response.json()

    return jresponse["state_text"]   

def start_mir():

    url=yaml_data["host"]+"/status"
    payload=json.dumps({"state_id": 3})
    headers={'Content-Type': 'application/json','Authorization': yaml_data["auth"]}

    response = requests.request("PUT", url, headers=headers, data=payload, verify=False)
    jresponse=response.json()

    return jresponse["state_text"] 
