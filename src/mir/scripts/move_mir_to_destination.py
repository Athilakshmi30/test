#!/usr/bin/env python

import requests
import json
import yaml
with open("/home/axalta_ws/src/mir/scripts/config.yaml", "r") as stream:
            yaml_data = yaml.safe_load(stream)

def get_mission_guid():
   ### Get mission Guid
  url=yaml_data["host"]+"/missions"
  payload={}
  headers={'Content-Type': 'application/json','Authorization': yaml_data["auth"]}
  get_missions = requests.request("GET", url, headers=headers, data=payload, verify=False)
  jmissions=get_missions.json()

  return jmissions

def move_to_destination(dest):
  jmissions=get_mission_guid()
  for data in jmissions:
    if(data["name"]==dest):
      mission_guid = data["guid"] 
      
  ### Load Mission to queue  
  url=yaml_data["host"]+"/mission_queue"
  payload = json.dumps({
    "mission_id": mission_guid,
    "parameters": []
  })
  headers = {'Content-Type': 'application/json','Authorization': yaml_data["auth"]}
  response = requests.request("POST", url, headers=headers, data=payload, verify=False)
  jresponse=response.json()
  #print(jresponse)
  print("Mission "+dest +"added to queue....")   
  return response.status_code

def delete_mission_queue():

  url=yaml_data["host"]+"/mission_queue"
  payload = {}
  headers = {'Content-Type': 'application/json','Authorization': yaml_data["auth"]}
  response = requests.request("DELETE", url, headers=headers, data=payload, verify=False)
  print("Mission Queue Deleted!")
