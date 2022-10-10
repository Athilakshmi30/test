#!/usr/bin/env python
from __future__ import print_function
import time
import keyboard
import roslibpy
import rospy
from std_msgs.msg import String
import requests
import json
import yaml


class Joystic():
    def __init__(self):

        try:
            # loading yaml

            with open("/home/axalta_ws/src/mir/scripts/config.yaml", "r") as stream:
                self.params = yaml.safe_load(stream)

            # connection MIR
            self.ip = self.params["ip"]
            self.host = 'http://'+self.ip+'/api/v2.0.0/'

            self.headers = {}
            self.headers['Content-Type'] = 'application/json'
            self.headers['Authorization'] = self.params["auth"]

            # retrieve host from param server in future
            self.client = roslibpy.Ros(host=self.ip, port=9090)

            self.client.run()
            if self.client.is_connected:
                print("mir client connected")

            self.talker = roslibpy.Topic(
                self.client, '/joystick_vel', 'std_msgs/String')

            # self.token = self.get_joystick_token()
            self.token = ""
            self.init_node()

        except KeyboardInterrupt:
            print("terminated from keyboard")
            self.talker.unadvertise()
            self.client.terminate()
            # self.client_core.terminate()
        except Exception as e:
            print("Exception : {}".format(e.message))

    def init_node(self):
        
        rospy.init_node("joystic_ros_node")
        rospy.Subscriber("/joystick_cmd", String, self.callback, queue_size=2)
        rospy.spin()

    def getRobotState(self):
        print('Status')

        get_status = requests.get(self.host + 'status', headers=self.headers)
        get_statusJ = json.loads(get_status.text)

        ST_Text = str(get_statusJ['state_text'])
        stateID = int(get_statusJ['state_id'])

        return {"text": ST_Text, "id": stateID}

    def get_joystick_token(self):
        while(1):
            service = roslibpy.Service(self.client, '/mirsupervisor/setRobotState', '/mirsupervisor/setRobotState')
            request = roslibpy.ServiceRequest(dict(robotState=11, web_session_id=self.params["web_sess_id"]))
            result = service.call(request)
            print(result)

            if result["joystick_token"] != "":
                print("TOKEN ----", result)
                return result["joystick_token"]

    def callback(self, data):
        print("\n")
        state = self.getRobotState()
        print(state)
        # manualmode=11
        # ready=3
        # pause=4
        #if data=="nil":
          

        if(state["id"] in [3, 4] and data.data!="nil"):
            print("Robot not in manual control mode")
            self.token = self.get_joystick_token()
        print("recieved message")
        print(" data : {}".format(data.data))

        x_lin = 0.0
        z_ang = 0.0
        if self.client.is_connected:
            if data.data == "up":
                print("Moving forward")
                x_lin = 0.3
            if data.data == "down":
                print("Moving back")
                x_lin = -0.3
            if data.data == "left":
                print("Moving left")
                z_ang = 0.3
            if data.data == "right":
                print("Moving right")
                z_ang = -0.3
            print("x_lin : {}, z_ang: {}".format(x_lin, z_ang))
            self.talker.publish(roslibpy.Message({"joystick_token": self.token, "speed_command": {"linear": {"x": x_lin, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": z_ang}}}))

            #time.sleep(.5)

            print("client connection :", self.client.is_connected)

    def close(self):

        self.talker.unadvertise()

        self.client.terminate()


if __name__ == "__main__":
    joystick = Joystic()
