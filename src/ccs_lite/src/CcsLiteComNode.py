#!/usr/bin/env python

from exceptions import KeyboardInterrupt
import rospy
import time
import sys
from ccs_lite_msgs.msg import CcsLiteData, PmsData, ProximityData, TemperatureHumidityData, UltrasonicData, GpsData, Door, Joystick, EnclosureStatus, Axis
from ccs_lite_msgs.msg import SprayGunCmd, LidarEnclosureCmd, MirEnclosureCmd, CcsLiteCmd


class CcsLiteCom:
    def __init__(self):
        rospy.init_node('CcsLiteComNode', anonymous=False)
        self.ccsLitePublisher = rospy.Publisher('/ccsLiteCom', CcsLiteData, queue_size=10)
        self.sprayGunPublisher = rospy.Publisher('/spray_control', SprayGunCmd, queue_size=10)
        self.lidarEncPublisher = rospy.Publisher('/lidar_enclosure_command', LidarEnclosureCmd, queue_size=10)
        self.mirEncPublisher = rospy.Publisher('/mir_enclosure_command', MirEnclosureCmd, queue_size=10)
        self.rate = rospy.Rate(1)
        self.ccsLite = CcsLiteData()
        self.pms = PmsData()
        self.proximity = ProximityData()
        self.tempHum = TemperatureHumidityData()
        self.ultrasonic = UltrasonicData()
        self.gps = GpsData()
        self.js = Joystick()
        self.enc = EnclosureStatus()
        self.sprayCmd = SprayGunCmd()
        self.mirCmd = MirEnclosureCmd()
        self.lidarCmd = LidarEnclosureCmd()
        self.ccsLiteCmd = CcsLiteCmd()
        self.door = Door()

    def publish_data(self):
        self.ccsLite.pmsData = self.pms
        self.ccsLite.gspData = self.gps
        self.ccsLite.joystickData = self.js
        self.ccsLite.temperatureHumidityData = self.tempHum
        self.ccsLite.ultrasonicData = self.ultrasonic
        self.ccsLite.proximityData = self.proximity
        self.ccsLite.enclosureStatusData = self.enc
        

    def gps_callback(self,data):
        self.gps = data
    
    def pms_callback(self,data):
        self.pms = data
 
    def proximity_callback(self,data):
        self.proximity = data
 
    def temphum_callback(self,data):
        self.tempHum = data
       

    def distance_callback(self,data):
        self.ultrasonic = data
 
    def js_callback(self,data):
        self.js = data

    def enc_callback(self,data):
        self.enc = data
 
    def cmd_callback(self,data):
        rospy.loginfo('\nInside Cmd callback')
        self.ccsLiteCmd = data
        print(self.ccsLiteCmd)
        


def gather_data(ccsLiteObj):
    rospy.Subscriber('/gps_data', GpsData, ccsLiteObj.gps_callback)
    rospy.Subscriber('/voltage', PmsData, ccsLiteObj.pms_callback)
    rospy.Subscriber('/ccs_lite_proximity', ProximityData, ccsLiteObj.proximity_callback)
    rospy.Subscriber('/temp_hum_data', TemperatureHumidityData, ccsLiteObj.temphum_callback)
    rospy.Subscriber('/ultrasonic_sensor_data', UltrasonicData, ccsLiteObj.distance_callback)
    rospy.Subscriber('/joystick', Joystick, ccsLiteObj.js_callback)
    rospy.Subscriber('/enclosure_status', EnclosureStatus, ccsLiteObj.enc_callback)
    rospy.Subscriber('/ccs_lite_cmd', CcsLiteCmd, ccsLiteObj.cmd_callback)
    

def publish_cmd(ccsLiteObj):
        
        rospy.loginfo('\nActuator: Spray Gun')
        ccsLiteObj.sprayCmd.sprayAction = ccsLiteObj.ccsLiteCmd.paint_gun_action
        ccsLiteObj.sprayGunPublisher.publish(ccsLiteObj.sprayCmd)

        rospy.loginfo('\nActuator: Mir Doors')
        ccsLiteObj.door.doorAction = ccsLiteObj.ccsLiteCmd.mir_door_action
        ccsLiteObj.mirCmd.mirDoor = ccsLiteObj.door
        ccsLiteObj.mirEncPublisher.publish(ccsLiteObj.mirCmd)

        rospy.loginfo('\nActuator: Lidar Door')
        ccsLiteObj.door.doorAction = ccsLiteObj.ccsLiteCmd.lidar_door_action
        ccsLiteObj.lidarCmd.lidarDoor = ccsLiteObj.door
        ccsLiteObj.lidarEncPublisher.publish(ccsLiteObj.lidarCmd)

        ccsLiteObj.ccsLitePublisher.publish(ccsLiteObj.ccsLite)

if __name__ == '__main__':
    try:
        ccsLiteObj = CcsLiteCom()
        gather_data(ccsLiteObj)
        rate = rospy.Rate(10)
        while(not rospy.is_shutdown()):
            publish_cmd(ccsLiteObj)
            rate.sleep() 
        rospy.spin()
    except(KeyboardInterrupt, SystemExit):
        sys.exit(0)
