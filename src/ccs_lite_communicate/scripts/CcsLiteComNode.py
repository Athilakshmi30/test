#!/usr/bin/env python

#from exceptions import KeyboardInterrupt
import rospy
import time
import sys
from std_msgs.msg import Bool,String ,Float32
from ccs_lite_msgs.msg import CcsLiteData, PmsData, ProximityData, TemperatureHumidityData, UltrasonicData, GpsData,Joystick, EnclosureStatus,EmergencyData
from ccs_lite_msgs.msg import SprayGunCmd, LidarEnclosureCmd, MirEnclosureCmd, CcsLiteCmd
from ccs_lite_communicate.srv import *
import WebsocketROSClient as ros_ws
# ws_client = ros_ws.ws_client('192.168.12.45',9090)
# ws_client.connect()

class CcsLiteCom:
    def __init__(self):
        rospy.init_node('ccs_lite_comm', anonymous=False)
        rospy.Subscriber('/ccsLiteCom', CcsLiteData, self.ccsLiteCom_callback)
        rospy.Subscriber('/ccs_lite_emergency', EmergencyData, self.emergency_callback)
        self.b_l = Bool()
        self.ccsLite = CcsLiteData()
        self.ccsLiteCmd = CcsLiteCmd()
        self.ccsLiteCmd.paint_gun_action = False
        self.ccsLiteCmd.lidar_door_action = False
        self.ccsLiteCmd.mir_door_action = False
        self.emergency_data = EmergencyData()
        s = rospy.Service('ccs_lite_command', CcsLiteCommand, self.handle_lite_cmd)
        #s1 = rospy.Service('ccs_lite_mir_door_action_server', CcsLiteCmd, self.handle_lite_cmd_mir_door)
        rospy.set_param('axalta/ccscore/ccs_lite_communicate_EMERGENCY',False)
        rospy.set_param("axalta/ccscore/ccs_lite_communicate/EMERGENCY_RESET",False)
        rospy.set_param("axalta/ccscore/ccs_lite_communicate/pointcloud_and_arm_reset",False)
        rospy.set_param("axalta/ccscore/ccs_lite_communicate/core_main_processes",False) 
        self.com_start()

    def ccsLiteCom_callback(self,data):
        self.ccsLite.pmsData = data.pmsData
        self.ccsLite.gspData = data.gspData
        self.ccsLite.joystickData = data.joystickData
        self.ccsLite.temperatureHumidityData = data.temperatureHumidityData
        self.ccsLite.ultrasonicData = data.ultrasonicData
        self.ccsLite.proximityData = data.proximityData
        self.ccsLite.enclosureStatusData = data.enclosureStatusData

    def handle_lite_cmd_mir_door(self,req):
        print("--------------------ccs_lite_mir_door_action_server------------------------------")
        self.ccsLiteCmd.paint_gun_action = False
        self.ccsLiteCmd.lidar_door_action = True
        self.ccsLiteCmd.mir_door_action = True
        return CcsLiteCmdResponse(True)

    def emergency_callback(self,data):
        self.emergency_data = data
        #print(data)

    def handle_lite_cmd(self,req):
        print("got req")
        print(req)
        self.ccsLiteCmd = req.cmd
        #print(req.cmd)
        return CcsLiteCommandResponse(True)    

    def com_start(self):    
        rate = rospy.Rate(10)
        cmd_pub = rospy.Publisher('/ccs_lite_cmd', CcsLiteCmd, queue_size=10)
        cmd_pub_1 = rospy.Publisher('/ccs_lite_cmd_dupe', CcsLiteCmd, queue_size=10)
        # ws_client.publish('/ccs_lite_cmd',self.ccsLiteCmd)
        pub_wsl = rospy.Publisher('sample_pub', Bool, queue_size=10)#
        pub = rospy.Publisher('corepms', Float32, queue_size=10)#
        pub1 = rospy.Publisher('coregps', String, queue_size=10)#
        pub2 = rospy.Publisher('corejoystick', String, queue_size=10)#
        pub3 = rospy.Publisher('coretemp',Float32, queue_size=10)#
        pub4 = rospy.Publisher('corehum',Float32, queue_size=10)
        #pub4 = rospy.Publisher('coreultrasonic', String, queue_size=10)
        pub5 = rospy.Publisher('coreproximity', Bool, queue_size=10)#
        pub6 = rospy.Publisher('coreenclosure', EnclosureStatus, queue_size=10)
        prev = False
        rospy.set_param("axalta/ccscore/ccs_lite_communicate_EMERGENCY",False)

        while(not rospy.is_shutdown()):
            #print("self.emergency_data.emergencyDetect :", self.emergency_data.emergencyDetect)
            if(self.emergency_data.emergencyDetect.data!=prev):
                if(prev == True):
                    print("prev : ",prev)
                    rospy.set_param("axalta/ccscore/ccs_lite_communicate/EMERGENCY_RESET",True) 
                rospy.set_param("axalta/ccscore/ccs_lite_communicate_EMERGENCY",not (prev))
                print(rospy.get_param("axalta/ccscore/ccs_lite_communicate_EMERGENCY"))
                prev = self.emergency_data.emergencyDetect.data  

            if(rospy.has_param("axalta/ccscore/ccs_lite_communicate/EMERGENCY_RESET") and rospy.get_param("axalta/ccscore/ccs_lite_communicate/EMERGENCY_RESET")):  
                rospy.set_param("axalta/ccscore/ccs_lite_communicate/EMERGENCY_RESET",False)
                rospy.set_param("axalta/ccscore/ccs_lite_communicate/pointcloud_and_arm_reset",True)
                rospy.set_param("axalta/ccscore/ccs_lite_communicate/core_main_processes",True)   
                rospy.set_param("axalta/ccscore/ccs_lite_communicate/mir_door_action_emergency_reset",True)
                print("axalta/ccscore/ccs_lite_communicate/mir_door_action_emergency_reset")
                print(rospy.get_param("axalta/ccscore/ccs_lite_communicate/mir_door_action_emergency_reset")) 

            cmd_pub.publish(self.ccsLiteCmd)
            cmd_pub_1.publish(self.ccsLiteCmd)
            pub.publish(self.ccsLite.pmsData.voltage)
            gps_str = "latitude: " + str(self.ccsLite.gspData.latitude) + " longitude: " + str(self.ccsLite.gspData.longitude)
            pub1.publish(gps_str)
            pub2.publish(self.ccsLite.joystickData)
            pub3.publish(self.ccsLite.temperatureHumidityData.temperatureValue)
            pub4.publish(self.ccsLite.temperatureHumidityData.humidityValue)  
            #pub4.publish(lite.ccsLite.ultrasonicData 
            pub5.publish(self.ccsLite.proximityData.proximityDetect)
            pub6.publish(self.ccsLite.enclosureStatusData)
            pub_wsl.publish(self.b_l)
            rate.sleep()

    def com_restart(self):
        rospy.set_param("axalta/ccscore/ccs_lite_communicate_EMERGENCY",False)
        rospy.set_param("axalta/ccscore/ccs_lite_communicate/EMERGENCY_RESET",True) 
        self.ccsLite = CcsLiteData()
        self.ccsLiteCmd = CcsLiteCmd()
        self.ccsLiteCmd.paint_gun_action = False
        self.ccsLiteCmd.lidar_door_action = True
        self.ccsLiteCmd.mir_door_action = True
        self.emergency_data = EmergencyData()

if __name__ == '__main__':
    try:
        CcsLiteCom()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
