#!/usr/bin/env python

from arm_ctrl_navigate.msg import Plannedpath,PathStamped,Path
from arm_ctrl_navigate.srv import *
import rospy
import math

from dashboard.msg import *
from status_check.msg import stats
from mir.srv import *
#from status_check.msg import Status_data
from ccs_lite_communicate.srv import *
from ccs_lite_msgs.msg import CcsLiteCmd,EnclosureStatus,Door
import time

class Emergency():
    
    def __init__(self):
        rospy.init_node('emergency_handler')
        self.node_start_function()
 
    def ccs_lite_command_client_service(self,command):
        rospy.wait_for_service('ccs_lite_command')
        try:
            ccs_lite_command_service = rospy.ServiceProxy(
                'ccs_lite_command', CcsLiteCommand)
            resp1 = ccs_lite_command_service(command)
            print(resp1)
            return True
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)        

    def move_to_destination_recovery(destination):
        rospy.wait_for_service('move_to_destination')
        try:
            
            print("starting to home")
            move_to_destination_recovery_callable = rospy.ServiceProxy(
                'move_to_destination', MoveToDestination)
            resp1 = move_to_destination_recovery_callable(destination)
            print(resp1,"reached home")
            retry_count=0
            while((str(resp1.status).lower() != "completed") and retry_count <= 3):
                resp1 = move_to_destination_recovery_callable(destination)
                print(resp1)
                retry_count = retry_count + 1    
            if(retry_count >= 3):
                return False
            else:
                return True
            return True
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)  
            
    def node_start_function(self):
        rate = rospy.Rate(10)
        rospy.set_param("axalta/ccscore/ccs_lite_communicate/EMERGENCY_RESET",False)
        rospy.set_param("axalta/ccscore/ccs_lite_communicate_EMERGENCY",False)
        
        while not rospy.is_shutdown():
            cmdd = CcsLiteCmd()
           # rospy.sleep(1)
            """if(rospy.has_param("axalta/ccscore/ccs_lite_communicate/EMERGENCY_RESET") and rospy.get_param("axalta/ccscore/ccs_lite_communicate/EMERGENCY_RESET")):
                cmdd.lidar_door_action = True
                cmdd.mir_door_action = True
                cmdd.paint_gun_action = False

                self.ccs_lite_command_client_service(cmdd)
                self.move_to_destination_recovery("home") 
                rospy.set_param("axalta/ccscore/ccs_lite_communicate/EMERGENCY_RESET",False)""" 
            try:
                if(rospy.has_param("axalta/ccscore/dashboard/SOFTWARE_EMERGENCY_STOP") and rospy.has_param("axalta/ccscore/dashboard/ccs_lite_communicate_EMERGENCY")):
                    print("Inside Has Emergency Params")
                    a_b = rospy.get_param('axalta/ccscore/dashboard/SOFTWARE_EMERGENCY_STOP')
                    c_d  = rospy.get_param('axalta/ccscore/dashboard/ccs_lite_communicate_EMERGENCY')
                    if( a_b or c_d):
                        cmdd.lidar_door_action = True
                        cmdd.mir_door_action = True
                        cmdd.paint_gun_action = False
            except:
                print("Inside Emergency Handler Exception set ti 10 Sec")
               # rospy.sleep(10)
            self.ccs_lite_command_client_service(cmdd)
            rate.sleep()                   

if __name__ == "__main__":
    try:
        Emergency()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
