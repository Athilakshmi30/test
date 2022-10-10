#!/usr/bin/env python

from std_msgs.msg import String
from arm_ctrl_navigate.srv import *
import rospy
import sys
import time

class ArmSettingsSend():
    
     def __init__(self):
         self.arm_settings = String()
               
         self.arm_sett_pub = rospy.Publisher('arm_settings',String, queue_size=1)
         s = rospy.Service('arm_settings_to_default', ArmSettingsResetToDefault, self.handle_arm_settings_to_default)
         self.settings_service_flag = False
         self.arm_settings_func()
         
     def handle_arm_settings_to_default(self,req):
         try:    
             
             if(self.settings_service_flag):
                 print("-------in settings reset--------------")
                 time.sleep(1)
                 rospy.set_param("axalta/ccscore/dashboard/SETTINGS_APPLIED",True)
                 rospy.set_param('axalta/ccscore/dashboard/MOTOR_POWER_ACTION','DEFAULT')
                 self.settings_service_flag = False
                 time.sleep(1)
             else:
                 rospy.set_param("axalta/ccscore/dashboard/SETTINGS_APPLIED",False)    
             
             
             return ArmSettingsResetToDefaultResponse(True)
         except Exception as e:
             print(e)
             return ArmSettingsResetToDefaultResponse(False)
             
     """def handle_arm_settings(self,req): 
         print("-------in settings server--------------")     
          
         print("arm settings",self.arm_settings)
         self.settings_service_flag = True    
         return ArmSettingsServResponse(self.arm_settings)"""             
         
     def arm_settings_func(self):
     
         try:
             
             rate = rospy.Rate(5)
             while not rospy.is_shutdown():
                 if(rospy.has_param('axalta/ccscore/dashboard/MOTOR_POWER_ACTION')):
                      
                     settings_str = rospy.get_param('axalta/ccscore/dashboard/MOTOR_POWER_ACTION')
                     
                     if(settings_str.lower().strip().replace(" ","") == "motoron"):
                         self.arm_settings = "MOTOR ON"
                         self.settings_service_flag = True
                     elif(settings_str.lower().strip().replace(" ","") == "motoroff"):
                         self.arm_settings  = "MOTOR OFF"
                         self.settings_service_flag = True
                     elif(settings_str.lower().strip().replace(" ","") == "powerhigh"):
                         self.arm_settings = "POWER HIGH"
                         self.settings_service_flag = True
                     elif(settings_str.lower().strip().replace(" ","") == "powerlow"):
                         self.arm_settings = "POWER LOW"
                         self.settings_service_flag = True
                     elif(settings_str.lower().strip().replace(" ","") == "home"):
                         self.arm_settings = "HOME"
                         self.settings_service_flag = True
                     elif(settings_str.lower().strip().replace(" ","") == "reset"):
                     
                         self.arm_settings = "RESET"
                         self.settings_service_flag = True
                     else:
                         self.arm_settings = "DEFAULT"
                 else:
                     self.arm_settings = "DEFAULT"   
                 self.arm_sett_pub.publish(self.arm_settings)             
                 rate.sleep()
         
         except Exception as e:
         
             print(e)
             
if __name__ == "__main__":
    try:
        rospy.init_node('arm_settings_send')
        a = ArmSettingsSend()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        sys.exit(0)
    except Exception as e:
        print(e)                                   
                             
