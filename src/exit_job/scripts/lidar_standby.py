#!/usr/bin/env python


import rospy
from subprocess import Popen, PIPE
import os
import time

normal_command = 'systemctl start lidar_normal'.split()

#command1 = 'systemctl stop lidar_normal'.split()

standby_command = 'systemctl start lidar_standby'.split()

#command3 = 'systemctl stop lidar_standby'.split()

first = True

sudo_password = 'axalta'

if __name__ == "__main__":
    try:
        rospy.init_node('lidar_standby')
        rate = rospy.Rate(3)
        
        
        while not rospy.is_shutdown():
            
            
            if(rospy.has_param("axalta/ccscore/dashboard/LIDARSTART") and rospy.get_param("axalta/ccscore/dashboard/LIDARSTART")):
                   
                print("----------PUSHING LIDAR TO NORMAL-----------")  
                p = Popen(['sudo','-S']+normal_command,stdin=PIPE,stderr=PIPE,universal_newlines=True)
                sudo_prompt = p.communicate(sudo_password+'\n')[1]
                print("---------------> LIDAR IN NORMAL MODE.... ")
                rospy.set_param("axalta/ccscore/dashboard/LIDARSTART",False)
                rospy.set_param("axalta/ccscore/dashboard/LIDARSTANDBY",True)
                #delay will be required

            if(((rospy.has_param("axalta/ccscore/dashboard/LIDARSTANDBY") and rospy.get_param("axalta/ccscore/dashboard/LIDARSTANDBY")) or first) and rospy.has_param("axalta/ccscore/dashboard/LIDAR_OFF_TRIGGER") and rospy.get_param("axalta/ccscore/dashboard/LIDAR_OFF_TRIGGER")):
                global first
                print("----------PUSHING LIDAR TO STANDBY-----------")  
                p = Popen(['sudo','-S']+standby_command,stdin=PIPE,stderr=PIPE,universal_newlines=True)
                sudo_prompt = p.communicate(sudo_password+'\n')[1]
                print("---------------> LIDAR IN STANDBY MODE.... ")
                first = False
                rospy.set_param("axalta/ccscore/dashboard/LIDARSTART",False)
                rospy.set_param("axalta/ccscore/dashboard/LIDARSTANDBY",False) 
                rospy.set_param("axalta/ccscore/dashboard/LIDAR_OFF_TRIGGER",False)
                #delay will be required
            rate.sleep()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
