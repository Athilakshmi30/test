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

flag = False

def move_to_destination_client(destination):
        rospy.wait_for_service('move_to_destination')
        try:
            
            print("starting to home")
            move_to_destination_client_callable = rospy.ServiceProxy(
                'move_to_destination', MoveToDestination)
            resp1 = move_to_destination_client_callable(destination)
            print(resp1,"reached home")
            retry_count=0
            while((str(resp1.status).lower() != "completed") and retry_count <= 3):
                resp1 = move_to_destination_client_callable(destination)
                print(resp1)
                retry_count = retry_count + 1    
            if(retry_count >= 3):
                return False
            else:
                return True
            return True
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            
def execute_mir():
    if(rospy.has_param("axalta/ccscore/arm_service/NEED_WAIT") and rospy.get_param("axalta/ccscore/arm_service/NEED_WAIT")):            
        time.sleep(11)
        rospy.set_param("axalta/ccscore/arm_service/NEED_WAIT",False)
    else:
        time.sleep(5)     
    result=move_to_destination_client("Home")
    print("mir service execution status:",result)
    return result
                    
if __name__ == "__main__":
    try:
        rospy.init_node('mir_back_home')
        #global flag
        rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_DONE', False)
        while not rospy.is_shutdown():
            if(rospy.has_param("axalta/ccscore/arm_service/ACK_CALLED_SEALER") and rospy.has_param("axalta/ccscore/arm_service/ACK_CALLED_BASE2") and rospy.has_param("axalta/ccscore/arm_service/ACK_CALLED_CLEAR2")):
                  
                if(rospy.get_param("axalta/ccscore/arm_service/ACK_CALLED_SEALER")):
                    rospy.set_param("axalta/ccscore/arm_service/NEED_WAIT",True)
                    #result = execute_mir()
                    time.sleep(11)
                    flag = True 
                    rospy.set_param("axalta/ccscore/arm_service/ACK_CALLED_SEALER",False)
                    rospy.set_param("axalta/ccscore/arm_service/SEALER_COAT_2",True)
                    rospy.set_param("axalta/ccscore/arm_service/SEALER_COAT_1",True)
                    
                if(rospy.get_param("axalta/ccscore/arm_service/ACK_CALLED_BASE2")):
                    rospy.set_param("axalta/ccscore/arm_service/NEED_WAIT",True)
                    #result = execute_mir()
                    time.sleep(11)
                    flag = True 
                    rospy.set_param("axalta/ccscore/arm_service/ACK_CALLED_BASE2",False)
                    rospy.set_param("axalta/ccscore/arm_service/BASE_COAT_2",True)
                    
                if(rospy.get_param("axalta/ccscore/arm_service/ACK_CALLED_CLEAR2")):
                    rospy.set_param("axalta/ccscore/arm_service/NEED_WAIT",True)
                    result = execute_mir()
                    
                    flag = True 
                    rospy.set_param("axalta/ccscore/arm_service/ACK_CALLED_CLEAR2",False)    
                    rospy.set_param("axalta/ccscore/arm_service/CLEAR_COAT_2",True)
                        
                if(rospy.has_param('axalta/ccscore/arm_service/LAST_JOB') and rospy.get_param('axalta/ccscore/arm_service/LAST_JOB') and flag):
                    print("exit job done")
                    rospy.set_param('axalta/ccscore/dashboard/PAINTING_DONE',True)
                    rospy.set_param('axalta/ccscore/arm_service/LAST_JOB',False) 
                    flag = False
                    
            if(rospy.has_param("axalta/ccscore/arm_service/ARM_HOMED_AFTER_RESET")):
                if(rospy.get_param("axalta/ccscore/arm_service/ARM_HOMED_AFTER_RESET")):
                    #execute_mir()
                    #print("mir service execution status:",result)
                    rospy.set_param('axalta/ccscore/arm_service/ARM_LOCK_SITUATION',False)
                    rospy.set_param("axalta/ccscore/arm_service/ARM_HOMED_AFTER_RESET",False)         
                    rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_DONE', True)
                    rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_TRIGGER', True)
                    rospy.set_param("axalta/ccscore/dashboard/LIDARSTART",True)        
               
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
