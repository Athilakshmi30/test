#!/usr/bin/env python


import rospy
import time
from ccs_lite_communicate.srv import *
from ccs_lite_msgs.msg import *

rospy.set_param("axalta/ccscore/dashboard/ui_communicator/reset_done",False)
rospy.set_param("axalta/ccscore/ccs_lite_communicate/rostocvimage/reset_done",False)
rospy.set_param("axalta/ccscore/dashboard/reset_handler/reset_done",False)
rospy.set_param("axalta/ccscore/statusCheck/reset_done",False)

mirdoor_cmd = CcsLiteCmd()

def reset_params():
    rospy.set_param("WSL/Windows/LAB", [1111])
    rospy.set_param("WSL/Windows/LAB_SET", False)
    rospy.set_param("Windows/Reconstruction_check", False)
    rospy.set_param("Windows/Segmentation_check", False)
    rospy.set_param("axalta/ccscore/dashboard/AUTONOMOUS_MODE", False)
    rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 0)
    rospy.set_param("axalta/ccscore/dashboard/ChargingStation", "home")
    rospy.set_param("axalta/ccscore/dashboard/EXIT_JOB_DONE", False)
    rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","")
    
    rospy.set_param("axalta/ccscore/dashboard/GOTCONFIRMATION", False)
    rospy.set_param("axalta/ccscore/dashboard/HSV_LOWER", '[40,114,125]')
    rospy.set_param("axalta/ccscore/dashboard/HSV_UPPER", '[43,220,241]')
    rospy.set_param("axalta/ccscore/dashboard/Home", "home")
    rospy.set_param("axalta/ccscore/dashboard/IMAGE_SEGMENTATION_DONE", False)
    rospy.set_param("axalta/ccscore/dashboard/IMAGE_STITCHING_DONE", False)
    rospy.set_param("axalta/ccscore/dashboard/IsManuallyCropped", False)
    rospy.set_param("axalta/ccscore/dashboard/LAB_VALUE", '[0,0,0]')
    rospy.set_param("axalta/ccscore/dashboard/LeftBack", "target")
    rospy.set_param("axalta/ccscore/dashboard/LeftFront", "target")
    rospy.set_param("axalta/ccscore/dashboard/MANUAL_MODE", False)
    rospy.set_param("axalta/ccscore/dashboard/MIRTargetPositionCheck", False)
    rospy.set_param("axalta/ccscore/dashboard/PointCloudGenerated", False)
    rospy.set_param("axalta/ccscore/dashboard/PAINTING_DONE", False)
    rospy.set_param("axalta/ccscore/dashboard/PAINTJOBPROCESS", False)
    rospy.set_param("axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS", "exit")
    rospy.set_param("axalta/ccscore/dashboard/RESTART_JOB_DONE", False)
    rospy.set_param("axalta/ccscore/dashboard/RightBack", "target")
    rospy.set_param("axalta/ccscore/dashboard/RightFront", "target")
    rospy.set_param("axalta/ccscore/dashboard/SCANNINGDONECHECK", False)
    rospy.set_param("axalta/ccscore/dashboard/SETTINGS_APPLIED", False)
    rospy.set_param("axalta/ccscore/dashboard/SOFTWARE_EMERGENCY_STOP", False)
    rospy.set_param("axalta/ccscore/dashboard/SURFACE_MAPPING_DONE", False)
    rospy.set_param("axalta/ccscore/dashboard/TAPE_COLOR", "#8FB74B")
   # rospy.set_param("axalta/ccscore/dashboard/ccs_lite_communicate_EMERGENCY", False)
    rospy.set_param("axalta/ccscore/dashboard/password", "admin")
    rospy.set_param("axalta/ccscore/dashboard/username", "admin")
    rospy.set_param("start_trajectory_calculation", False)
    rospy.set_param("axalta/ccscore/dashboard/ARMScanningPositionReached", False)
    rospy.set_param("axalta/ccscore/dashboard/ARMPaintingPositionReached", False)
    #rospy.set_param('Windows/USB_Check', False)
    rospy.set_param('reset_trajectory_planning', False)
    rospy.set_param("axalta/ccscore/dashboard/ReconstructionDoneCheck",False)
    rospy.set_param("axalta/ccscore/dashboard/SCANNINGDONECHECK",False)
    rospy.set_param("axalta/ccscore/dashboard/reset_trajectory_planning",False)
    rospy.set_param("axalta/ccscore/dashboard/startSegmentationservercalled",False)
    rospy.set_param('axalta/ccscore/dashboard/reset_handler/reset_done', True)

def mir_door_action_client_call_exit(command):
    rospy.wait_for_service('ccs_lite_command')
    try:
        ccs_lite_command_service = rospy.ServiceProxy('ccs_lite_command', CcsLiteCommand)
        resp1 = ccs_lite_command_service(command)
        #print(resp1)
        return True

    except rospy.ServiceException as e:
            print("Service call failed: %s" % e)   

def mir_door_action_cmd_exit():
    mirdoor_cmd.lidar_door_action = False
    mirdoor_cmd.mir_door_action = False
    mirdoor_cmd.paint_gun_action = False
    mir_door_action_client_call_exit(mirdoor_cmd)

if __name__ == '__main__':
    try:
        rospy.init_node('reset_handler', anonymous=True)
        while not rospy.is_shutdown():
            if(rospy.has_param("axalta/ccscore/dashboard/reset_trajectory_planning") and rospy.get_param("axalta/ccscore/dashboard/reset_trajectory_planning")):#making True in login_server node
                    rospy.set_param("axalta/ccscore/ccs_lite_communicate/pointcloud_and_arm_reset",True)#making False in lib_cloud..... node
                    rospy.set_param("axalta/ccscore/dashboard/reset_trajectory_planning",False)

                    #reset_params()
            if(rospy.has_param("axalta/ccscore/dashboard/EXIT_JOB_TRIGGER") and rospy.get_param("axalta/ccscore/dashboard/EXIT_JOB_TRIGGER")):#making True in surface_handler node and making false when axalta/ccscore/dashboard/RESTART_JOB_DONE became True
                    rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","")
                    rospy.set_param("axalta/ccscore/ccs_lite_communicate/pointcloud_and_arm_reset",True)#making False in lib_cloud..... node
                    rospy.set_param("axalta/ccscore/ccs_lite_communicate/core_main_processes",True)
                    mir_door_action_cmd_exit()
                   

            if(rospy.has_param("axalta/ccscore/ccs_lite_communicate/core_main_processes") and rospy.get_param("axalta/ccscore/ccs_lite_communicate/core_main_processes")):
                    
                    rospy.set_param("axalta/ccscore/dashboard/robot_dashbord_restart",True)
                    reset_params()

            if(rospy.get_param("axalta/ccscore/dashboard/ui_communicator/reset_done") and rospy.get_param('axalta/ccscore/ccs_lite_communicate/rostocvimage/reset_done') and rospy.get_param("axalta/ccscore/dashboard/reset_handler/reset_done") and rospy.get_param("axalta/ccscore/statusCheck/reset_done") and rospy.get_param("axalta/ccscore/dashboard/restart_pointcloud_and_arm_completed") and rospy.get_param("axalta/ccscore/robot_dashboard/reset_done")):
                rospy.set_param("axalta/ccscore/dashboard/RESTART_JOB_DONE",True)
                rospy.set_param("axalta/ccscore/ccs_lite_communicate/core_main_processes",False)
                rospy.set_param("axalta/ccscore/dashboard/ui_communicator/reset_done",False)
                rospy.set_param("axalta/ccscore/ccs_lite_communicate/rostocvimage/reset_done",False)
                rospy.set_param("axalta/ccscore/dashboard/reset_handler/reset_done",False)
                rospy.set_param("axalta/ccscore/statusCheck/reset_done",False)
                rospy.set_param("axalta/ccscore/dashboard/restart_pointcloud_and_arm_completed",False)
                rospy.set_param("axalta/ccscore/robot_dashboard/reset_done",False)
                
    

            if(rospy.has_param("axalta/ccscore/dashboard/RESTART_JOB_DONE") and rospy.get_param('axalta/ccscore/dashboard/RESTART_JOB_DONE')):
                time.sleep(3)
                rospy.set_param('axalta/ccscore/dashboard/RESTART_JOB_DONE', False)
                rospy.set_param("axalta/ccscore/dashboard/EXIT_JOB_TRIGGER", False)
    except rospy.ROSInterruptException:
        pass


   