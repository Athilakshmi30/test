#!/usr/bin/env python3
from __future__ import print_function

import rospy
from ai_module.srv import ImageProc
from status_check.srv import *
from status_check.msg import *
from dashboard.msg import *
from dashboard.srv import *
from std_msgs.msg import *
from main_package.msg import *
from move_ur10e.srv import *
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
from rtde_python.srv import *
# OpenCV2 for saving an image
#import cv2
#import base64
from main_package.srv import MainTrigger


sts = Status_data()
global val
val = Jobs()
map_image = String()


js = ""


def handle_main_function(req):
    print("inside main client")
    # if(mod_selection == "start ai module"):
    #     print("inside ai bef try")
    #     rospy.wait_for_service('Start_imageProcessing_server')
    #     try:
    #         print("inside ai")
    #         start_image_proc = rospy.ServiceProxy(
    #             'Start_imageProcessing_server', ImageProc)
    #         start = "start"
    #         resp1 = start_image_proc(start)
    #         print(resp1)
    #         return resp1

    #     except rospy.ServiceException as e:
    #         print("Service call failed: %s" % e)

    if(req.mod_selection == "start arm scanning"):
        print("inside arm scanning main function")
        rospy.wait_for_service('start_ur_service')
        try:
            print("inside ai")
            start_arm_scanning = rospy.ServiceProxy('start_ur_service', StartRobot)
            start = True
            resp1 = start_arm_scanning(start)
            print(resp1)
            return resp1

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
    
    if(req.mod_selection == "call Trajectory"):
        rospy.wait_for_service('Start_Trajectory_calculation_server')
        try:
            start_trajec_proc = rospy.ServiceProxy(
                'Start_Trajectory_calculation_server', StartTrajCalculation)
            start = "start"
            resp1 = start_trajec_proc(start)
            print(resp1)
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    if(req.mod_selection == "sealer" or req.mod_selection == "base1" or req.mod_selection == "base2" or req.mod_selection == "clear1" or req.mod_selection == "clear2"):
        rospy.wait_for_service('Start_Painting_server')
        try:
            start_sealer_coat = rospy.ServiceProxy(
                'Start_Painting_server', StartPainting)
            resp1 = start_sealer_coat(req.mod_selection)
            if(req.mod_selection == "sealer"):
                print("inside sealer client")
                print(resp1.sealer)
                return resp1.sealer
            if(req.mod_selection == "base1"):
                print(resp1.base1)
                return resp1.base1
            if(req.mod_selection == "base2"):
                print(resp1.base2)
                return resp1.base2
            if(req.mod_selection == "clear1"):
                print(resp1.clear1)
                return resp1.clear1
            if(req.mod_selection == "clear2"):
                print(resp1.clear2)
                return resp1.clear2

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


#def status_update():
    # if(rospy.has_param('axalta/ccscore/dashboard/RESTART_JOB_DONE')):
    #     val.RestartJobDone = rospy.get_param(
    #         'axalta/ccscore/dashboard/RESTART_JOB_DONE')
    #else:
    #    val.RestartJobDone = False

    # if(rospy.has_param('axalta/ccscore/dashboard/EXIT_JOB_DONE')):
    #    if(rospy.get_param('axalta/ccscore/dashboard/EXIT_JOB_DONE')):
    #       val.ExitJobDone = True
    #    else:
    #       val.ExitJobDone = False

    # if(rospy.has_param('axalta/ccscore/dashboard/IMAGE_STITCHING_DONE')):
    #     val.ImageStitchingDone = rospy.get_param(
    #         'axalta/ccscore/dashboard/IMAGE_STITCHING_DONE')
    # else:
    #     val.ImageStitchingDone = False

    # if(rospy.has_param('axalta/ccscore/dashboard/IMAGE_SEGMENTATION_DONE')):
    #    val.ImageSegmentationDone = rospy.get_param(
    #       'axalta/ccscore/dashboard/IMAGE_SEGMENTATION_DONE')
    #else:
    #   val.ImageSegmentationDone = False

    #if(rospy.has_param("axalta/ccscore/dashboard/IsManuallyCropped")):
    #   val.isManuallyCropped = rospy.get_param(
    #      "axalta/ccscore/dashboard/IsManuallyCropped")
    #else:
    #   val.isManuallyCropped = False

    #if(rospy.has_param("axalta/ccscore/dashboard/MIRTargetPositionCheck")):
    #   val.MIRTargetPositionReached = rospy.get_param(
    #      "axalta/ccscore/dashboard/MIRTargetPositionCheck")
    #else:
    #    val.MIRTargetPositionReached = False

    #if(rospy.has_param("axalta/ccscore/dashboard/PointCloudGenerated")):
    #   val.PointCloudGenerated = rospy.get_param(
    #      "axalta/ccscore/dashboard/PointCloudGenerated")
    #else:
    #    val.PointCloudGenerated = False

    # if(rospy.get_param("axalta/ccscore/dashboard/FeasibilityCheckDone")):
    #     val.BaseCoat1.isFeasible = True
    #     val.BaseCoat2.isFeasible = True
    #     val.SealerCoat.isFeasible = True
    #     val.ClearCoat1.isFeasible = True
    #     val.ClearCoat2.isFeasible = True
    # else:
    #     val.BaseCoat1.isFeasible = False
    #     val.BaseCoat2.isFeasible = False
    #     val.SealerCoat.isFeasible = False
    #     val.ClearCoat1.isFeasible = False
    #     val.ClearCoat2.isFeasible = False

def painting_done_callback(data):
    val.PaintingDone = data.data

def scanning_done_callback(data):
    val.ScanningDone = data.data
    

def reconstruction_done_callback(data):
    val.ImageStitchingDone = data.data
    val.PointCloudGenerated = data.data

def Trajectory_calculation_completion_callback(data):
    val.TrajectoryPlanningDone = data.data

def Feasibility_check_completion_callback(data):
    val.FeasibilityCheckDone = data.data


def sealer_coat_completion_callback(data):
    val.SealerCoat.isCompleted = data.data

def base_coat_1_completion_callback(data):
     val.BaseCoat1.isCompleted = data.data

def base_coat_2_completion_callback(data):
    val.BaseCoat2.isCompleted = data.data

def clear_coat_1_completion_callback(data):
    val.ClearCoat1.isCompleted = data.data

def clear_coat_2_completion_callback(data):
    val.ClearCoat2.isCompleted = data.data

def sealer_coat_feasibility_callback(data):
    val.ClearCoat2.isFeasible = data.data

def base_coat_1_feasibility_callback(data):
    val.ClearCoat2.isFeasible = data.data

def base_coat_2_feasibility_callback(data):
    val.ClearCoat2.isFeasible = data.data

def clear_coat_1_feasibility_callback(data):
    val.ClearCoat2.isFeasible = data.data

def clear_coat_2_feasibility_callback(data):
    val.ClearCoat2.isFeasible = data.data

def core_status_data_callback(data):
    global sts
    sts = data


def core_joystick_data_callback(data):
    #if(rospy.get_param('axalta/ccscore/dashboard/MANUAL_MODE')):
        global js
        js = data

def map_image_callback(data):
    global map_image
    map_image = data
        



def get_core_status():
     print("inside get_core_status")
     global pub_map_mir
     pub_map_mir = rospy.Publisher('/main_package/map_image_to_ui', String, queue_size=3)
     rospy.Subscriber("status_check_data", Status_data,core_status_data_callback)
     rospy.Subscriber("corejoystick", String, core_joystick_data_callback)
     rospy.Subscriber("move_ur10e/sealer_coat_completion_check",Bool, sealer_coat_completion_callback)
     rospy.Subscriber("move_ur10e/base_coat_1_completion_check", Bool, base_coat_1_completion_callback)
     rospy.Subscriber("move_ur10e/base_coat_2_completion_check", Bool, base_coat_2_completion_callback)
     rospy.Subscriber("move_ur10e/clear_coat_1_completion_check", Bool, clear_coat_1_completion_callback)
     rospy.Subscriber("move_ur10e/clear_coat_2_completion_check", Bool, clear_coat_2_completion_callback)
     rospy.Subscriber("move_ur10e/sealer_coat_feasibility_check",Bool, sealer_coat_feasibility_callback)
     rospy.Subscriber("move_ur10e/base_coat_1_feasibility_check", Bool, base_coat_1_feasibility_callback)
     rospy.Subscriber("move_ur10e/base_coat_2_feasibility_check", Bool, base_coat_2_feasibility_callback)
     rospy.Subscriber("move_ur10e/clear_coat_1_feasibility_check", Bool, clear_coat_1_feasibility_callback)
     rospy.Subscriber("move_ur10e/clear_coat_2_feasibility_check", Bool, clear_coat_2_feasibility_callback)
     rospy.Subscriber("move_ur10e/feasibility_check_completion_status", Bool, Feasibility_check_completion_callback)
     rospy.Subscriber("trajectory_calculation_completion_check", Bool, Trajectory_calculation_completion_callback)
     rospy.Subscriber("painting_done_check", Bool, painting_done_callback)
     rospy.Subscriber("/rtde_python/scanning_done_status_to_main", Bool, scanning_done_callback)
     rospy.Subscriber("/map_mir_to_main", String, map_image_callback)
     rospy.Subscriber('/rtde_python/reconstruction_done_status_to_main', Bool, reconstruction_done_callback)
     pub_job_progress = rospy.Publisher('job_progress_from_main', Jobs, queue_size=3)
     pub_status = rospy.Publisher('status_check_from_main', Status_data, queue_size=3)
     pub_core_joystick = rospy.Publisher('core_joystick_from_main', String, queue_size=3)
     main_service = rospy.Service("main_package/Main_Trigger_server",MainTrigger,handle_main_function)
     
     while not rospy.is_shutdown():
        #status_update()
        pub_map_mir.publish(map_image)
        pub_status.publish(sts)
        pub_core_joystick.publish(js)
        pub_job_progress.publish(val)
      #  pub_map_mir.publish(map_image)


# def pub_status():


if __name__ == "__main__":
    rospy.init_node("main_call")
    #bridge = CvBridge()
    # status_update()
    get_core_status()
    # pub_status()
    rospy.spin()
