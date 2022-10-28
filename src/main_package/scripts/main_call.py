#!/usr/bin/env python
from __future__ import print_function

import rospy
from status_check.srv import *
from status_check.msg import *
from dashboard.msg import *
from dashboard.srv import *
from std_msgs.msg import *
from main_package.msg import *
from trajectory_planning.srv import StartPainting,StartPaintingResponse,StartTrajCalculation,StartTrajCalculationResponse
from rtde_python.srv import *
from trajectory_planning.msg import CoatCompletionStatus,CoatTrajectoryFeasiblity

from main_package.srv import MainTrigger,MainTriggerResponse


sts = Status_data()
global val
val = Jobs()
map_image = String()


js = ""


def handle_main_function(req):
    resp1 = False
    if(req.mod_selection == "move arm to scanning position"):
        print("move arm to scanning position")
        rospy.wait_for_service('start_ur_service')
        try:
            start_arm_move = rospy.ServiceProxy('start_ur_service', StartRobot)
            start = True
            start_arm_move_resp = start_arm_move(start)
            resp1 = start_arm_move_resp.response
            print(resp1)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            
    if(req.mod_selection == "start arm scanning"):
        rospy.wait_for_service('/rtde_python/arm_start_scanning')
        try:
            rospy.loginfo("inside main call start scanning")
            arm_scanning_start = rospy.ServiceProxy('/rtde_python/arm_start_scanning', StartRobotScanning)
            arm_scanning_start_resp = arm_scanning_start()
            resp1 = arm_scanning_start_resp.response
            
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    if(req.mod_selection == "move arm to painting position"):
        rospy.wait_for_service('/rtde_python/end_to_paint_pos')
        try:
            rospy.loginfo("inside main call move arm to painting position")
            end_to_paint = rospy.ServiceProxy('/rtde_python/end_to_paint_pos', RobotMovePaintPos)
            end_to_paint_resp = end_to_paint()
            resp1 = end_to_paint_resp.response
            
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


    if(req.mod_selection == "call Trajectory Planning"):
        rospy.wait_for_service('start_trajectory_planning_calculation')
        try:
            rospy.loginfo("inside main call Trajectory planning")
            start_trajec_proc = rospy.ServiceProxy(
                'start_trajectory_planning_calculation', StartTrajCalculation)
            start_trajec_proc_resp = start_trajec_proc()
            resp1 = start_trajec_proc_resp.resp
            
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            

    if(req.mod_selection == "sealer" or req.mod_selection == "base1" or req.mod_selection == "base2" or req.mod_selection == "clear1" or req.mod_selection == "clear2"):
        rospy.wait_for_service('start_painting_service')
        try:
            start_paint_coat = rospy.ServiceProxy(
                'start_painting_service', StartPainting)
            start_paint_coat_resp = start_paint_coat(req.mod_selection)
            if(req.mod_selection == "sealer"):
                resp1 = start_paint_coat_resp.sealer

            if(req.mod_selection == "base1"):
                resp1 = start_paint_coat_resp.base1
            
            if(req.mod_selection == "base2"):
                resp1 = start_paint_coat_resp.base2
                
            if(req.mod_selection == "clear1"):
                resp1 = start_paint_coat_resp.clear1
                
            if(req.mod_selection == "clear2"):
                resp1 = start_paint_coat_resp.clear2
                

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


    print("main trig resp",resp1) 
    print("main trig type resp",type(resp1))
    print(MainTriggerResponse(resp1))
    print(type(MainTriggerResponse(resp1)))
    return MainTriggerResponse(resp1)

def status_update():
    if(rospy.has_param('axalta/ccscore/dashboard/RESTART_JOB_DONE')):
        val.RestartJobDone = rospy.get_param('axalta/ccscore/dashboard/RESTART_JOB_DONE')
    else:
       val.RestartJobDone = False
    
    if(rospy.has_param('axalta/ccscore/dashboard/EXIT_JOB_DONE')):
          val.ExitJobDone = rospy.get_param('axalta/ccscore/dashboard/EXIT_JOB_DONE')
    else:
          val.ExitJobDone = False
  
    if(rospy.has_param('axalta/ccscore/dashboard/IMAGE_SEGMENTATION_DONE')):
       val.ImageSegmentationDone = rospy.get_param('axalta/ccscore/dashboard/IMAGE_SEGMENTATION_DONE')
       
    else:
      val.ImageSegmentationDone = False

    if(rospy.has_param("axalta/ccscore/dashboard/IsManuallyCropped")):
      val.isManuallyCropped = rospy.get_param("axalta/ccscore/dashboard/IsManuallyCropped")
    else:
      val.isManuallyCropped = False

    if(rospy.has_param("axalta/ccscore/dashboard/MIRTargetPositionCheck")):
      val.MIRTargetPositionReached = rospy.get_param("axalta/ccscore/dashboard/MIRTargetPositionCheck")
    else:
       val.MIRTargetPositionReached = False

    # if(rospy.has_param("Windows/USB_Check")):
    #   val.MIRTargetPositionReached = rospy.get_param("Windows/USB_Check")
    # else:
    #    val.MIRTargetPositionReached = False


def painting_done_callback(data):
    val.PaintingDone = data.data

def arm_scanning_pos_done_callback(data):
    val.ArmScanningPositionReached = data.data

def arm_painting_pos_done_callback(data):
    val.ArmPaintingPositionReached = data.data

def scanning_done_callback(data):
    val.ScanningDone = data.data
    

def reconstruction_done_callback(data):
    val.ImageStitchingDone = data.data
    val.PointCloudGenerated = data.data

def Trajectory_calculation_completion_callback(data):
    val.TrajectoryPlanningDone = data.data
    val.FeasibilityCheckDone = data.data

def coats_completion_data_callback(data):
    val.SealerCoat.isCompleted = data.sealer
    val.BaseCoat1.isCompleted = data.base1
    val.BaseCoat2.isCompleted = data.base2
    val.ClearCoat1.isCompleted = data.clear1
    val.ClearCoat2.isCompleted = data.clear2


def coats_feasibility_data_callback(data):
     val.SealerCoat.isFeasible = data.sealer_coat_feasiblity
     val.BaseCoat1.isFeasible = data.base1_coat_feasiblity
     val.BaseCoat2.isFeasible = data.base2_coat_feasiblity
     val.ClearCoat1.isFeasible = data.clear1_coat_feasiblity
     val.ClearCoat2.isFeasible = data.clear2_coat_feasiblity
     val.SealerCoat.feasibilityPercentage = data.sealer_coat_ik_percentage
     val.BaseCoat1.feasibilityPercentage = data.base1_coat_ik_percentage
     val.BaseCoat2.feasibilityPercentage = data.base2_coat_ik_percentage
     val.ClearCoat1.feasibilityPercentage = data.clear1_coat_ik_percentage
     val.ClearCoat2.feasibilityPercentage = data.clear2_coat_ik_percentage
     
     
def core_status_data_callback(data):
    global sts
    sts = data


def core_joystick_data_callback(data):
    if(rospy.get_param('axalta/ccscore/dashboard/MANUAL_MODE')):
        global js
        js = data

def map_image_callback(data):
    global map_image
    map_image = data
        
def restart_main():
    js = ""

def get_core_status():
     print("inside get_core_status")
     global pub_map_mir
     pub_map_mir = rospy.Publisher('/main_package/map_image_to_ui', String, queue_size=3)
     rospy.Subscriber("status_check_data", Status_data,core_status_data_callback)
     rospy.Subscriber("corejoystick", String, core_joystick_data_callback)
     rospy.Subscriber("coats_feasibility", CoatTrajectoryFeasiblity, coats_feasibility_data_callback)
     rospy.Subscriber("coat_completion_status", CoatCompletionStatus, coats_completion_data_callback)
     rospy.Subscriber("trajectory_planning_status", Bool, Trajectory_calculation_completion_callback)
     rospy.Subscriber("painting_status", Bool, painting_done_callback)
     rospy.Subscriber("/rtde_python/scanning_done_status_to_main", Bool, scanning_done_callback)
     rospy.Subscriber("/map_mir_to_main", String, map_image_callback)
     rospy.Subscriber('/rtde_python/reconstruction_done_status_to_main', Bool, reconstruction_done_callback)

     rospy.Subscriber('/rtde_python/arm_scanning_pos_reached_status_to_main', Bool, arm_scanning_pos_done_callback)
     rospy.Subscriber('/rtde_python/arm_painting_pos_reached_status_to_main', Bool, arm_painting_pos_done_callback)
    
     pub_job_progress = rospy.Publisher('job_progress_from_main', Jobs, queue_size=3)
     pub_status = rospy.Publisher('status_check_from_main', Status_data, queue_size=3)
     pub_core_joystick = rospy.Publisher('core_joystick_from_main', String, queue_size=3)
     main_service = rospy.Service("main_package/Main_Trigger_server",MainTrigger,handle_main_function)
     
     while not rospy.is_shutdown():
        try:
            status_update()
        except:
            print("Execption in Main_call.py setting sleep 10 Sec")
        if(rospy.has_param("axalta/ccscore/ccs_lite_communicate/EMERGENCY_RESET") or rospy.has_param("axalta/ccscore/dashboard/EXIT_JOB_TRIGGER")):
                if(rospy.get_param("axalta/ccscore/ccs_lite_communicate/EMERGENCY_RESET") or rospy.get_param("axalta/ccscore/dashboard/EXIT_JOB_TRIGGER")):
                    restart_main() 
        pub_map_mir.publish(map_image)
        pub_status.publish(sts)
        pub_core_joystick.publish(js)
        pub_job_progress.publish(val)



if __name__ == "__main__":
    rospy.init_node("main_call")
    get_core_status()
    rospy.spin()
