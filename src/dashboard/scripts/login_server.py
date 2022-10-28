#!/usr/bin/env python
from __future__ import print_function
from __future__ import absolute_import

from dashboard.srv import *
from dashboard.msg import *
import rospy
from mir.srv import *
import time
import base64
import yaml
import threading
import os
from std_msgs.msg import *
#from pathlib import Path

from jobdetailshandler import JobDetailsHandler
from surface_handler import SurfaceHandler
from spraygun_handler import SpraygunHandler
import imageprocesshandler
from main_package.srv import MainTrigger
from ccs_lite_communicate.srv import *
from ccs_lite_msgs.msg import *


mir_action = False

mir_action_painting = False

class Dashboard():

    def __init__(self):
        rospy.init_node('login_server')
        self.coat = 1
        self.init_services()
        
       #  self.init_params()
        # rospy.Subscriber('core_report',Report, self.report_callback)
        # self.report = Report()

    # def function_current_process_start_camera_scanning(self,name):
    #     st_ai = "start ai module"
    #     res = call_modules(st_ai)
    #     if(res):
    #         rospy.set_param("axalta/ccscore/dashboard/PointCloudGenerated",True)
            
    def function_current_process_start_scanning(self,name):
        print("inside arm scanning thread")
        rospy.wait_for_service('main_package/Main_Trigger_server')
        try:
            print("inside arm scanning")
            arm_home_to_scanning_pos = rospy.ServiceProxy('main_package/Main_Trigger_server', MainTrigger)
            start = "move arm to scanning position"
            resp1 = arm_home_to_scanning_pos(start)

            # if(resp1):
            #      rospy.set_param("axalta/ccscore/dashboard/ScanningDone",True)
            # print(resp1)
           # return resp1

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def function_camera_connection_check(self,name):
        cam_conn = os.system("/home/axalta_ws/src/ai_module/sdk_launcher_camera_connection.sh")
        print("called camera connection")
    
    def init_services(self):
        #<----------------------services for ui to core trigger----------------->
        s1 = rospy.Service('login_server', LoginCreds, self.handle_login)

        s2 = rospy.Service('jobDetails_server', JobDetails,self.handle_job_details)

        s3 = rospy.Service('mir_move_to_charging_server', MIRMoveToCharging, self.handle_MIR_moveto_charging)

        s4 = rospy.Service('robotPositionCheck_server',Robotposition, self.handle_robot_position_check)

        s5 = rospy.Service('startScanning_server', StartScanning, self.handle_start_scanning)

        s6 = rospy.Service('updateTapeColor_server', UpdateTapeColor, self.handle_update_tape_color)

        s7 = rospy.Service('getStitchedImage_server', StitchedImage, self.handle_stitched_image)

        s8 = rospy.Service('startSegmentation_server', StartSegmentation, self.handle_start_Segmentation)

        s9 = rospy.Service('getSegmentedImage_server', SegmentedImage, self.handle_segmented_image)

        s10 = rospy.Service('startTrajectoryPlanning_server', StartTrajectoryPlanning, self.handle_start_trajectory_planning)    
        
        s11 = rospy.Service('triggerArmPaintingPosition_server', ArmMoveHome, self.handle_move_arm_to_home)

        s12 = rospy.Service('startPaintProcess_server',StartPainting, self.handle_start_painting)

        #s17 = rospy.Service('skipPaintProcess_server',SpraygunSettingsEdit, self.handle_skip_paint_process)

        #<----------------------services for set and edit settings----------------->
        s13 = rospy.Service('spraygun_settings_server',SpraygunSettings, self.handle_spray_gun)

        s14 = rospy.Service('spraygun_settings_edit_server',SpraygunSettingsEdit, self.handle_spray_gun_edit)

        #<----------------------services for exit and resetting----------------->
        s15 = rospy.Service('exitJob_server', ExitJob, self.handle_exit_job)

        s16 = rospy.Service('resetFeasibilityCheck_server', ResetFeasibilityCheck, self.handle_reset_feasibility_check)
        #<---------------------unused services----------------->

        # s5 = rospy.Service('arm_jointaction_server',ARMJointAction, self.handle_arm_jointaction)

        # s10 = rospy.Service('joystick_config_server',JoystickConfig,self.handle_joystickConfig)

        #s7 = rospy.Service('surface_mapping_image_server', SurfaceMappingImage, self.handle_surfaceMapping)

        #s21 = rospy.Service('mir_map_create_initiate_server',MIRMapCreateInitiate, self.handle_MIR_map_create_initiate)

        #s14 = rospy.Service('startStitching_server', StartStitching, self.handle_start_Stitching)
    
        # s1 = rospy.Service('report_server', Reportservice, self.handle_report)

        

        

        
        
            
    #def init_params(self):
     #   rospy.set_param("axalta/ccscore/dashboard/LIDARSTART", False)
    #    rospy.set_param(
    #        'axalta/ccscore/dashboard/SOFTWARE_EMERGENCY_STOP', False)
     #   rospy.set_param('axalta/ccscore/dashboard/PAINTJOBPROCESS', False)
     #   rospy.set_param('axalta/ccscore/dashboard/GOTCONFIRMATION', False)
      #  rospy.set_param('axalta/ccscore/dashboard/AUTONOMOUS_MODE', False)
       # rospy.set_param('axalta/ccscore/dashboard/MANUAL_MODE', False)
        #rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_TRIGGER', False)
        #rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_DONE', False)
        #rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 0)
        #rospy.set_param("axalta/ccscore/dashboard/SETTINGS_APPLIED",False)
        #rospy.set_param("axalta/ccscore/dashboard/PointCloudGenerated",False)
        #rospy.set_param("axalta/ccscore/dashboard/SCANNINGDONECHECK", False)
        #rospy.set_param("axalta/ccscore/dashboard/MIRTargetPositionCheck", False)
    # def report_callback(self,data):
    #    self.report = data

    def handle_login(self, req):
        rospy.loginfo("login sevice called")
        print('-----------------------login_server------------------------------------------------')
        print('request:', req)
        dashboard_username = rospy.get_param(
            'axalta/ccscore/dashboard/username')
        dashboard_password = rospy.get_param(
            'axalta/ccscore/dashboard/password')
        login_cred_resp = LoginCredsResponse(
            req.username == dashboard_username and req.password == dashboard_password)
        print('response:', login_cred_resp)
        print('time:', time.time())
        camera_thread = threading.Thread(target=self.function_camera_connection_check,args=(1,))
        camera_thread.start()
        # cam_conn = os.system("/home/axalta_ws/src/ai_module/sdk_launcher_camera_connection.sh")
        print("called camera connection")
        return login_cred_resp

    def handle_job_details(self, req):
        obj_jobdetailshandler = JobDetailsHandler()
        return obj_jobdetailshandler.handle_job_details(req)

    def handle_robot_position_check(self, req):
        rospy.loginfo('-----------------------robotPositionCheck_server------------------------------------------------')
        print('request:', req)

        if (req.position_state):
            rospy.set_param('axalta/ccscore/dashboard/MANUAL_MODE', False)
            global mir_action
            mir_action = True
            t2 = threading.Thread(target=self.function_current_process_start_scanning,args=(1,))
            t2.start()
        if(self.coat == 1):
            rospy.set_param('axalta/ccscore/dashboard/PAINTJOBPROCESS', True)
            self.coat = self.coat + 1
        
        print('response:', RobotpositionResponse(True))
        print('time:', time.time())
        return RobotpositionResponse(True)

    def handle_start_scanning(self, req):
        rospy.loginfo('-----------------------startScanning_server------------------------------------------------')
        print('request:', req)

        rospy.wait_for_service('main_package/Main_Trigger_server')
        try:
            print("inside start arm scanning try")
            start_arm_scanning = rospy.ServiceProxy('main_package/Main_Trigger_server', MainTrigger)
            start = "start arm scanning"
            resp1 = start_arm_scanning(start)
            print("response in start scanning is(login server)",resp1.response)
            print("response type in start scanning is(login server)",type(resp1.response))
            return StartScanningResponse(resp1.response)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return StartScanningResponse(False)

    def handle_move_arm_to_home(self, req):
        rospy.loginfo('-----------------------ArmPaintingPosition_server------------------------------------------------')
        print('request:', req)
        rospy.wait_for_service('main_package/Main_Trigger_server')
        try:
            print("inside start ArmPaintingPosition_server try")
            move_arm_to_painting_pos = rospy.ServiceProxy('main_package/Main_Trigger_server', MainTrigger)
            start = "move arm to painting position"
            resp1 = move_arm_to_painting_pos(start)
            return ArmMoveHomeResponse(resp1.response)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return ArmMoveHomeResponse(False)

    def handle_stitched_image(self,req):
        img = imageprocesshandler.handle_stitched_image(req)
        return img

    def handle_update_tape_color(self,req):
        response = imageprocesshandler.handle_update_tape_color(req)
        return response

    def handle_start_Segmentation(self,req):
        response = imageprocesshandler.handle_start_Segmentation(req)
        return response
    
    def handle_segmented_image(self,req):
       img = imageprocesshandler.handle_segmented_image_img(req)
       return img


    """def handle_surfaceMapping(self, req):
        surface_handle = SurfaceHandler()
        response = surface_handle.handle_surfaceMapping(req)
        return response"""

    def handle_start_painting(self, req):
        rospy.loginfo('----------------------startPaintProcess_server-------------------------------------------------')
        #rospy.set_param("")
        # cmd_.lidar_door_action = True
        # cmd_.mir_door_action = True
        # cmd_.paint_gun_action = False
        
        
        surface_handle = SurfaceHandler()
        response = surface_handle.handle_surface_selected(req)
        return response

    def handle_spray_gun(self, req):
        get_spray_settings = SpraygunHandler()
        settings = get_spray_settings.handle_spray_gun(req)
        return settings

    
    def handle_exit_job(self, req):
        surface_handle_exit = SurfaceHandler()
        response = surface_handle_exit.handle_exit_job(req)
        return response
    
    def handle_spray_gun_edit(self, req):
        set_spray_settings = SpraygunHandler()
        edited = set_spray_settings.handle_spray_gun_edit(req)
        return edited
        

    
    
    '''
    def handle_report(self, req):
        print('-----------------------report_server------------------------------------------------')
        print('request:', req)
        print('response:', ReportserviceResponse(self.report))
        print('time:', time.time())
        return ReportserviceResponse(self.report)
    '''
    
    def handle_start_trajectory_planning(self, req):
        rospy.loginfo('----------------start_trajectory_planning_server------------------------------------------------')
        print('time:', time.time())
         
        rospy.wait_for_service('main_package/Main_Trigger_server')
        try:
            print("inside call Trajectory Planning")
            start_Trajectory_planning = rospy.ServiceProxy('main_package/Main_Trigger_server', MainTrigger)
            start_traj = "call Trajectory Planning"
            resp_traj= start_Trajectory_planning(start_traj)
            if(resp_traj):
                 print("Process started")
                 return StartTrajectoryPlanningResponse(True)
            else:
                print("couldn't start Trajectory process")
                return StartTrajectoryPlanningResponse(False)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return StartTrajectoryPlanningResponse(False)

        
    
    def handle_arm_jointaction(self, req):
        rospy.loginfo('-----------------------arm_mir_config_server------------------------------------------------')
        print('request:', req)
        arm_joint_action = str(req.motor_power_action).lower()
        rospy.set_param(
            'axalta/ccscore/dashboard/MOTOR_POWER_ACTION', arm_joint_action)
            
        while(not rospy.get_param("axalta/ccscore/dashboard/SETTINGS_APPLIED")):
            continue
        rospy.set_param("axalta/ccscore/dashboard/SETTINGS_APPLIED",False)        
        print(ARMJointActionResponse(True))
        return ARMJointActionResponse(True)

    def handle_home_pos_config(self, req):
        rospy.loginfo('-----------------------booth_pos_config_server------------------------------------------------')
        print('request:', req)
        print('response:', HomeConfigurationResponse(True))
        return HomeConfigurationResponse(True)

    def handle_MIR_moveto_charging(self, req):
        rospy.loginfo('-----------------------mir_move_to_charging_server------------------------------------------------')
        print('request:', req)
        print("move_to_destination_client(), destination received :charging")
        try:
            obj_jobdetails_handler = JobDetailsHandler()
            return obj_jobdetails_handler.move_to_destination_client("ChargingStation")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return MIRMoveToChargingResponse(False)

    def handle_reset_feasibility_check(self, req):
        rospy.loginfo('-----------------------reset_feasibility_check_server------------------------------------------------')
        print('request:', req)
        rospy.set_param("axalta/ccscore/dashboard/reset_trajectory_planning",True)
        while not rospy.get_param("axalta/ccscore/dashboard/restart_pointcloud_and_arm_completed"):
            pass
        rospy.set_param("axalta/ccscore/dashboard/restart_pointcloud_and_arm_completed",False)
        return ResetFeasibilityCheckResponse(True)

def painting_done_callback_login(data):
    global mir_action_painting
    mir_action_painting = data.data
    #print(mir_action_painting)
    

def mir_door_action_client_call(command):
    rospy.wait_for_service('ccs_lite_command')
    try:
        ccs_lite_command_service = rospy.ServiceProxy('ccs_lite_command', CcsLiteCommand)
        resp1 = ccs_lite_command_service(command)
        #print(resp1)
        return True

    except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

if __name__ == "__main__":
    cmd_ = CcsLiteCmd()
    Dashboard()

    rate = rospy.Rate(20)
    cmd_pub = rospy.Publisher('/ccs_lite_cmd', CcsLiteCmd, queue_size=10)
    rospy.Subscriber("painting_status", Bool, painting_done_callback_login)
    while (not rospy.is_shutdown()):
        #print(mir_action)
        if mir_action:
            cmd_.lidar_door_action = False
            cmd_.mir_door_action = True
            cmd_.paint_gun_action = False
            mir_action = False
            mir_door_action_client_call(cmd_)

        elif mir_action_painting:
            cmd_.lidar_door_action = False
            cmd_.mir_door_action = False
            cmd_.paint_gun_action = False
            mir_action_painting = False
            mir_door_action_client_call(cmd_)

       
        # else:
        #     cmd_.lidar_door_action = False
        #     cmd_.mir_door_action = False
        #     cmd_.paint_gun_action = False
        # cmd_pub.publish(cmd_)
        rate.sleep()

    rospy.spin()
