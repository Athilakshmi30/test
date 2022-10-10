#!/usr/bin/env python

from __future__ import print_function
from std_msgs.msg import *
from status_check.msg import *
from status_check.srv import *
from sensor_msgs.msg import PointCloud2

from sensor_msgs import point_cloud2
import rospy
from dashboard.srv import *
from dashboard.msg import *
from mir.srv import *
import time
import base64
import yaml
from mir.srv import *
import time
from subprocess import Popen, PIPE
import os
import sys
import cv2
from pathlib import Path
import threading
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from dashboard import *  
#import AdaptiveSeg



#Make sure all imported msg and srv files are available. Give mock application as a ROS2 package. If any function depends on external topic/service give hardcoded values to mock.
global status
status = Status_data() 
js = "I'm Here"

def UI_send():
    print("Inside UI send")
    global js
    global p
    
    p = TypeFloat()
    #global points_
    #pub = rospy.Publisher('UI_data', String, queue_size=10)
    global stat_pub
    global progress_pub
    #global points
    #points = PointCloud2()
    stat_pub = rospy.Publisher('core_status',Status_data,queue_size=10)
    
    joystick_pub = rospy.Publisher('joystick_command',String,queue_size=3)
    joystick_pub_act = rospy.Publisher('joystick_cmd',String,queue_size=3)
    progress_pub = rospy.Publisher('job_progress',Jobs,queue_size=3)
    img_pub = rospy.Publisher('map_mir', String,queue_size=2)
    #rospy.Subscriber("/rgb_pointcloud", PointCloud2, point_cloud_data_callback)
    
    rate = rospy.Rate(100) # 1hz
    while not rospy.is_shutdown():
        #print("Im in")
        #hello_str = "hello world %s" % rospy.get_time() mir;60 ccs:60
        
        #rospy.loginfo(hello_str)
        #print(js)
        #pub.publish(hello_str)
        status.mir.ready= True
        status.mir.running = True
        status.mir.fatalerr = False
        status.mir.remarks = "mir_status"
        status.arm.ready= True
        status.arm.running = True
        status.arm.fatalerr = False
        status.arm.remarks = "arm_status"
        status.ainode.ready= True
        status.ainode.running = True
        status.ainode.fatalerr = False
        status.ainode.remarks = "ainode_status"
        status.lidar.ready= True
        status.lidar.running = True
        status.lidar.fatalerr = False
        status.lidar.remarks = "lidar_status"
        status.sqe_sensors.ready= True
        status.sqe_sensors.running = True
        status.sqe_sensors.fatalerr = False
        status.sqe_sensors.remarks = "sqe_sensors_status"
        status.air_supply.ready= True
        status.air_supply.running = True
        status.air_supply.fatalerr = False
        status.air_supply.remarks = "air_supply_status"
        status.paint_gun.ready= True
        status.paint_gun.running = True
        status.paint_gun.fatalerr = False
        status.paint_gun.remarks = "paint_gun_status"
        status.process.ready= True
        status.process.running = True
        status.process.fatalerr = False
        status.process.remarks = "process_status"
        status.camera.ready= True
        status.camera.running = True
        status.camera.fatalerr = False
        status.camera.remarks = "camera_status"
        status.realsense_camera.ready= True
        status.realsense_camera.running = True
        status.realsense_camera.fatalerr = False
        status.realsense_camera.remarks = "realsense_camera_status"
        status.connectivity = True
        status.arm_lock_flag = False
        status.emergency_flag = False
        #status.completion_percentage = 60
        status.mir_battery = 60
        status.ccs_battery = 60
        status.temperature = "28"
        status.humidity = "80" 
        #status.tape_color = "#ff000" 
        if(rospy.has_param("axalta/ccscore/dashboard/TAPE_COLOR")):
            status.tape_color =  rospy.get_param("axalta/ccscore/dashboard/TAPE_COLOR")
            
        if(rospy.has_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE")):
            status.completion_percentage = rospy.get_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE")
            
        if(rospy.get_param("axalta/ccscore/dashboard/PaintingDone")):
            status.current_process = "Painting has completed"
            #print("Painting has completed")
        
        stat_pub.publish(status)
        js = "I'm Here"
        if(rospy.has_param("axalta/ccscore/dashboard/MANUAL_MODE") and rospy.get_param("axalta/ccscore/dashboard/MANUAL_MODE")):
            joystick_pub.publish(js)
            joystick_pub_act.publish(js)
            js = "I'm Here" 
        else:
            #global js
            joystick_pub.publish(js)
            joystick_pub_act.publish(js)   
        progress_pub.publish(status_update())
        #status_update()
        img_pub.publish(mapmir_img())
        #rate.sleep()
    rospy.spin()

'''def status_check_data_callback(data):#generate mock data here
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    global status
    status = data

def joystick_data_callback(data):#generate mock data here
    if(rospy.get_param('axalta/ccscore/dashboard/MANUAL_MODE')):
        global js
        js = data
    '''
#def report_data_callback(data):
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


    
def status_update():#generate mock data here accordingly
    global val
    val = Jobs()                                                                                       
    val.ObjectIdentificationDone = False
    val.SurfaceMappingDone = False  
    #val.TrajectoryPlanningDone = False 
    #val.PaintingDone = False
    #val.RestartJobDone = False
    #val.ExitJobDone = False
    #val.ImageStitchingDone = False
   # val.ImageSegmentationDone = False
    #val.FeasibilityCheckDone = False 
    #val.BaseCoat1.isFeasible = False            #  M    H
   # val.BaseCoat2.isFeasible = False            #  O    E
   # val.SealerCoat.isFeasible = False          #  C    R         #  K    E
   # val.ClearCoat1.isFeasible = False           #   
   # val.ClearCoat2.isFeasible = False
    #val.BaseCoat1.isCompleted = False            #  M    H
    #val.BaseCoat2.isCompleted = False            #  O    E
    #val.SealerCoat.isCompleted = False          #  C    R         #  K    E
    #val.ClearCoat1.isCompleted = False           #   
    #val.ClearCoat2.isCompleted = False           #  DATA
    if(rospy.get_param("axalta/ccscore/dashboard/TrajectoryPlanningDone")):
         val.TrajectoryPlanningDone = True
    else:
         val.TrajectoryPlanningDone = False
    if(rospy.get_param("axalta/ccscore/dashboard/RestartJobDone")):
         val.RestartJobDone = True
    else:
         val.RestartJobDone = False
    if(rospy.get_param("axalta/ccscore/dashboard/ImageStitchingDone")):
         val.ImageStitchingDone = True
    else:
         val.ImageStitchingDone = False
    if(rospy.get_param("axalta/ccscore/dashboard/FeasibilityCheckDone")):
         val.FeasibilityCheckDone = True
    else:
         val.FeasibilityCheckDone = False
    if(rospy.get_param("axalta/ccscore/dashboard/ImageSegmentationDone")):
         val.ImageSegmentationDone = True
    else:
         val.ImageSegmentationDone = False
    
    if(rospy.get_param("axalta/ccscore/dashboard/EXIT_JOB_DONE")):
         val.ExitJobDone = True
    else:
         val.ExitJobDone = False
         
    if(rospy.get_param("axalta/ccscore/dashboard/IsManuallyCropped")):
         val.isManuallyCropped = True
    else:
         val.isManuallyCropped = False
         
    if(rospy.get_param("axalta/ccscore/dashboard/FeasibilityCheckDone")):
         val.BaseCoat1.isFeasible = True
         val.BaseCoat2.isFeasible = True
         val.SealerCoat.isFeasible = True
         val.ClearCoat1.isFeasible = True
         val.ClearCoat2.isFeasible = True
    else:    
         val.BaseCoat1.isFeasible = False            #  M    H
         val.BaseCoat2.isFeasible = False            #  O    E
         val.SealerCoat.isFeasible = False          #  C    R         #  K    E
         val.ClearCoat1.isFeasible = False           #   
         val.ClearCoat2.isFeasible = False           #  M    H
        
    
    val.SealerCoat.isCompleted = rospy.get_param("axalta/ccscore/arm_service/SEALER_COAT")
    val.BaseCoat1.isCompleted =  rospy.get_param("axalta/ccscore/arm_service/BASE_COAT_1") 
    val.BaseCoat2.isCompleted =  rospy.get_param("axalta/ccscore/arm_service/BASE_COAT_2") 
    val.ClearCoat1.isCompleted = rospy.get_param("axalta/ccscore/arm_service/CLEAR_COAT_1")
    val.ClearCoat2.isCompleted = rospy.get_param("axalta/ccscore/arm_service/CLEAR_COAT_2")
    
    if(rospy.get_param("axalta/ccscore/arm_service/CLEAR_COAT_2")):
         rospy.set_param("axalta/ccscore/dashboard/PaintingDone",True)
         
    if(rospy.get_param("axalta/ccscore/dashboard/PaintingDone")):
         val.PaintingDone = True
    else:
         val.PaintingDone = False
         
                           
    return val
    
def mapmir_img():
    ImgData= ""
    dir_name = os.path.dirname(os.path.abspath(__file__))
    par_of_script = os.path.normpath(os.path.join(dir_name, os.pardir)) 
    par_of_dashbord = os.path.normpath(os.path.join(par_of_script, os.pardir))
    par_of_src = os.path.normpath(os.path.join(par_of_dashbord, os.pardir))           
    filename = os.path.join(par_of_src, 'img/mapimagerotated.png')
    #print("Path of image:")
    #print(filename)
    with open(filename, "rb") as img_file:
        objarea_string = base64.b64encode(img_file.read())
    ImgData = str(objarea_string, 'UTF-8')
    return ImgData 

  
    
    
def function_current_process_robot_position(name):
    status.current_process = "Scanning is in progress.."
    print(status.current_process)
    rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 50)
    time.sleep(5)
    status.current_process = "Scanning has completed"
    print(status.current_process)  
    rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 100)
    time.sleep(1)
    status.current_process = "Area identification is in progress.."
    print(status.current_process)
    rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 50)
    time.sleep(5)
    rospy.set_param("axalta/ccscore/dashboard/ImageStitchingDone",True)
    

def function_current_process_handle_job_details(name):
    status.current_process = "New job has started"
    print(status.current_process)
    
def function_current_process_mir_position(name):
    status.current_process =  "Robot is in motion.."  
    rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 50)
    print(status.current_process)
    
def function_current_process_mir_position_check(name):
    status.current_process =  "Robot is in motion.."  
    rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 50)
    print(status.current_process)

def function_current_process_mir_position_manual(name):
    status.current_process =  "please use joystick to move the robot"  
    print(status.current_process)

def function_current_process_start_segmentation(name):
    time.sleep(5)
    rospy.set_param("axalta/ccscore/dashboard/ImageSegmentationDone",True)
    status.current_process = "Area identification has completed"
    print(status.current_process)
    rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 100)
    
def function_current_process_start_Trajectory_planning(name):
    status.current_process = "Trajectory planning is in progress.."
    print(status.current_process)
    rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 50)
    
    
    time.sleep(5)
   
    status.current_process = "Trajectory planning has completed"
    print(status.current_process)
    rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 100)
    rospy.set_param("axalta/ccscore/dashboard/TrajectoryPlanningDone",True)
    

    time.sleep(5)
    status.current_process = "Feasibility check is in progress.."
    print(status.current_process)
    rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 50)
    
    time.sleep(5)
    status.current_process = "Feasibility check has completed"
    print(status.current_process)
    rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 100)
    rospy.set_param("axalta/ccscore/dashboard/FeasibilityCheckDone",True)      
 
    

def function_current_process_surface_selected(name):
    status.current_process = "Painting process is in progress.."
    print(status.current_process)
    rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 50)
    
def function_current_process_surface_selected_Sealer(name):
    time.sleep(5)
    rospy.set_param("axalta/ccscore/arm_service/SEALER_COAT",True)

def function_current_process_surface_selected_Base_1(name):
    time.sleep(5)
    rospy.set_param("axalta/ccscore/arm_service/BASE_COAT_1",True)
    
def function_current_process_surface_selected_Base_2(name):
    time.sleep(5)
    rospy.set_param("axalta/ccscore/arm_service/BASE_COAT_2",True)
    
def function_current_process_surface_selected_Clear_1(name):
    time.sleep(5)
    rospy.set_param("axalta/ccscore/arm_service/CLEAR_COAT_1",True)
    
def function_current_process_surface_selected_Clear_2(name):
    time.sleep(5)  
    rospy.set_param("axalta/ccscore/arm_service/CLEAR_COAT_2",True)
     

    

def function_current_process_exit_current_job(name):  
    time.sleep(5)
    status.current_process = "please wait while the robot moves back to charging station.." 
    print(status.current_process)
    time.sleep(5)
    rospy.set_param("axalta/ccscore/dashboard/LIDARSTART", False)
    rospy.set_param(
            'axalta/ccscore/dashboard/SOFTWARE_EMERGENCY_STOP', False)
    rospy.set_param('axalta/ccscore/dashboard/PAINTJOBPROCESS', False)
    rospy.set_param('axalta/ccscore/dashboard/GOTCONFIRMATION', False)
    rospy.set_param('axalta/ccscore/dashboard/AUTONOMOUS_MODE', False)
    rospy.set_param('axalta/ccscore/dashboard/MANUAL_MODE', False)
    rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_TRIGGER', False)
    rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_DONE', False)
    rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 0)
    rospy.get_param("axalta/ccscore/dashboard/SETTINGS_APPLIED",False)
    rospy.set_param("axalta/ccscore/dashboard/TAPE_COLOR","#FF0000")
    rospy.set_param("axalta/ccscore/dashboard/TrajectoryPlanningDone",False)
    rospy.set_param("axalta/ccscore/dashboard/RestartJobDone",False)
    rospy.set_param("axalta/ccscore/dashboard/ImageStitchingDone",False)
    rospy.set_param("axalta/ccscore/dashboard/FeasibilityCheckDone",False)
    rospy.set_param("axalta/ccscore/dashboard/ImageSegmentationDone",False)
    rospy.set_param("axalta/ccscore/dashboard/PaintingDone",False)
    rospy.set_param("axalta/ccscore/dashboard/IsManuallyCropped",False)
    #rospy.set_param("axalta/ccscore/dashboard/ExitJobDone",False)
    rospy.set_param("axalta/ccscore/arm_service/BASE_COAT_1",False)
    rospy.set_param("axalta/ccscore/arm_service/BASE_COAT_2",False)
    rospy.set_param("axalta/ccscore/arm_service/CLEAR_COAT_1",False)
    rospy.set_param("axalta/ccscore/arm_service/CLEAR_COAT_2",False)
    rospy.set_param("axalta/ccscore/arm_service/SEALER_COAT",False)
    status.current_process = ""
    rospy.set_param("axalta/ccscore/dashboard/RestartJobDone",True)
    
    time.sleep(3)
    rospy.set_param("axalta/ccscore/dashboard/RestartJobDone",False)

def function_reset_job_deafult_value(name):  
    rospy.set_param("axalta/ccscore/dashboard/LIDARSTART", False)
    rospy.set_param(
            'axalta/ccscore/dashboard/SOFTWARE_EMERGENCY_STOP', False)
    rospy.set_param('axalta/ccscore/dashboard/PAINTJOBPROCESS', False)
    rospy.set_param('axalta/ccscore/dashboard/GOTCONFIRMATION', False)
    rospy.set_param('axalta/ccscore/dashboard/AUTONOMOUS_MODE', False)
    rospy.set_param('axalta/ccscore/dashboard/MANUAL_MODE', False)
    rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_TRIGGER', False)
    rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_DONE', False)
    rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 0)
    rospy.get_param("axalta/ccscore/dashboard/SETTINGS_APPLIED",False)
    rospy.set_param("axalta/ccscore/dashboard/TAPE_COLOR","#FF0000")
    rospy.set_param("axalta/ccscore/dashboard/TrajectoryPlanningDone",False)
    rospy.set_param("axalta/ccscore/dashboard/RestartJobDone",False)
    rospy.set_param("axalta/ccscore/dashboard/ImageStitchingDone",False)
    rospy.set_param("axalta/ccscore/dashboard/FeasibilityCheckDone",False)
    rospy.set_param("axalta/ccscore/dashboard/ImageSegmentationDone",False)
    rospy.set_param("axalta/ccscore/dashboard/PaintingDone",False)
    rospy.set_param("axalta/ccscore/dashboard/IsManuallyCropped",False)
    #rospy.set_param("axalta/ccscore/dashboard/ExitJobDone",False)
    rospy.set_param("axalta/ccscore/arm_service/BASE_COAT_1",False)
    rospy.set_param("axalta/ccscore/arm_service/BASE_COAT_2",False)
    rospy.set_param("axalta/ccscore/arm_service/CLEAR_COAT_1",False)
    rospy.set_param("axalta/ccscore/arm_service/CLEAR_COAT_2",False)
    rospy.set_param("axalta/ccscore/arm_service/SEALER_COAT",False)
    status.current_process = ""
    rospy.set_param("axalta/ccscore/dashboard/RestartJobDone",True)
    
    time.sleep(3)
    rospy.set_param("axalta/ccscore/dashboard/RestartJobDone",False)


def function_current_process_reset_job_progress(name):
    status.current_process = "please wait.." 
    print(status.current_process)

class Dashboard():

    def __init__(self):
        #rospy.init_node('login_server')
        self.coat = 1
        self.jobID = '1'
        self.init_services()
        self.init_params()
        # rospy.Subscriber('core_report',Report, self.report_callback)
        # self.report = Report()

    def init_services(self):
        s1 = rospy.Service('login_server', LoginCreds, self.handle_login)  #gave op
        # s1 = rospy.Service('report_server', Reportservice, self.handle_report)
        s2 = rospy.Service('jobDetails_server', JobDetails,self.handle_job_details)               #gave op
        s3 = rospy.Service('robotPositionCheck_server',
                           Robotposition, self.handle_robot_position_check)     #gave op

        s4 = rospy.Service('spraygun_settings_server',
                           SpraygunSettings, self.handle_spray_gun)     #gave op

        s5 = rospy.Service('spraygun_settings_edit_server',
                            SpraygunSettingsEdit, self.handle_spray_gun_edit)   #gave
        
        #s19 = rospy.Service('surface_mapping_image_server',
         #                   SurfaceMappingImage, self.handle_surfaceMapping)
        s6 = rospy.Service('startPaintProcess_server',
                            SurfaceMappingConfirm, self.handle_surface_selected)     
        # s21 = rospy.Service('mir_map_create_initiate_server',
        #                    MIRMapCreateInitiate, self.handle_MIR_map_create_initiate)
        s7 = rospy.Service('mir_move_to_charging_server',
                            MIRMoveToCharging, self.handle_MIR_moveto_charging)     #gave op
                         
        s8 = rospy.Service('start_segmentation_server', StartSegmentation, self.handle_start_segmentation)
        s9 = rospy.Service('getSegmentedImage_server', SegmentedImage, self.handle_segmented_image)
        s10 = rospy.Service('getStitchedImage_server', StitchedImage, self.handle_stitched_image)
        s11 = rospy.Service('startTrajectoryPlanning_server', StartTrajectoryPlanning, self.handle_start_trajectory_planning)
        s12 = rospy.Service('updateTapeColor_server', UpdateTapeColor, self.handle_update_tape_color)
        s13 = rospy.Service('getPointCloud_server', GetPointCloud, self.handle_get_point_coud) #no need in actual application
        s14 = rospy.Service('exitJob_server', ExitJob, self.handle_exit_job)
        s15 = rospy.Service('resetJobProgress_server', ResetJobProgress, self.handle_reset_job_progress) #no need in actual application
       # s16 = rospy.Service('getTapeImage_server', TapeImage, self.handle_get_tape_image)


    def init_params(self):
        rospy.set_param("axalta/ccscore/dashboard/LIDARSTART", False)
        rospy.set_param(
            'axalta/ccscore/dashboard/SOFTWARE_EMERGENCY_STOP', False)
        rospy.set_param('axalta/ccscore/dashboard/PAINTJOBPROCESS', False)
        rospy.set_param('axalta/ccscore/dashboard/GOTCONFIRMATION', False)
        rospy.set_param('axalta/ccscore/dashboard/AUTONOMOUS_MODE', False)
        rospy.set_param('axalta/ccscore/dashboard/MANUAL_MODE', False)
        rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_TRIGGER', False)
        rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_DONE', False)
        rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 0)
        rospy.get_param("axalta/ccscore/dashboard/SETTINGS_APPLIED",False)
        rospy.set_param("axalta/ccscore/dashboard/TAPE_COLOR","#FF0000")
        rospy.set_param("axalta/ccscore/dashboard/TrajectoryPlanningDone",False)
        rospy.set_param("axalta/ccscore/dashboard/RestartJobDone",False)
        rospy.set_param("axalta/ccscore/dashboard/ImageStitchingDone",False)
        rospy.set_param("axalta/ccscore/dashboard/FeasibilityCheckDone",False)
        rospy.set_param("axalta/ccscore/dashboard/ImageSegmentationDone",False)
        rospy.set_param("axalta/ccscore/dashboard/IsManuallyCropped",False)
        rospy.set_param("axalta/ccscore/dashboard/PaintingDone",False)
        rospy.set_param("axalta/ccscore/arm_service/BASE_COAT_1",False)
        rospy.set_param("axalta/ccscore/arm_service/BASE_COAT_2",False)
        rospy.set_param("axalta/ccscore/arm_service/CLEAR_COAT_1",False)
        rospy.set_param("axalta/ccscore/arm_service/CLEAR_COAT_2",False)
        rospy.set_param("axalta/ccscore/arm_service/SEALER_COAT",False)
         
        


    # def report_callback(self,data):
    #    self.report = data

   #def function_current_process_surface_mir_moving_to_charging(self,name):
    #    time.sleep(2)
     #   mir_resp1 = self.move_to_destination_client("ChargingStation")
    
    def handle_segmented_image(self, req):
        try:
            print(
                "----------------Segmented image service------------------- ")
            print("request: ", req)
            print('time:', time.time())
            is_ManuallyCropped = req.isManuallyCropped
            objectArea1 = SegmentedImageResponse()
            
            if not is_ManuallyCropped:
                 
                 dir_name = os.path.dirname(os.path.abspath(__file__))
                 par_of_script = os.path.normpath(os.path.join(dir_name, os.pardir)) 
                 par_of_dashbord = os.path.normpath(os.path.join(par_of_script, os.pardir))
                 par_of_src = os.path.normpath(os.path.join(par_of_dashbord, os.pardir))           
                 filename = os.path.join(par_of_src, 'img/SegmentedImg.png')
                 print("Path of image:")
                 print(filename)
                 with open(filename, "rb") as img_file:
                     objarea_string = base64.b64encode(img_file.read())
                 objectArea1.ImageData = str(objarea_string, 'UTF-8')
                 
                 
            else:
                 status.current_process = "Updating changes.." 
                 print(status.current_process)
                 rospy.set_param("axalta/ccscore/dashboard/IsManuallyCropped",True)
                 res = self.convert_2d()
                 
                 if(res):
                      
                       dir_name = os.path.dirname(os.path.abspath(__file__))
                       par_of_script = os.path.normpath(os.path.join(dir_name, os.pardir)) 
                       par_of_dashbord = os.path.normpath(os.path.join(par_of_script, os.pardir))
                       par_of_src = os.path.normpath(os.path.join(par_of_dashbord, os.pardir))           
                       filename = os.path.join(par_of_src, 'img/croppedImage.png')
                       print("Path of image:")
                       print(filename)
                       
                       with open(filename, "rb") as img_file:
                           objarea_string = base64.b64encode(img_file.read())
                       objectArea1.ImageData = str(objarea_string, 'UTF-8')
                       
                       
                       
                 else:
                      print("Couldn't create png file")
            
            return objectArea1          
        except Exception as e:
            print(e, "has occurred")
            
            
    def convert_2d(self):
        dir_name = os.path.dirname(os.path.abspath(__file__))
        par_of_script = os.path.normpath(os.path.join(dir_name, os.pardir)) 
        par_of_dashbord = os.path.normpath(os.path.join(par_of_script, os.pardir))
        par_of_src = os.path.normpath(os.path.join(par_of_dashbord, os.pardir))  
        filename = os.path.join(par_of_src, 'share/pointcloud.ply')
        pcd = o3d.io.read_point_cloud(filename)
        filename1 = os.path.join(par_of_src, 'share/pointcloud.pcd')
        o3d.io.write_point_cloud(filename1, pcd)


        pcd1 = o3d.io.read_point_cloud(filename1)

        # Visualize Point Cloud
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(pcd1)

        # Updates
        vis.update_geometry(pcd1)
        vis.poll_events()
        vis.update_renderer()

        # Capture image
       # time.sleep(0.000000000000000000000000000000000000000000001)
        filename2 = os.path.join(par_of_src, 'img/croppedImage.png')
        #vis.capture_screen_image(filename2)
        image = vis.capture_screen_float_buffer(True)
        depth = vis.capture_depth_float_buffer()
        color = np.asarray(image)
        plt.imsave(filename2, np.asarray(color), dpi = 1)
        # image = vis.capture_screen_float_buffer()

        # Close
        vis.destroy_window()
        return True
    
    def handle_stitched_image(self, req):
        try:
            print(
                "----------------Stitched image service------------------- ")
            print("request: ", req)
            print('time:', time.time())
            objectArea2 = StitchedImageResponse()
            dir_name1 = os.path.dirname(os.path.abspath(__file__))
            par_of_script1 = os.path.normpath(os.path.join(dir_name1, os.pardir)) 
            par_of_dashbord1 = os.path.normpath(os.path.join(par_of_script1, os.pardir))
            par_of_src1 = os.path.normpath(os.path.join(par_of_dashbord1, os.pardir))           
            filename = os.path.join(par_of_src1, 'img/StitchedImg.png')
            print("Path of image:")
            print(filename)
            with open(filename, "rb") as img_file:
                objarea_string = base64.b64encode(img_file.read())
            objectArea2.ImageData = str(objarea_string, 'UTF-8')
            return objectArea2
        except Exception as e:
            print(e, "has occurred")        
    
    
    def handle_exit_job(self, req):
        try:
           print("----------------Exit_Job service------------------- ")
           if not rospy.get_param("axalta/ccscore/dashboard/PaintingDone"):
                    rospy.set_param("axalta/ccscore/dashboard/PaintingDone",True)
           self.exit_current_job()   
           return ExitJobResponse(True)       
           
        except Exception as e:
            print(e, "has occurred")  
            return ExitJobResponse(False)
    
    def handle_reset_job_progress(self, req):
        try:
            print("----------------reset_job_progress service------------------- ")
            t19 = threading.Thread(target=function_current_process_reset_job_progress,args=(1,))
            t19.start()
            t25 = threading.Thread(target=function_reset_job_deafult_value,args=(1,))
            t25.start()
           
            print(ResetJobProgressResponse(True))                    
            return ResetJobProgressResponse(True)       
           
        except Exception as e:
            print(e, "has occurred")  
            return ResetJobProgressResponse(False)
    
            
    
    def handle_login(self, req):
        rospy.loginfo("login sevice called")
        print('-----------------------login_server------------------------------------------------')
        print('request:', req)
        print('time:', time.time())
        dashboard_username = "admin"
        dashboard_password = "admin"
        login_cred_resp = LoginCredsResponse(
            req.username == dashboard_username and req.password == dashboard_password)
        print('response:', login_cred_resp)
        print('time:', time.time())
        return login_cred_resp


    def handle_robot_position_check(self, req):
        print('-----------------------robotPositionCheck_server------------------------------------------------')
        print('request:', req)
        
        t1 = threading.Thread(target=function_current_process_robot_position,args=(1,))
        t1.start()
        
        if (req.position_state):
            rospy.set_param('axalta/ccscore/dashboard/MANUAL_MODE', False)
        if(self.coat == 1):
            rospy.set_param('axalta/ccscore/dashboard/PAINTJOBPROCESS', True)
            self.coat = self.coat + 1
        print('response:', RobotpositionResponse(True))
        print('time:', time.time())
        return RobotpositionResponse(True)

    def handle_home_pos_config(self, req):
        print('-----------------------booth_pos_config_server------------------------------------------------')
        print('request:', req)
        print('response:', HomeConfigurationResponse(True))
        return HomeConfigurationResponse(True)

    def handle_MIR_moveto_charging(self, req):
        print('-----------------------mir_move_to_charging_server------------------------------------------------')
        print('request:', req)
        print("move_to_destination_client(), destination received :charging")
        try:
            return self.move_to_destination_client("ChargingStation")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return MIRMoveToChargingResponse(False)

    def handle_start_segmentation(self, req):
        rospy.loginfo("start Segmentation sevice called")
        print('-----------------------start_segmentation_server------------------------------------------------')
        print('response:', StartSegmentationResponse(True) )
        print('time:', time.time())
        t4 = threading.Thread(target=function_current_process_start_segmentation,args=(1,))
        t4.start()
        
        return StartSegmentationResponse(True)
    
    def handle_start_trajectory_planning(self, req):
        rospy.loginfo("start Trajectory sevice called")
        print('----------------start_trajectory_planning_server------------------------------------------------')
        print('response:', StartTrajectoryPlanningResponse(True) )
        print('time:', time.time())
        t7 = threading.Thread(target=function_current_process_start_Trajectory_planning,args=(1,))
        t7.start()
        return StartTrajectoryPlanningResponse(True)
         
           
    
    def handle_update_tape_color(self, req):
        try:
            print(
                "----------------Update Tape color Service------------------- ")
            print("request: ", req)
            print('time:', time.time())
            obj_as = AdaptiveSeg.AdaptiveSegCl()
            
            if(req.isTemp):
                  rospy.set_param("axalta/ccscore/dashboard/TAPE_COLOR",req.tape_color)
                  status.tape_color = req.tape_color
                  tapeColor_is = req.tape_color
                  obj_as.Tape_Tracker("/home/superadmin/mock_ws/src/Segmentation/V2.1/Data/color",tapeColor_is)
            else:
                  status.tape_color = req.tape_color
                  tapeColor_is = req.tape_color
            print("response: ", UpdateTapeColorResponse(True))
            return UpdateTapeColorResponse(True)     
        except Exception as e:
            print(e, "has occurred")


    
    
    def handle_get_point_coud(self,req):
        print('----------------get_pointCloud_server------------------------------------------------')
        print('time:', time.time())
        
        typ_ = TypeFloat()
        resp = GetPointCloudResponse()
        p_list = []
        #print('response:', GetPointCloudResponse(points))
        #print(sub)
        for i in points_:
            #print(i)
            poi_ = Points()
            poi_.pointvar = i
            #print(poi_)
            typ_.data.append(poi_) 
            #print(p_list)
        #resp.pointCloudData = p_list
        #print(typ_)
        #print(p_list)
        resp.pointCloudData = typ_
        return resp 
    
    
    
    
    def set_robot_operating_mode(self, mode):

        try:

            if mode == 1:  # autonomous
                rospy.set_param('axalta/ccscore/dashboard/MANUAL_MODE', False)
                rospy.set_param(
                    'axalta/ccscore/dashboard/AUTONOMOUS_MODE', True)
            elif mode == 0:  # manual
                rospy.set_param('axalta/ccscore/dashboard/MANUAL_MODE', True)
                rospy.set_param(
                    'axalta/ccscore/dashboard/AUTONOMOUS_MODE', False)
            return True
        except Exception as e:
            print("Error : ", e)
            return False

    def set_robot_to_manualmode(self):
        print("set_robot_to_manualmode()")
        
        t8.start()
        status = self.set_robot_operating_mode(0)  # 1 = autonomous, 0= manual
        print("response :", status)
        return JobDetailsResponse(status)

    def move_to_target(self, destination):
        mir_resp = False
        #t3.start()
        if(destination == "leftfront"):
            mir_resp = self.move_to_destination_client("LeftFront")
            rospy.set_param(
            'axalta/ccscore/dashboard/MIR_PAINTBOOTH_LOCATION', "LeftFront")
            print("target reached")
            print(mir_resp)
            return mir_resp
        elif(destination == "leftback"):
            mir_resp = self.move_to_destination_client("LeftBack")
            rospy.set_param(
            'axalta/ccscore/dashboard/MIR_PAINTBOOTH_LOCATION', "LeftBack")
            print("target reached")
            return mir_resp
        elif(destination == "rightfront"):
            mir_resp = self.move_to_destination_client("RightFront")
            rospy.set_param(
            'axalta/ccscore/dashboard/MIR_PAINTBOOTH_LOCATION', "RightFront")
            print("target reached")
            return mir_resp
        elif(destination == "rightback"):
            mir_resp = self.move_to_destination_client("RightBack")
            rospy.set_param(
            'axalta/ccscore/dashboard/MIR_PAINTBOOTH_LOCATION', "RightBack")
            print("target reached")
            return mir_resp
                
        else:
            # add other cases in future or use switch case
            return False

    def init_paint_process_params(self, destination):
        #rospy.set_param(
        #    'axalta/ccscore/dashboard/MIR_PAINTBOOTH_LOCATION', destination)
        rospy.set_param(
            'axalta/ccscore/dashboard/JOBID', self.jobID)
        # rospy.set_param("axalta/ccscore/dashboard/NEWJOB", True)
        rospy.set_param(
            "axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 50)
        self.jobID = str(int(self.jobID) + 1)
        rospy.set_param(
            'axalta/ccscore/dashboard/PAINTING_DONE', False)

    def robot_auto_functions(self, destination):
        
        try:
            
            # setting robot to auto mode
            self.set_robot_operating_mode(1)  # 1 = autonomous, 0= manual
            # moving robot to destination
            
            mir_resp = self.move_to_target(destination)
            #mir_resp = True  
            if(mir_resp):
                # safety delay until vibrations die out to enable LIDAR take better pointclouds
                time.sleep(2)
                # initializing parameters for painting process
                self.init_paint_process_params(destination)
                #print('response from fn:', True)
                return True
            else:
                print('response:', False)
                return False
                
        except Exception as e:
            print("exception message :", e)
            return False

    def move_to_destination_client(self, destination):
        #rospy.wait_for_service('move_to_destination')
       # t21 = threading.Thread(target=function_current_process_mir_position,args=(1,))
        print("move_to_destination_client(), destination received :", destination)
        #if(destination == "ChargingStation"):
        #     t21.start()
         #    t21.join()
       
        t23.start()
             #t23.join()

        resp1 = "completed"
        #resp1 = "failed"

        print(resp1)
        retry_count = 0
        while(resp1 != "completed" and retry_count < 3):
             resp1 = "completed"
             #resp1 = "failed"
             print(resp1)
             retry_count = retry_count + 1
        if(retry_count >= 3):
                return False
        else:
                return True
        #return True


    def handle_job_details(self, req):
        print('-----------------------handle_job_details------------------------------------------------')
        try:
            print('request:', req)
            print('time:', time.time())
            #status = Status_data()
            t2 = threading.Thread(target=function_current_process_handle_job_details,args=(1,))
            t2.start()
            
            global t3
            global t8
            global t23
            t3 = threading.Thread(target=function_current_process_mir_position,args=(1,))
            t23 = threading.Thread(target=function_current_process_mir_position_check,args=(1,))
            t8 = threading.Thread(target=function_current_process_mir_position_manual,args=(1,))
            #  robot manual mode operation
            if(req.operating_mode.lower() == 'manual'):
                #time.sleep(5)
                return self.set_robot_to_manualmode()

            #  robot  autonomous mode operations
            elif(req.operating_mode.lower() == 'autonomous'):
                destination = str(req.mir_paintbooth_location).lower().replace(" ","")
                status = self.robot_auto_functions(destination)
                #status = True
                #time.sleep(5)
                print("response :", status)
                return JobDetailsResponse(status)
            #return True
            

        except Exception as e:
            print(e, "has occurred")
            return JobDetailsResponse(False)
            

    def update_yaml(self, data):

        with open("/home/superadmin/axalta_ws/src/dashboard/config/dashboard_persistent_settings.yaml", 'r+') as file:
            doc = yaml.safe_load(file)
            if(list(data.keys())[0] in doc.keys()):
                del doc[list(data.keys())[0]]
            file.seek(0)
            documents = yaml.dump(doc, file, default_flow_style=False)
            file.truncate()
        with open("/home/superadmin/axalta_ws/src/dashboard/config/dashboard_persistent_settings.yaml", 'a') as file:
            documents = yaml.dump(data, file, default_flow_style=False)

    def get_coat_settings(self, index):
        libe = ['sealercoat', 'basecoat1',
                'basecoat2', 'clearcoat1', 'clearcoat2']
        coat_num = libe[index]
      #left,right,top,bottom offset added 
        set_list_ = ['axalta/ccscore/dashboard/SPRAYGUN_NAME_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_NO_OF_LAYERS_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_AIR_PRESSURE_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_COAT_NUMBER_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_INDEX_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_DELAY_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_LEFT_OFFSET_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_RIGHT_OFFSET_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_TOP_OFFSET_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_BOTTOM_OFFSET_' + coat_num,
                     ]

        return set_list_

    def read_settings_params(self, index):
        get_settings = []
        set_list_ = self.get_coat_settings(index)
        for k in set_list_:
            get_settings.append(rospy.get_param(k))
        return get_settings

    def handle_spray_gun(self, req):
        print('-----------------------spraygun_settings_server------------------------------------------------')
        print('request:', req)
        print('response:', 'True')
        print('time:', time.time())
     #left,right,top,bottom offset added 
        settings = SpraygunSettingsResponse()
        for i in range(5):
            sett = GunSettings()
            get_settings = self.read_settings_params(i)
            sett.name = get_settings[0]
            sett.no_of_layers = get_settings[1]
            sett.air_pressure = get_settings[2]
            sett.coat_number = get_settings[3]
            sett.traverse_speed = get_settings[4]
            sett.index = get_settings[5]
            sett.gun_distance1 = get_settings[6]
            sett.delay = get_settings[7]
            sett.left_offset=get_settings[8]
            sett.right_offset=get_settings[9]
            sett.top_offset=get_settings[10]
            sett.bottom_offset=get_settings[11]
            settings.spraygun_settings.append(sett)

        print('response:', settings)
        print('time:', time.time())
        return settings

    def load_spraygun_params(self, set_dict_):
        try:
            for k, v in set_dict_.items():
                self.update_yaml({k: v})

            for k, v in set_dict_.items():
                rospy.set_param(k, v)
            return True

        except Exception as e:
            print("Exception occured :", e)
            return False

    def handle_spray_gun_edit(self, req):
        print('-----------------------spraygun_settings_edit_server------------------------------------------------')
        print('request:', req)

        print('time:', time.time())
        if(req.coat_number > 0):
            coat_num = str(req.name).lower() + str(req.coat_number)
        else:
            coat_num = str(req.name).lower()
        #left,right,top,bottom offset added 
        set_dict_ = {'axalta/ccscore/dashboard/SPRAYGUN_NAME_' + coat_num: req.name,
                     'axalta/ccscore/dashboard/SPRAYGUN_NO_OF_LAYERS_' + coat_num: req.no_of_layers,
                     'axalta/ccscore/dashboard/SPRAYGUN_AIR_PRESSURE_' + coat_num: req.air_pressure,
                     'axalta/ccscore/dashboard/SPRAYGUN_COAT_NUMBER_' + coat_num: req.coat_number,
                     'axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_' + coat_num: req.traverse_speed,
                     'axalta/ccscore/dashboard/SPRAYGUN_INDEX_' + coat_num: req.index,
                     'axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_' + coat_num: req.gun_distance1,
                     'axalta/ccscore/dashboard/SPRAYGUN_DELAY_' + coat_num: req.delay,
                     'axalta/ccscore/dashboard/SPRAYGUN_LEFT_OFFSET_' + coat_num: req.left_offset,
                     'axalta/ccscore/dashboard/SPRAYGUN_RIGHT_OFFSET_' + coat_num: req.right_offset,
                     'axalta/ccscore/dashboard/SPRAYGUN_TOP_OFFSET_' + coat_num: req.top_offset,
                     'axalta/ccscore/dashboard/SPRAYGUN_BOTTOM_OFFSET_' + coat_num: req.bottom_offset,
                     }
       
        param_status = self.load_spraygun_params(set_dict_)
        print('response:', SpraygunSettingsEditResponse(param_status))
        return SpraygunSettingsEditResponse(param_status)


    def handle_surfaceMapping(self, req):
        try:
            print(
                "----------------surface mapping image service------------------- ")
            print("request: ", req)
            objectArea = SurfaceMappingImageResponse()
            with open("/home/super_admin/axalta_ws/img/image_to_ui.png", "rb") as img_file:
                objarea_string = base64.b64encode(img_file.read())
            objectArea.image_data = objarea_string
            points = ContourPoints()
            pts = ImageCoord()
            pts1 = ImageCoord()
            pts2 = ImageCoord()
            pts3 = ImageCoord()
            pts.x = 1198
            pts.y = 43
            pts1.x = 1090
            pts1.y = 42
            pts2.x = 1093
            pts2.y = 49
            pts3.x = 1190
            pts3.y = 46
            points.point_list.append(pts)
            points.point_list.append(pts1)
            points.point_list.append(pts2)
            points.point_list.append(pts3)
            objectArea.boundary = points
            print(points)
            rospy.set_param(
                'axalta/ccscore/dashboard/OBJECT_IDENTIFICATION_DONE', True)
            return objectArea
        except Exception as e:
            print(e, "has occurred")

    def surface_confirm_set_params(self, mir_resp, paint_proc, coat_num):
        t5.start()
        if(mir_resp or coat_num == 2):
            rospy.set_param(
                'axalta/ccscore/dashboard/GOTCONFIRMATION', True)
            rospy.set_param(
                'axalta/ccscore/dashboard/CURRENT_COAT_NUMBER', coat_num)
            rospy.set_param(
                'axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS', paint_proc)
            rospy.set_param(
                'axalta/ccscore/dashboard/SURFACE_MAPPING_DONE', True)
           
        if(paint_proc == "base" and coat_num == 1):
            t11.start()
            
        if(paint_proc == "base" and coat_num == 2):
            t12.start()
            
        if(paint_proc == "clear" and coat_num == 1):
            t13.start()
            
        if(paint_proc == "clear" and coat_num == 2):
            t14.start()
            
        if(paint_proc == "sealer" and coat_num == 1):
            t10.start()
        return True   
       # else:
       #     return False


    def initiate_first_coats(self, paint_proc):
        mir_resp = False
        if(paint_proc == "base" or paint_proc == "clear"):
            #mir_resp = self.move_to_destination_client(str(rospy.get_param('axalta/ccscore/dashboard/MIR_PAINTBOOTH_LOCATION')))
            mir_resp = True
        else:
            mir_resp=True
        if(paint_proc == "base"):
            rospy.set_param(
                'axalta/ccscore/dashboard/mir_home_to_pos_move_second', True)
        elif(paint_proc == "clear"):
            rospy.set_param(
                'axalta/ccscore/dashboard/mir_home_to_pos_move_clear', True)
        return mir_resp

    def exit_current_job(self):

        rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_DONE', True)
        rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_TRIGGER', True)
        rospy.set_param("axalta/ccscore/dashboard/LIDARSTART", True)
        t6 = threading.Thread(target=function_current_process_exit_current_job,args=(1,))
        t6.start()
        #status.current_process = "System reset for a new job is in progress.."
    
    def initiate_painting(self, paint_proc, coat_num):
        mir_resp=False
        if(coat_num == 1):
            mir_resp=self.initiate_first_coats(paint_proc)

        #if(paint_proc == "exit"):
         #   self.exit_current_job()
          #  return True

        set_status=self.surface_confirm_set_params(
            mir_resp, paint_proc, coat_num)
        
        return set_status

    def handle_surface_selected(self, req):
        try:
            print("------------------surface confirm----------------------")
            
            paint_proc=str(req.paint_process).lower()
            coat_num=req.coat_number
            timetaken=req.TimeSpan
            print("request :", paint_proc, coat_num)
            print("time taken : ", timetaken)
            global t5
            t5 = threading.Thread(target=function_current_process_surface_selected,args=(1,))
            
            global t10
            global t11
            global t12
            global t13
            global t14
            t10 = threading.Thread(target=function_current_process_surface_selected_Sealer,args=(1,))
            t11 = threading.Thread(target=function_current_process_surface_selected_Base_1,args=(1,))
            t12 = threading.Thread(target=function_current_process_surface_selected_Base_2,args=(1,))
            t13 = threading.Thread(target=function_current_process_surface_selected_Clear_1,args=(1,))
            t14 = threading.Thread(target=function_current_process_surface_selected_Clear_2,args=(1,))
            #t20 = threading.Thread(target=self.function_current_process_surface_mir_moving_to_charging,args=(1,))
           # if(paint_proc == "exit"):
            #    if not rospy.get_param("axalta/ccscore/dashboard/PaintingDone"):
            #        rospy.set_param("axalta/ccscore/dashboard/PaintingDone",True)
           # if(rospy.get_param("axalta/ccscore/dashboard/PaintingDone") and paint_proc == "exit"):
           #      t20.start()
                 
            surface_confirmation=self.initiate_painting(paint_proc, coat_num)
                 
            
            print("response: ", surface_confirmation)

            return SurfaceMappingConfirmResponse(surface_confirmation)

        except Exception as e:
            print("response: ", False)
            return SurfaceMappingConfirmResponse(False)   

def point_cloud_data_callback(msg):
    global points_
    print("CALL BACK IN CLOUD_CREATION")
    

    header = Header()
    header.frame_id = "camera_aligned_depth_to_color_frame_rec"
    header.stamp = rospy.Time.now()

    cloud=msg
    points_list = []
   
    
    pc2 = list(point_cloud2.read_points(cloud,field_names = ['x','y','z'], skip_nans=True))
    pc2_copy = pc2
   # print(pc2)
    print("-------------pointcloud read completed-------------------")   

    
    for data in pc2:
        x = data[0]
        y = data[1]
        z = data[2]   # - depth
       # rgb = float(data[3])
        points_list.append([x, y, z])
        #for item in points_list:
            #print(item)
        points_ = points_list
        #for i in points_:
             #print(i)
    
def sub_call():
    sub = rospy.Subscriber("/rgb_pointcloud", PointCloud2, point_cloud_data_callback)
    #rospy.spin()

if __name__ == "__main__":
    rospy.init_node('mock_application', anonymous=True)
    sub_call()
    Dashboard()
    UI_send()
    
