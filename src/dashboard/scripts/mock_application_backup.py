#!/usr/bin/env python

from __future__ import print_function
from std_msgs.msg import String
from status_check.msg import *
from status_check.srv import *

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



#Make sure all imported msg and srv files are available. Give mock application as a ROS2 package. If any function depends on external topic/service give hardcoded values to mock.
global status
status = Status_data() 
js = "I'm Here"

def UI_send():
    print("Inside UI send")
    global js
    #pub = rospy.Publisher('UI_data', String, queue_size=10)
    global stat_pub
    stat_pub = rospy.Publisher('core_status',Status_data,queue_size=10)
    
    joystick_pub = rospy.Publisher('joystick_command',String,queue_size=3)
    joystick_pub_act = rospy.Publisher('joystick_cmd',String,queue_size=3)
    progress_pub = rospy.Publisher('job_progress',Jobs,queue_size=3)
    img_pub = rospy.Publisher('map_mir', String,queue_size=2)
    
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
        status.arm_lock_flag = True
        status.emergency_flag = True
        status.completion_percentage = 60
        status.mir_battery = 60
        status.ccs_battery = 60
        status.temperature = "28"
        status.humidity = "80" 
        status.tape_color = "#008000"
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
    val.ObjectIdentificationDone = True
    val.SurfaceMappingDone = True  
    val.TrajectoryPlanningDone = True 
    val.PaintingDone = True
    val.RestartJobDone = True
    val.ExitJobDone = True
    val.ImageStitchingDone = True
    val.ImageSegmentationDone = True
    val.FeasibilityCheckDone = True 
    val.BaseCoat1.isFeasible = True            #  M    H
    val.BaseCoat2.isFeasible = True            #  O    E
    val.SealerCoat.isFeasible = True          #  C    R         #  K    E
    val.ClearCoat1.isFeasible = True           #   
    val.ClearCoat2.isFeasible = True
    val.BaseCoat1.isCompleted = True            #  M    H
    val.BaseCoat2.isCompleted = True            #  O    E
    val.SealerCoat.isCompleted = True          #  C    R         #  K    E
    val.ClearCoat1.isCompleted = True           #   
    val.ClearCoat2.isCompleted = True           #  DATA
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
    time.sleep(5)
    status.current_process = "Scanning is in progress.."
    stat_pub.publish(status)
    print(status.current_process)
    time.sleep(5)
    status.current_process = "Scanning has completed"
    print(status.current_process)
    stat_pub.publish(status)
    time.sleep(5)
    status.current_process = "Area identification is in progress.."
    print(status.current_process)
    stat_pub.publish(status)
    val.ImageStitchingDone = True
    progress_pub.publish(val)
    

class Dashboard():

    def __init__(self):
        #rospy.init_node('login_server')
        self.coat = 1
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
        s6 = rospy.Service('surface_mapping_confirm_server',
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
        rospy.set_param("axalta/ccscore/dashboard/TAPE_COLOR","#008000")

    # def report_callback(self,data):
    #    self.report = data

    def handle_segmented_image(self, req):
        try:
            print(
                "----------------Segmented image service------------------- ")
            print("request: ", req)
            print('time:', time.time())
            objectArea1 = SegmentedImageResponse()
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
            return objectArea1
        except Exception as e:
            print(e, "has occurred")
            
            
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

    def handle_job_details(self, req):
        obj_jobdetailshandler = JobDetailsHandler()
        print("Job_details_server is returning")
        return obj_jobdetailshandler.handle_job_details(req)

    
    
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

    '''def handle_surfaceMapping(self, req):
        surface_handle = SurfaceHandler()
        response = surface_handle.handle_surfaceMapping(req)
        return response
        return response
'''
    def handle_surface_selected(self, req):
        surface_handle = SurfaceHandler()
        response = surface_handle.handle_surface_selected(req)

    def handle_spray_gun(self, req):
        get_spray_settings = SpraygunHandler()
        settings = get_spray_settings.handle_spray_gun(req)
        return settings

    def handle_spray_gun_edit(self, req):
        set_spray_settings = SpraygunHandler()
        edited = set_spray_settings.handle_spray_gun_edit(req)
        return edited


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
            obj_jobdetails_handler = JobDetailsHandler()
            return obj_jobdetails_handler.move_to_destination_client("ChargingStation")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return MIRMoveToChargingResponse(False)

    def handle_start_segmentation(self, req):
        status = Status_data()
        rospy.loginfo("start Segmentation sevice called")
        print('-----------------------start_segmentation_server------------------------------------------------')
        print('response:', StartSegmentationResponse(True) )
        print('time:', time.time())
        status.current_process = "Area identification has completed"
        return StartSegmentationResponse(True)
    
    def handle_start_trajectory_planning(self, req):
        rospy.loginfo("start Trajectory sevice called")
        status = Status_data()
        val = Jobs()
        print('----------------start_trajectory_planning_server------------------------------------------------')
        val.TrajectoryPlanningDone == False
        status.current_process = "Trajectory planning is in progress.."
        val.TrajectoryPlanningDone == True
        time.sleep(3)
        if(val.TrajectoryPlanningDone == True):
              status.current_process = "Trajectory planning has completed"
        val.FeasibilityCheckDone = False
        time.sleep(2)
        status.current_process = "Feasibility check is in progress.."
        time.sleep(3)
        val.FeasibilityCheckDone = True
        if(val.FeasibilityCheckDone == True):
              status.current_process = "Feasibility check has completed" 
        print('response:', StartTrajectoryPlanningResponse(True) )
        print('time:', time.time())
        return StartTrajectoryPlanningResponse(True)   
    
    def handle_update_tape_color(self, req):
        try:
            status = Status_data()
            print(
                "----------------Update Tape color Service------------------- ")
            print("request: ", req)
            print('time:', time.time())
            if(req.isTemp):
                  rospy.set_param("axalta/ccscore/dashboard/TAPE_COLOR",req.tape_color)
                  status.tape_color = req.tape_color
                  tapeColor_is = req.tape_color
            else:
                  status.tape_color = req.tape_color
                  tapeColor_is = req.tape_color
            return UpdateTapeColorResponse(True)     
        except Exception as e:
            print(e, "has occurred")
    
class JobDetailsHandler():

    def __init__(self):
        self.jobID = '1'
        status = Status_data()

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
        time.sleep(2)
        status.current_process = "Robot is in motion.."
        status = self.set_robot_operating_mode(0)  # 1 = autonomous, 0= manual
        print("response :", status)
        return JobDetailsResponse(status)

    def move_to_target(self, destination):
        mir_resp = False
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
            status.current_process = "Robot is in motion.."
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
                print('response from fn:', True)
                return True
            else:
                print('response:', False)
                return False
                
        except Exception as e:
            print("exception message :", e)
            return False

    def move_to_destination_client(self, destination):
        #rospy.wait_for_service('move_to_destination')
        print("move_to_destination_client(), destination received :", destination)
        
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
            status.current_process = "New job has started"
            #  robot manual mode operation
            if(req.operating_mode.lower() == 'manual'):
                return self.set_robot_to_manualmode()

            #  robot  autonomous mode operations
            elif(req.operating_mode.lower() == 'autonomous'):
                destination = str(req.mir_paintbooth_location).lower().replace(" ","")
                status = self.robot_auto_functions(destination)
                #status = True
                print("response :", status)
                return JobDetailsResponse(status)
            return True
            

        except Exception as e:
            print(e, "has occurred")
            return JobDetailsResponse(False)
            
                        
class SpraygunHandler():

    def __init__(self):
        pass

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
        

class SurfaceHandler():
    def __init__(self):
        pass

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
        if(mir_resp or coat_num == 2):
            rospy.set_param(
                'axalta/ccscore/dashboard/GOTCONFIRMATION', True)
            rospy.set_param(
                'axalta/ccscore/dashboard/CURRENT_COAT_NUMBER', coat_num)
            rospy.set_param(
                'axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS', paint_proc)
            rospy.set_param(
                'axalta/ccscore/dashboard/SURFACE_MAPPING_DONE', True)
            return True
        else:
            return False

    def initiate_first_coats(self, paint_proc):
        mir_resp = False
        mir_move_job_details_handler = JobDetailsHandler()
        if(paint_proc == "base" or paint_proc == "clear"):
            mir_resp = mir_move_job_details_handler.move_to_destination_client(str(rospy.get_param('axalta/ccscore/dashboard/MIR_PAINTBOOTH_LOCATION')))
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
        rospy.set_param(
            'axalta/ccscore/dashboard/EXIT_JOB_TRIGGER', True)
        rospy.set_param("axalta/ccscore/dashboard/LIDARSTART", True)
        status.current_process = "System reset for a new job is in progress.."
    def initiate_painting(self, paint_proc, coat_num):
        mir_resp=False

        if(coat_num == 1):
            mir_resp=self.initiate_first_coats(paint_proc)

        if(paint_proc == "exit"):
            self.exit_current_job()
            return True

        set_status=self.surface_confirm_set_params(
            mir_resp, paint_proc, coat_num)
        return set_status

    def handle_surface_selected(self, req):
        try:
            print("------------------surface confirm----------------------")
            status = Status_data()
            status.current_process = "Painting process is in progress.."
            paint_proc=str(req.paint_process).lower()
            coat_num=req.coat_number
            timetaken=req.timespan
            print("request :", paint_proc, coat_num)
            print("time taken : ", timetaken)

            surface_confirmation=self.initiate_painting(paint_proc, coat_num)
            print("response: ", surface_confirmation)

            return SurfaceMappingConfirmResponse(surface_confirmation)

        except Exception as e:
            print(req)
            print("response: ", False)
            return SurfaceMappingConfirmResponse(False)   

if __name__ == "__main__":
    rospy.init_node('mock_application', anonymous=True)
    Dashboard()
    UI_send()
    
