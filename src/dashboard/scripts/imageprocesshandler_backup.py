#!/usr/bin/env python3
from __future__ import print_function
from __future__ import absolute_import

import rospy
import time
import base64
import yaml
import threading
import os
from pathlib import Path
import shutil
from dashboard.srv import *
from dashboard.msg import *
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt


dir_name = os.path.dirname(os.path.abspath(__file__)) #will return /home/superadmin/axalta_ws/src/dashboard/scripts
par_of_script = os.path.normpath(os.path.join(dir_name, os.pardir))#will return /home/superadmin/axalta_ws/src/dashboard
par_of_dashbord = os.path.normpath(os.path.join(par_of_script, os.pardir))#will return /home/superadmin/axalta_ws/src
par_of_src = os.path.normpath(os.path.join(par_of_dashbord, os.pardir))#will return /home/superadmin/axalta_ws
par_of_ax_ws = os.path.normpath(os.path.join(par_of_src, os.pardir))#will return /home/superadmin

file_path = os.path.join(par_of_ax_ws, 'Version2')#will return /home/superadmin/Version2

import sys
sys.path.append(file_path)


import check_new_flow
import Snapshot
import ast
#from main_package.main_call import call_modules"

# def function_current_process_start_stitching_(name):
#     path_reconstruction = os.path.join(par_of_ax_ws, 'Version2/Data/')
#     check_new_flow.reduce_file_c_d(path_reconstruction)

#     upper_t = rospy.get_param("axalta/ccscore/dashboard/HSV_UPPER")
#     lower_t = rospy.get_param("axalta/ccscore/dashboard/HSV_LOWER")
#     res_upper = ast.literal_eval(upper_t)
#     res_lower = ast.literal_eval(lower_t)
#     valRange=12

    
#     # #path_tape_1 = "/home/superadmin/Version/Data/color"
#     path_Tape_tracking = os.path.join(par_of_ax_ws, 'Version2/Data/color')
#     flag_tapetracking = check_new_flow.GenSeg.Tape_Tracker(path_Tape_tracking,upper = res_upper,lower = res_lower,valRange = valRange)
    
#     #reconstruction
#     #path_reconstruction = os.path.join(par_of_ax_ws, 'Version2/Data/')
#     path_reconstruction_output = os.path.join(par_of_ax_ws, 'Version2/Data/output')
#     if not os.path.exists(path_reconstruction_output):
#          os.mkdir(path_reconstruction_output)
         
#     flag_reconstruction = check_new_flow.GenSeg.reconstruction(path_reconstruction,path_reconstruction_output)
#     if(flag_reconstruction):
#          rospy.set_param("axalta/ccscore/dashboard/IMAGE_STITCHING_DONE",True)
         
#     path_tape_seg = "/home/superadmin/Ve_/Data/segmentation/Image.png"""
#     #segmentation
#     path_reconstruction_output = os.path.join(par_of_ax_ws, 'Version2/Data/output')
#     path_reconstruction_output = os.path.join(par_of_ax_ws, 'Version2/Data/output')
#     if not os.path.exists(path_reconstruction_output):
#          os.mkdir(path_reconstruction_output)
#     path_segmentation_input = os.path.join(path_reconstruction_output, 'Image.png')
#     path_config = os.path.join(par_of_ax_ws, 'Version2/config')

    
#     flag_segmenation =check_new_flow.GenSeg.Segmentation(path_segmentation_input,path_config,path_reconstruction_output)
#     if(flag_segmenation):
#             rospy.set_param("axalta/ccscore/dashboard/IMAGE_SEGMENTATION_DONE",True)
#             if(rospy.get_param("axalta/ccscore/dashboard/IMAGE_SEGMENTATION_DONE")):
#                rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Area identification has completed")
#                rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 100)
#     else:
#          print("Segmenation Failed")
         
#     path_pointcloud_ply = os.path.join(par_of_ax_ws, 'Version2/Data/output/pointcloud.ply')
#     path_share_ply = os.path.join(par_of_src, 'share/pointcloud.ply')
#     shutil.copy(path_pointcloud_ply,path_share_ply)
#     #call_modules("call ai module")
    

    
    
def handle_segmented_image(req):
        try:
            rospy.loginfo(
                "----------------Segmented image service------------------- ")
            print("request: ", req)
            print('time:', time.time())
            is_ManuallyCropped = req.isManuallyCropped
            objectArea1 = SegmentedImageResponse()
            
            if not is_ManuallyCropped:
                 path_segmented_image = os.path.join(par_of_ax_ws, 'Version2/Data/output/segment.png')
                 print("Path of image:")
                 print(path_segmented_image)
                 with open(path_segmented_image, "rb") as img_file:
                     objarea_string = base64.b64encode(img_file.read())
                 objectArea1.ImageData = str(objarea_string, 'UTF-8')
              
            else:
                 rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Updating changes..")
                 rospy.set_param("axalta/ccscore/dashboard/IsManuallyCropped",True)
                 #res = convert_2d()
                 path_cropped_image_ = os.path.join(par_of_src, 'share')
                 Snapshot.run(path_cropped_image_)
                   
                 path_cropped_image = os.path.join(path_cropped_image_, 'Image.png')
                 print("Path of image:")
                 print(path_cropped_image)
                    
                 with open(path_cropped_image, "rb") as img_file:
                           objarea_string = base64.b64encode(img_file.read())
                 objectArea1.ImageData = str(objarea_string, 'UTF-8')
                 
            return objectArea1          
        except Exception as e:
            print(e, "has occurred")

def handle_stitched_image(req):
        try:
            rospy.loginfo(
                "----------------Stitched image service------------------- ")
            print("request: ", req)
            print('time:', time.time())
            objectArea2 = StitchedImageResponse()
                 
            path_img = os.path.join(par_of_ax_ws, 'Version2/Data/output/Image.png')
            
            print("Path of image:")
            print(path_img)
            with open(path_img, "rb") as img_file:
                objarea_string = base64.b64encode(img_file.read())
            objectArea2.ImageData = str(objarea_string, 'UTF-8')
            
            return objectArea2
        except Exception as e:
            print(e, "has occurred")
    
    
def set_state(tapeColor_is,hsvlower_is,hsvupper_is):
          path_yaml = os.path.join(par_of_script, 'config/dashboard_persistent_settings.yaml')
          
          with open(path_yaml) as f:
                 doc = yaml.safe_load(f)
                 
          doc['axalta/ccscore/dashboard/TAPE_COLOR'] = tapeColor_is
          doc['axalta/ccscore/dashboard/HSV_LOWER'] = hsvlower_is
          doc['axalta/ccscore/dashboard/HSV_UPPER'] = hsvupper_is
          
          with open(path_yaml, 'w') as f:
                 yaml.safe_dump(doc, f, default_flow_style=False)
          return True
    
def handle_update_tape_color(req):
        try:
            rospy.loginfo(
                "----------------Update Tape color Service------------------- ")
            print("request: ", req)
            print('time:', time.time())
            tapeColor_is = req.tape_color
            hsvlower_is = req.hsvLower
            hsvupper_is = req.hsvUpper
            rospy.set_param("axalta/ccscore/dashboard/TAPE_COLOR",tapeColor_is)
            rospy.set_param("aaxalta/ccscore/dashboard/HSV_LOWER",hsvlower_is)
            rospy.set_param("axalta/ccscore/dashboard/HSV_UPPER",hsvupper_is)
            
            if not req.isTemp:
                  status = set_state(tapeColor_is,hsvlower_is,hsvupper_is)
                  if not status:
                       return False 

            print("response: ", True)
            return True          
                           
        except Exception as e:
            print(e, "has occurred")
    
    
    
"""def handle_start_Segmentation(self,req):
        rospy.loginfo("start segmentation sevice called")
        print('-----------------------start_Segmentation_server------------------------------------------------')
        t4 = threading.Thread(target=function_current_process_segmentation,args=(1,))
        t4.start()
        
        return StartSegmentationResponse(True)"""
    
    
def handle_start_Stitching(req):
        rospy.loginfo("start Stitching sevice called")
        print('-----------------------start_Stitching_server------------------------------------------------')
        print('time:', time.time())

        t2 = threading.Thread(target=function_current_process_start_stitching_,args=(1,))
        t2.start()
        
        print('response:', StartStitchingResponse(True))
        return StartStitchingResponse(True)
