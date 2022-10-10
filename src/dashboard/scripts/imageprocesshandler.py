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
import front_view_new
import ast


dir_name = os.path.dirname(os.path.abspath(__file__)) #will return /home/superadmin/axalta_ws/src/dashboard/scripts
par_of_script = os.path.normpath(os.path.join(dir_name, os.pardir))#will return /home/superadmin/axalta_ws/src/dashboard
par_of_dashbord = os.path.normpath(os.path.join(par_of_script, os.pardir))#will return /home/superadmin/axalta_ws/src
par_of_src = os.path.normpath(os.path.join(par_of_dashbord, os.pardir))#will return /home/superadmin/axalta_ws
par_of_ax_ws = os.path.normpath(os.path.join(par_of_src, os.pardir))#will return /home/superadmin

#file_path = os.path.join(par_of_ax_ws, 'Version2')#will return /home/superadmin/Version2

#import sys
#sys.path.append(file_path)


#import check_new_flow
#import Snapshot
#import Snapshot_ui
import ast
#from main_package.main_call import call_modules


#def function_current_process_start_Segmentation(name):
    # path_reconstruction = os.path.join(par_of_ax_ws, 'Version2/Data/')
    # check_new_flow.reduce_file_c_d(path_reconstruction)

    # upper_t = rospy.get_param("axalta/ccscore/dashboard/HSV_UPPER")
    # lower_t = rospy.get_param("axalta/ccscore/dashboard/HSV_LOWER")
    # res_upper = ast.literal_eval(upper_t)
    # res_lower = ast.literal_eval(lower_t)
    # valRange=12

    
    # # #path_tape_1 = "/home/superadmin/Version/Data/color"
    # path_Tape_tracking = os.path.join(par_of_ax_ws, 'Version2/Data/color')
    # flag_tapetracking = check_new_flow.GenSeg.Tape_Tracker(path_Tape_tracking,upper = res_upper,lower = res_lower,valRange = valRange)
    
    # #reconstruction
    # #path_reconstruction = os.path.join(par_of_ax_ws, 'Version2/Data/')
    # path_reconstruction_output = os.path.join(par_of_ax_ws, 'Version2/Data/output')
    # if not os.path.exists(path_reconstruction_output):
    #      os.mkdir(path_reconstruction_output)
         
    # flag_reconstruction = check_new_flow.GenSeg.reconstruction(path_reconstruction,path_reconstruction_output)
    # if(flag_reconstruction):
    #      rospy.set_param("axalta/ccscore/dashboard/IMAGE_STITCHING_DONE",True)
         
    # path_tape_seg = "/home/superadmin/Ve_/Data/segmentation/Image.png"""
    # #segmentation
    # path_reconstruction_output = os.path.join(par_of_ax_ws, 'Version2/Data/output')
    # path_reconstruction_output = os.path.join(par_of_ax_ws, 'Version2/Data/output')
    # if not os.path.exists(path_reconstruction_output):
    #      os.mkdir(path_reconstruction_output)
    # path_segmentation_input = os.path.join(path_reconstruction_output, 'Image.png')
    # path_config = os.path.join(par_of_ax_ws, 'Version2/config')

    
    # flag_segmenation =check_new_flow.GenSeg.Segmentation(path_segmentation_input,path_config,path_reconstruction_output)
    # if(flag_segmenation):
    #         rospy.set_param("axalta/ccscore/dashboard/IMAGE_SEGMENTATION_DONE",True)
    #         if(rospy.get_param("axalta/ccscore/dashboard/IMAGE_SEGMENTATION_DONE")):
    #            rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Area identification has completed")
    #            rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 100)
    # else:
    #      print("Segmenation Failed")
         
    # path_pointcloud_ply = os.path.join(par_of_ax_ws, 'Version2/Data/output/pointcloud.ply')
    # path_share_ply = os.path.join(par_of_src, 'share/pointcloud.ply')
    # shutil.copy(path_pointcloud_ply,path_share_ply)
    # st_ai = "call ai module"
    # res = call_modules(st_ai)
   # if(res):


def function_current_process_start_Segmentation(lab_value_string):
    #changing lab value from string to list(e.g : "[1,2,3]" --> [1,2,3])
    lab_value = ast.literal_eval(lab_value_string)

    rospy.set_param("WSL/Windows/LAB_SET",True)
    rospy.set_param("WSL/Windows/LAB",lab_value)

    cmd_image_seg = os.system("/home/axalta_ws/src/ai_module/sdk_launcher_segmentation.sh")   
    
    if(rospy.get_param("Windows/Segmentation_check")):
        
        if os.path.isfile("/home/axalta_ws/img/Segment_internal.png"):
            os.remove("/home/axalta_ws/img/Segment_internal.png")

        os.system("cp /mnt/c/Users/axalta/Desktop/shared/image/Segment_internal.png /home/axalta_ws/img/")
        os.rename("/home/axalta_ws/img/Segment_internal.png","/home/axalta_ws/img/SegmentedImage.png")

        if os.path.isfile("/home/axalta_ws/pointcloud/pointcloud.ply"):
            os.remove("/home/axalta_ws/pointcloud/pointcloud.ply")

        os.system("cp /mnt/c/Users/axalta/Desktop/shared/pointcloud/pointcloud.ply /home/axalta_ws/pointcloud/")
        os.system("cp /mnt/c/Users/axalta/Desktop/shared/pointcloud/crop_paint_output.ply /home/axalta_ws/pointcloud/")

        rospy.set_param("axalta/ccscore/dashboard/IMAGE_SEGMENTATION_DONE",True)
        rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Area identification has completed")
        rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 100)
        
    
def handle_segmented_image_img(req):
        try:
            rospy.loginfo("----------------Segmented image service------------------- ")
            print("request: ", req)
            print('time:', time.time())

            is_ManuallyCropped = req.isManuallyCropped
            objectArea1 = SegmentedImageResponse()
            
            if not is_ManuallyCropped:
                 path_segmented_image = "/home/axalta_ws/img/SegmentedImage.png"
                 rospy.loginfo("Path of image:")
                 rospy.loginfo(path_segmented_image)

                 with open(path_segmented_image, "rb") as img_file:
                     objarea_string = base64.b64encode(img_file.read())
                 objectArea1.ImageData = str(objarea_string)
              
            else:
                 rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Updating changes..")
                 rospy.set_param("axalta/ccscore/dashboard/IsManuallyCropped",True)
                
                 if os.path.isfile("/home/axalta_ws/pointcloud/pointcloud_manualsegmented.ply"):
                    os.remove("/home/axalta_ws/pointcloud/pointcloud_manualsegmented.ply")
                  
                 os.system("cp /mnt/c/Users/axalta/Desktop/shared/pointcloud/pointcloud_manualsegmented.ply /home/axalta_ws/pointcloud/")

                 path_cropped_image_ = "/home/axalta_ws/pointcloud/pointcloud_manualsegmented.ply"

                 os.system("cp /home/axalta_ws/pointcloud/pointcloud_manualsegmented.ply /home/axalta_ws/pointcloud/crop_paint_output.ply")
                 
                 front_view_new.Snapshot(path_cropped_image_)
     
                 path_cropped_image_path = "/home/axalta_ws/img/manual_cropped_front_view_image.png"

                 print("Path of image:")
                 print(path_cropped_image_path)
                    
                 with open(path_cropped_image_path, "rb") as img_file:
                           objarea_string = base64.b64encode(img_file.read())
                 objectArea1.ImageData = str(objarea_string)
            
            return objectArea1          
        except Exception as e:
            print(e, "has occurred")

def handle_stitched_image(req):
        try:
            rospy.loginfo("----------------Stitched image service------------------- ")
            print("request: ", req)
            print('time:', time.time())

            objectArea2 = StitchedImageResponse()
            
            path_img = "/home/axalta_ws/img/reconstructed_image.png"

            rospy.loginfo("Path of image:")
            rospy.loginfo(path_img)

            with open(path_img, "rb") as img_file:
                objarea_string = base64.b64encode(img_file.read())
            objectArea2.ImageData = str(objarea_string)
            
            return objectArea2
        except Exception as e:
            print(e, "has occurred")
    
    
def set_state(tapeColor_is):
          path_yaml = os.path.join(par_of_script, 'config/dashboard_persistent_settings.yaml')
          
          with open(path_yaml) as f:
                 doc = yaml.safe_load(f)
                 
          doc['axalta/ccscore/dashboard/TAPE_COLOR'] = tapeColor_is

          
          with open(path_yaml, 'w') as f:
                 yaml.safe_dump(doc, f, default_flow_style=False)
          return True
    
def handle_update_tape_color(req):
        try:
            rospy.loginfo("----------------Update Tape color Service------------------- ")
            print("request: ", req)
            print('time:', time.time())
            tapeColor_is = req.tape_color
            lab_value = req.LAB
            rospy.set_param("axalta/ccscore/dashboard/TAPE_COLOR",tapeColor_is)
            rospy.set_param("axalta/ccscore/dashboard/LAB_VALUE",lab_value)
            
            if not req.isTemp:
                  status = set_state(tapeColor_is)
                  if not status:
                       return False 

            print("response: ", True)
            return True          
                           
        except Exception as e:
            print(e, "has occurred")
            return False 


def handle_start_Segmentation(req):
    try:
        rospy.loginfo('-----------------------start_Segmentation_server------------------------------------------------')
        print('time:', time.time())
        rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Area identification is in progress..")
        rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 50)
        lab_value_str = rospy.get_param("axalta/ccscore/dashboard/LAB_VALUE")

        t2 = threading.Thread(target=function_current_process_start_Segmentation,args=(lab_value_str,))
        t2.start()
        
        print('response:', True)
        return True

    except Exception as e:
        print(e, "has occurred")
        return False

    