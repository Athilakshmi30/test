#!/usr/bin/env python3
from __future__ import print_function
import rospy
import shutil
from enum import Flag
from pkgutil import extend_path
#import tkinter
#from tkinter import filedialog
import os
from ai_module.srv import ImageProc,ImageProcResponse

import ast


dir_name = os.path.dirname(os.path.abspath(__file__)) #will return /home/superadmin/axalta_ws/src/ai_module/scripts
par_of_script = os.path.normpath(os.path.join(dir_name, os.pardir))#will return /home/superadmin/axalta_ws/src/ai_module
par_of_dashbord = os.path.normpath(os.path.join(par_of_script, os.pardir))#will return /home/superadmin/axalta_ws/src
par_of_src = os.path.normpath(os.path.join(par_of_dashbord, os.pardir))#will return /home/superadmin/axalta_ws
par_of_ax_ws = os.path.normpath(os.path.join(par_of_src, os.pardir))#will return /home/superadmin

file_path = os.path.join(par_of_ax_ws, 'Version2')#will return /home/superadmin/Version2
import sys
sys.path.append(dir_name)
import ai_flow
#print(from ai_flow import *)

def handle_image_processing(req):
    try:
        
        obj_ai = ai_flow.GenSeg()
        print("inside ai module server")
        path_reconstruction = os.path.join(par_of_ax_ws, 'Version2/Data/')
        print("inside ai module server2")
        obj_ai.reduce_file_c_d(path_reconstruction)
        print("inside ai module server3")
        upper_t = rospy.get_param("axalta/ccscore/dashboard/HSV_UPPER")
        print("inside ai module server4")
        lower_t = rospy.get_param("axalta/ccscore/dashboard/HSV_LOWER")
        print("inside ai module server4")
        res_upper = ast.literal_eval(upper_t)
        print(res_upper)
        res_lower = ast.literal_eval(lower_t)
        valRange=12

        print("inside ai module server6")
        # #path_tape_1 = "/home/superadmin/Version/Data/color"
        path_Tape_tracking = os.path.join(par_of_ax_ws, 'Version2/Data/color')
        flag_tapetracking = obj_ai.Tape_Tracker(path_Tape_tracking,upper = res_upper,lower = res_lower,valRange = valRange)
        print("inside ai module server7")
        #reconstruction
        #path_reconstruction = os.path.join(par_of_ax_ws, 'Version2/Data/')
        path_reconstruction_output = os.path.join(par_of_ax_ws, 'Version2/Data/output')
        if not os.path.exists(path_reconstruction_output):
            os.mkdir(path_reconstruction_output)
            
        flag_reconstruction = obj_ai.reconstruction(path_reconstruction,path_reconstruction_output)
        if(flag_reconstruction):
            rospy.set_param("axalta/ccscore/dashboard/IMAGE_STITCHING_DONE",True)
            
        #path_tape_seg = "/home/superadmin/Ve_/Data/segmentation/Image.png"""
        #segmentation

        path_segmentation_input = os.path.join(path_reconstruction_output, 'Image.png')
        path_config = os.path.join(par_of_ax_ws, 'Version2/config')

        
        flag_segmenation =obj_ai.Segmentation(path_segmentation_input,path_config,path_reconstruction_output)
        if(flag_segmenation):
                rospy.set_param("axalta/ccscore/dashboard/IMAGE_SEGMENTATION_DONE",True)
                rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Area identification has completed")
                rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 100)
        else:
            print("Segmenation Failed")
            
        path_pointcloud_ply = os.path.join(par_of_ax_ws, 'Version2/Data/output/pointcloud.ply')
        path_share_ply = os.path.join(par_of_src, 'share/pointcloud.ply')
        shutil.copy(path_pointcloud_ply,path_share_ply)

        if(flag_tapetracking and flag_reconstruction and flag_segmenation):
            stat = "process completed"
            print(stat)
            return ImageProcResponse(stat)
        else:
            stat = "process failed"
            print(stat)
            return ImageProcResponse(stat)

    except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            stat = "process failed"
            return ImageProcResponse(stat)

    

def ai_module_server():
    rospy.init_node("Image_processing_server_node")
    ai_service = rospy.Service("Start_imageProcessing_server",ImageProc,handle_image_processing)
    


if __name__ == "__main__":
    ai_module_server()
    rospy.spin()

        
