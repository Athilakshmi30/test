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

def handle_image_processing(req):
    try:
        cmd_image_rec = os.system("/home/axalta_ws/src/ai_module/sdk_launcher.sh")
        cmd_image_copy = os.system("cp /mnt/c/Users/axalta/Desktop/Pavan_axalta/Version3_Dev/Output/Image.png /home/axalta_ws/img/")
        print("im inside start imageprocessing server")
       # stat = True
        print(True)
        return True
    
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        #stat = False
        return False


def ai_module_server():
    rospy.init_node("Image_processing_server_node")
    ai_service = rospy.Service("Start_imageProcessing_server",ImageProc,handle_image_processing)
    


if __name__ == "__main__":
    ai_module_server()
    rospy.spin()