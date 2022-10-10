"""
Created on Tue June 29 14:31:24 2022

@author: 
    
    Frame reduction : Mithun Devanand
    
    Image stitching : Kira Yadav(20290909)
    
    Reconstruction : Pavan Nasireddy(2000120)
    
    Tape tracking : Kajal Asodekar(20270877)
    
    Segmentation : Shristi Srivastava(20293082), Kira Yadav(20290909)

    Integtration : Kalaivani Devaraj(1053418)
    
"""
#!/usr/bin/env python3
from __future__ import print_function

import rospy
import os
dir_name = os.path.dirname(os.path.abspath(__file__)) #will return /home/superadmin/axalta_ws/src/ai_module/scripts
par_of_script = os.path.normpath(os.path.join(dir_name, os.pardir))#will return /home/superadmin/axalta_ws/src/ai_module
par_of_dashbord = os.path.normpath(os.path.join(par_of_script, os.pardir))#will return /home/superadmin/axalta_ws/src
par_of_src = os.path.normpath(os.path.join(par_of_dashbord, os.pardir))#will return /home/superadmin/axalta_ws
par_of_ax_ws = os.path.normpath(os.path.join(par_of_src, os.pardir))#will return /home/superadmin

file_path = os.path.join(par_of_ax_ws, 'Version2')#will return /home/superadmin/Version2
import sys
sys.path.append(file_path)


from enum import Flag
from pkgutil import extend_path
#import tkinter
#from tkinter import filedialog

import json
import glob
from tracemalloc import take_snapshot
import TapeSegmentation
import Snapshot as sp
import Reiterate_segment_hsv as Segmentation
import cv2
import shutil
import numpy as np
from PIL import Image, ImageEnhance
import collections 
import PointsToCrop as ptc
import Cloud_clean as clcl
import ast

# class Error(Exception):
    # """Base class for other exceptions"""
    # pass
# 
# class DirectoryNotExistsError(Error):
    # """For directory not found"""
    # pass
# class FolderStructure(Error):
    # """Color,depth and camera_instrinsic"""
    # pass
# 

def t1():
    print("dd")

class GenSeg:
    # def __init__(self):
    #     if(os.path.exists("Reconstruction_output/output.ply")):
    #         os.remove('Reconstruction_output/output.ply')
    #     if(os.path.exists("Reconstruction_output/output.npz")):
    #         os.remove('Reconstruction_output/output.npz')
    #     if(os.path.exists("Reconstruction_output/output.log")):
    #         os.remove('Reconstruction_output/output.log')
    
    def Tape_Tracker(self,input_dir , upper , lower , valRange=12):

        path_data = os.path.normpath(os.path.join(input_dir, os.pardir))
        path_log = os.path.join(path_data, 'output')
        #sys.stdout = open(path_log + '/Ailog.txt','a')
        imgpath = input_dir
        print(input_dir)
        path_tape = os.path.dirname(input_dir)
        #print("path_tape",path_tape)
        #os.path(input_dir)
        #print(os.getcwd())
        #color_r_cmd1 = "rm -rf *[13579].jpg"
        #color_r_cmd1 = "rm -rf *[2468].jpg"
        flag = TapeSegmentation.track(imgpath, upper, lower, valRange)
        #result_dir = os.path.join('input','TrackedFrames')
        
        
        os.rename(input_dir,path_tape+ '/color_1')
        os.rename(path_tape+ '/TrackedFrames',path_tape+ '/color')
        #sys.stdout.close()
        return flag
    # def TakeSnap(self):
          
    #     sp.run()
    #     if(os.path.exists("Segmentation/Image.png")):
    #         print(True)
    #         curr = os.getcwd()
    #         return(os.path.join(curr,'Segmentation/Image.png')) 
    def reconstruction(self,input_dir , output_dir):
        #sys.stdout = open(output_dir + '/Ailog.txt','a')
        print(input_dir)
        print(os.listdir(input_dir))
        flag = True
        try:
        
            if(os.path.exists(input_dir) is not True):
                raise DirectoryNotExistsError
            else:
                try:
                    l_files_dir = os.listdir(input_dir)
                    r_file_dir = ['camera_intrinsic.json', 'color', 'depth' , 'TrackedFrames']
                    print(l_files_dir)
                    s1 = set(r_file_dir)
                    s2 = set(l_files_dir)
                    if s1.issubset(s2):
                          raise FileNotFoundError
                    print(l_files_dir)
                  #  if not all([item in l_files_dir for item in r_file_dir]):
                  #      raise FileNotFoundError
                except(FileNotFoundError): 
                    print("Reconstruction Folder structure is not maintained")
                    return 0
            print(os.listdir(input_dir))
                    #os.system(cmd)
            camera_instrinsic =  os.path.join(input_dir,'camera_intrinsic.json')
            dir_name_reconstruction = os.path.dirname(os.path.abspath(__file__))
            print(dir_name_reconstruction)
            #json_reconstruction = os.path.join(dir_name_reconstruction, 'Version2')
            with open(file_path+ "/ReconstructionSystem/config/realsense.json", "r+") as f:
                json_object = json. load(f)
                json_object["path_dataset"] = input_dir
                json_object["path_intrinsic"] = camera_instrinsic
                f.seek(0)        # <--- should reset file position to the beginning.
                json.dump(json_object, f, indent=4)
                f.truncate()     # remove remaining part
            cmd_reconstruction_set = "python3 "+ file_path +"/ReconstructionSystem/run_system.py " + file_path +  "/ReconstructionSystem/config/realsense.json --make --refine --register --integrate"
            #cmd = "python3 /home/superadmin/Version2/ReconstructionSystem/run_system.py /home/superadmin/Version2/ReconstructionSystem/config/realsense.json --make --refine --register --integrate"
            #cmd = cmd_reconstruction_set
            os.system(cmd_reconstruction_set)
            """if(os.path.exists(output_dir)):
                shutil.rmtree(output_dir)
            os.mkdir(output_dir)"""
            pointcloudpath = os.path.join(input_dir , 'scene/integrated.ply')
            if(os.path.exists(pointcloudpath) is not True):
                flag = False
                return flag, None
            else:
                shutil.copy(pointcloudpath , output_dir)
                cur = os.path.join(output_dir,'integrated.ply')
                res = os.path.join(output_dir , 'pointcloud.ply')
                os.rename(cur , res)
                sp.run(output_dir)
                #sys.stdout.close()
                return flag, output_dir
                
                
        except(DirectoryNotExistsError):
            print("Resconstruction Input Directory Not present")
        return flag , None
        
        #print(cmd)
    # def Segmentation(self , input_path , output_dir , minh, mins, minv, maxh, maxs, maxv):
    def Segmentation(self,input_path,path_config,output_dir):
        #im = Image.open(input_path)
        #enhancer = ImageEnhance.Brightness(im)
        #factor = 1.8
        #imoutput = enhancer.enhance(factor)
       # sys.stdout = open(output_dir + '/Ailog.txt','a')
        imoutput = cv2.imread(input_path)
        # cnts,index,Flag = Segmentation.check_contour_2((imoutput),output_dir, minh, mins, minv, maxh, maxs, maxv)  #linux then  pass np.array(imoutput)
        cnts,index,Flag = Segmentation.check_contour_2(np.array(imoutput),path_config,output_dir)
        Req_points = cnts[index]
        if Flag == False:
            self.Reconstruction()
            #TakeSnapSHot()
            Segmentation()
        else:
            Printing_File = open(output_dir + '/output.txt','w')
            for e_point in Req_points:
                Printing_File.write(str(e_point) + '\n')
            Printing_File.close()
            #print("----------------------Segmentation Completed--------------------------")
            # print("----------------------Points2Crop--------------------------")
            input_ply = output_dir + '/pointcloud.ply'
            ptc.points2crop(input_ply, output_dir)
            clcl.cleaning(output_dir)
            #print("------------------------------------------------------------")
    # def PtCrop():
    #     ptc.points2crop()
    #     clcl.cleaning()
        # if(os.path.exists(r'Segmentation/Segment_Img.png') and os.path.exists('Segmentation/crop.ply')):
        #     curr = os.getcwd()
        #     return([os.path.join(curr,'Segmentation/Segment_img.png') , os.path.join(curr , 'Segmentation/crop.ply')]) 
       # sys.stdout.close()
        return Flag
    
    def reduce_file_c_d(self,input_dir):
        color_dir_reduce = input_dir + "color/*[123456789].jpg"
        depth_dir_reduce = input_dir + "depth/*[123456789].png"
        #print(depth_dir_reduce)
    # print(input_dir)
    # print(color_dir_reduce)
    # depth_dir_reduce = input_dir + "depth"
        list_c_files = glob.glob(color_dir_reduce)
        list_d_files = glob.glob(depth_dir_reduce)
        #print(list_o_files)
        for f_c_p in list_c_files:
            os.remove(f_c_p)

        for f_d_p in list_d_files:
            os.remove(f_d_p)
        

    # cmd = "cd "+ input_dir
    # os.system(cmd)
    # cm2check = "mkdir foldernewAI"
    # os.system(cm2check)
    # print("directory changed")
    # cmd2 = "pwd"
    # os.system(cmd2)
    # print("completed")


    
# 
# if __name__ == "__main__":
    # rospy.init_node("ai_module_node")
    # low = [40,114,125]
    # high = [43,220,241]
    # val_range = 12
    # tape_tracker_input = '/home/superadmin/Version2/Data/color'
    # obj = GenSeg()
    # obj.reduce_file_c_d('/home/superadmin/Version2/Data/')
    # obj.Tape_Tracker(tape_tracker_input,high,low,val_range)
    # obj.reconstruction('/home/superadmin/Version2/Data','/home/superadmin/Version2/Data/output')
    # obj.Segmentation('/home/superadmin/Version2/Data/output/Image.png','/home/superadmin/Version2/config/config_segmentation.yml','/home/superadmin/Version2/Data/output')
   # reduce_file_c_d()
    # rospy.spin()
    





############Initialization###########################
#c = GenSeg()

#input_dir = "/home/superadmin/Ve_/Data/color"
#output_dir = 'tracked'
########################################################

#(Not Necessary) min H = 32, min S = 130, min V = 114; max H = 41, max S = 179, max V = 153

#############TapeTracking#######################
#flag  = c.Tape_Tracker(input_dir, [41,179, 153], [32, 130, 114],12 )
########################################################################

###########Reconstruction###################
# flag_res = c.reconstruction(input_dir , 'output')
####################################################

# print(cv2.imread(imp))


####Segmenatation#############
# imp = r"C:\Users\2000120\Desktop\Projects\Version Collection\V2.1\Data\Segmentation\Image.png"
# lower = 46,152,154
# upper = 61,277,255
# Flag = c.Segmentation(imp ,output_dir,upper,lower)
#####################################################
#print(flag)
#print(path)

        
