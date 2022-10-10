#!/usr/bin/env python
import rospy
import numpy as np
from PIL import Image
import cv2 
from sensor_msgs.msg import PointCloud2, PointField 
from sensor_msgs import point_cloud2 as pc2
import struct
import ctypes
import statistics as st
from lidar_ouster.srv import *
from ouster_ros.srv import *
import numpy as np
import ouster_ros as os
import json
import itertools as it
import math

from std_msgs.msg import Header

from itertools import compress


class CloudGenerate():

    def __init__(self):
        self.flag = 0
        self.points = []
        #self.axes_shifted = []
        self.axes = []
        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('rgb',12,PointField.UINT32,1)]
        rospy.Subscriber("reconstructed_points", PointCloud2, self.on_new_cloud)    
        self.stop = True
        self.msg_count = 0
        
    def on_new_cloud(self,msg):
        #ctrlflg = rospy.get_param('axalta/ccscore/dashboard/PAINTJOBPROCESS')
        
        #if(ctrlflg and self.stop):
        if(self.stop):
            rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Area Identification in progress....")
            print ('got something!!')
            print(msg.fields)
            bgr,abc = self.cloud2bgr(msg)
            
            self.points = bgr
            
            self.axes = abc
            self.flag = 1
   
            self.stop = False
            rospy.set_param("axalta/ccscore/dashboard/LIDAR_OFF_TRIGGER",True)
            rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE",65) 

    def bgr_gen(self,seq_for_use):
        maxy = -1.0
        miny = 10.0
        maxx = -1.0
        minx = 10.0
        minz = 10.0
        maxz = -1.0 
        tot = 0 
        points = []
        axes = []
 
        l1 = []
        
        new_l = []
        l = list(seq_for_use)

        maxinten = -1.0

        
        
        print("input length",len(l))
       
        """l_maxz = max(l, key=lambda x: x[2])[2]
        l_minz = min(l, key=lambda x: x[2])[2] 
        l_maxy = max(l, key=lambda x: x[1])[1]
        l_miny = min(l, key=lambda x: x[1])[1]
        l_maxx = max(l, key=lambda x: x[0])[0]
        l_minx = min(l, key=lambda x: x[0])[0]"""  
 
        mn = []
        for pt in l:
            x = pt[0]
            y = pt[1]
            z = pt[2]
            if(maxy<y):
                maxy = y
            if(miny>y):
                miny = y 
            if(maxx<x):
                maxx = x
            if(minx>x):
                minx = x
            if(maxz<z):
                maxz = z
            if(minz>z):
                minz = z        
                 
             
            tot = tot+1 
            single_point = [x, y, z,pt[3]]
            #print(single_point)
            mn.append(single_point)
            
                    
        
        print(minx,maxx,miny,maxy,minz,maxz)
 
        full_cloud=[]
        prev_yval = 0
        yval = miny
        ybuf = 0.01  
        first = True
  
        no_of_cols = int(math.ceil((maxx - minx)/0.01))
        no_of_rows = int(math.ceil((maxy - miny)/0.01))
        print("no_of_rows",no_of_rows) 
        print("no_of_cols",no_of_cols)  
        while(yval<maxy):
            current_row = [] 
            for pt in mn:
                if(first and pt[1]<=yval+ybuf):
                    current_row.append(pt)
                    first = False 
                elif(pt[1]>yval and pt[1]<=yval+ybuf):
                    current_row.append(pt)
                #print(len(current_row))
              
            yval = yval+ybuf
            current_row.sort(key = lambda x : x[0])
            #print(len(current_row))
            full_cloud.append(current_row)          
        print("full no of rows",len(full_cloud))            
        
        yval = miny
        prev = True
        structured_cloud = [[[0.0,0.0,0.0,0]]*no_of_cols]*no_of_rows
        #for i in range(no_of_rows):
        #    for j in range(no_of_cols):
        for row in full_cloud:
            val = minx
            xval = minx
            xbuf = 0.01
            for pt in row:
                m = int((pt[1]-miny)/ybuf)
                n = int((pt[0]-minx)/xbuf)
                if((abs(pt[1]-yval)<0.005) and (abs(pt[0]-xval)<0.005) and (structured_cloud[m][n] == [0.0,0.0,0.0,0])):    
                    structured_cloud[m][n] = pt 
                xval = xval + xbuf 
            yval = yval + ybuf
            
            
        BIT_MOVE_16 = 2**16
        BIT_MOVE_8 = 2**8
        #print(full_cld)
        print("rows in structured cloud",len(structured_cloud))
                            
        img = []   
        for row in structured_cloud:
            img_row = []  
            for pt in row:
                img_row.append((pt[3] & 0b000000000000000011111111 ,(pt[3] & 0b000000001111111100000000)>>8 , (pt[3]& 0b111111110000000000000000)>>16))
                #print(pt[3]&0b000000001111)
               
            
            img.append(img_row)

        for m in img:
            assert len(m)==no_of_cols    
        """pixels = [
            [(54, 54, 54), (232, 23, 93), (71, 71, 71), (168, 167, 167)],
            [(204, 82, 122), (54, 54, 54), (168, 167, 167), (232, 23, 93)],
            [(71, 71, 71), (168, 167, 167), (54, 54, 54), (204, 82, 122)],
            [(168, 167, 167), (204, 82, 122), (232, 23, 93), (54, 54, 54)]
            ]

        # Convert the pixels into an array using numpy
        array = np.array(pixels, dtype=np.uint8)

        # Use PIL to create an image from the new array of pixels
        new_image = Image.fromarray(array)
        new_image.save('new.png') """
        imgarr = np.array(img,dtype=np.uint8)
        new_image = Image.fromarray(imgarr)
        new_image.save('new.png') 
        #print(img)         
        # xpts = np.random.uniform(minx-0.25,maxx,len(points)).tolist()
        
        # mn = points
        # border = points 
        # points = []
        # c = 0
        # for val in mn:
            
        #     if(val[0]>1.3):
        #         points.append([1.3,val[1],val[2]])
        #     else:    
        #         points.append(val)
            
        #     val[0] = xpts[c]
            
        #     if(val[0]>1.3):
        #         points.append([1.3,val[1],val[2]])
        #     else:    
        #         points.append(val)
            
        #     c = c+1
      


        
        # print("length of points",len(points)) 
        # print("input length",len(l))  
        
        # bool_list = self.in_hull(np.array(new_l),np.array(points))
        
      
        
        # axes = list(compress(new_l,bool_list))
        # axes_maxy = max(axes, key=lambda x: x[1])[1]
        # axes_miny = min(axes, key=lambda x: x[1])[1]
        # axes_maxz = max(axes, key=lambda x: x[2])[2]
        # axes_minz = min(axes, key=lambda x: x[2])[2]

        # new_axes = axes
        # for point_border in border:
        #     for point_xyz in axes:
        #         if(abs(point_border[1]-point_xyz[1])<=0.0001):
        #             new_axes.remove(point_xyz)
                        
        # rospy.set_param('axalta/ccscore/paintjobprocess/MINY',axes_miny)
        # rospy.set_param('axalta/ccscore/paintjobprocess/MAXY',axes_maxy)
        # rospy.set_param('axalta/ccscore/paintjobprocess/MAXZ',axes_maxz)
        # rospy.set_param('axalta/ccscore/paintjobprocess/MINZ',axes_minz)
        
        # print("removed new value")
        # new_axes_maxz = max(new_axes, key=lambda x: x[2])[2]
        # new_axes_minz = min(new_axes, key=lambda x: x[2])[2] 
        # new_axes_maxy = max(new_axes, key=lambda x: x[1])[1]
        # new_axes_miny = min(new_axes, key=lambda x: x[1])[1]
        # new_axes_maxx = max(new_axes, key=lambda x: x[0])[0]
        # new_axes_minx = min(new_axes, key=lambda x: x[0])[0]  
        # print(new_axes_maxz,new_axes_minz,new_axes_maxy,new_axes_miny)
        
        # zwise = new_axes_maxz  
        # new_points = [] 
        # new_line = []
        # set_point = new_axes[0][2]  
        # print("set_points",set_point) 
        # rownum = 0
        # for it in range(len(new_axes)-1):
                
                 
        #         if(abs(set_point-new_axes[it][2]) < 0.01):
                                        
        #             new_line.append([new_axes[it][0],new_axes[it][1],new_axes[it][2]])  
        #         else:
        #             set_point = new_axes[it][2]
                    
        #             if(len(new_line)>0):
        #                 new_points.append(new_line)
        #                 rownum = rownum+1  
        #                 new_line = []
                      
                    
                        
        # print("rownum",rownum)            
        # y_new = (np.arange(axes_miny, axes_maxy, 0.005)).tolist() 
        # polyfit_points = []
        # #print("new_points",new_points)
        # xtake = new_points 
        # prev = [pt[0] for row in xtake for pt in row if (pt[2]>-0.001 and pt[2]<0.001)]
       
        return points,axes
     

    def cloud2bgr(self,cloud):
        
        generator_trav = pc2.read_points(cloud, field_names = ['x','y','z','rgb'], skip_nans = True)
        #print(len(generator_trav))
        #points,axes,axes_shifted = self.bgr_gen(generator_trav)
        points,axes = self.bgr_gen(generator_trav)
        #return points,axes,axes_shifted
        return points,axes

        
if __name__ == '__main__':
    try:
            rospy.init_node('create_structured_cloud_xyzrgb', anonymous=True)
        #while(not rospy.has_param('axalta/ccscore/flow_seperator/FLOW')):
        #    continue 
        #if(rospy.get_param('axalta/ccscore/flow_seperator/FLOW') == "Scenario2"):
            print("process starting...")
            obj = CloudGenerate()
            header = Header()
            header.frame_id = "camera_aligned_depth_to_color_frame"
            rate = rospy.Rate(2)
            pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)
            
            pub2 = rospy.Publisher("axes_points", PointCloud2, queue_size=2)
            #pub3 = rospy.Publisher("axes_shifted_points", PointCloud2, queue_size=2)
            
            while not rospy.is_shutdown():
                if(rospy.has_param("axalta/ccscore/dashboard/RESTART_AREAIDNODE_TRIGGER") and rospy.get_param("axalta/ccscore/dashboard/RESTART_AREAIDNODE_TRIGGER")):
                    obj.flag=0 
                    obj.points = []
                    obj.axes = []
                    obj.stop = True
                    obj.msg_count = 0
                    rospy.set_param("axalta/ccscore/dashboard/RESTART_FILTERNODE_TRIGGER",True)  
                    rospy.set_param("axalta/ccscore/dashboard/RESTART_AREAIDNODE_TRIGGER",False)  
                if(obj.flag):
                    pcld2 = pc2.create_cloud(header, obj.fields, obj.points)
                    pcld2.header.stamp = rospy.Time.now()
                    pub.publish(pcld2)
                    
                    npcld3 = pc2.create_cloud(header, obj.fields, obj.axes)
                    npcld3.header.stamp = rospy.Time.now()
                    pub2.publish(npcld3)

                    #npcld4 = pc2.create_cloud(header, obj.fields, obj.axes_shifted)
                    #npcld4.header.stamp = rospy.Time.now()
                    #pub3.publish(npcld4)
                    #obj.flag=0
                rate.sleep()
            rospy.spin()

    except rospy.ROSInterruptException:
        pass


