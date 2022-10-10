#!/usr/bin/env python
import rospy
import numpy as np
from PIL import Image
import cv2 
from sensor_msgs.msg import PointCloud2, PointField 
from sensor_msgs import point_cloud2 as pc2

from lidar_ouster.srv import *
from ouster_ros.srv import *

import itertools as it
import math
import time
from std_msgs.msg import Header



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
       
        """l_maxz = max(l, key=lambda x: x[2])[2]"""

 

        mn = []
        twoDprojected_points = []
        for pt in l:
            x = pt[0]
            y = pt[1]
            z = pt[2]
            
            if(abs(x) < 1.7 and abs(y) < 1.7 and abs(z) < 1.7):
                tot = tot + 1 
                single_point = [round(x,6), round(y,6), round(z,6),pt[3]]
                #print(single_point)
                mn.append(single_point)
                twoDprojected_points.append([x,y,pt[3]])
   
        for pt in mn:
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
                 
             
            
        print(tot)
        print(minx,maxx,miny,maxy,minz,maxz)

        ybuf = 0.0005  
        xbuf = 0.0005
        no_of_cols = int(math.ceil((maxx - minx)/0.0005))
        no_of_rows = int(math.ceil((maxy - miny)/0.0005))
        print("no_of_rows",no_of_rows) 
        print("no_of_cols",no_of_cols)
 
        cloud_full = []
        start = time.time()
        arr = np.arange(miny,maxy,ybuf)
        arr = arr.tolist()
        #print(arr)
        for yval in arr:
            current_row = []
            for pt in mn:
                #if(abs(pt[1]-yval) < 0.001 and abs(pt[0]-xval) < 0.001):
                if(abs(pt[1]-yval) <= 0.001):
                    current_row.append(pt)
                #xval = xval + xbuf    
            #current_row.sort(key = lambda x : x[0])             
            cloud_full.append(current_row)
        end = time.time()

        print("time_taken y slicing : ",end - start)    

        #print(cloud_full)
         
        structured_cloud = [[[0.0,0.0,0.0,0]]*no_of_cols]*no_of_rows   
        #print(structured_cloud)

        for row in cloud_full:
            #val = minx
            xval = minx
       
            for pt in row:
                m = int((pt[1]-miny)/ybuf)
                n = int((pt[0]-minx)/xbuf)
                print("m ",m,"n ",n)
                if((abs(pt[1]-yval)<0.008) and (abs(pt[0]-xval)<0.008) and (structured_cloud[m][n] == [0.0,0.0,0.0,0])):    
                    structured_cloud[m][n] = pt 
                xval = xval + xbuf 
            yval = yval + ybuf
        
        """start = time.time()
        inter_cloud = []  
        for row in cloud_full:
            if(len(row)>0):
                xval = row[0][0]
            else:
                continue    
            row_points = []
            for pt in row:
                if(abs(pt[0]-xval) <= 0.01):    
                    row_points.append(pt)
                xval = xval + xbuf    
            inter_cloud.append(row_points)
        end = time.time()
        print("time taken x slicing : ",end-start)"""
        
        """lenval = 0
        for row in cloud_full:
            for pt in row:
                lenval = lenval + 1

        print("length after y slicing :" , lenval)"""

        """start = time.time()
        row_val = 0
     
        structured_cloud = cloud_full
        for row in cloud_full:
            
            if(len(row) == 0):
                structured_cloud[row_val] = [[0.0,0.0,0.0,0]]*no_of_cols
                row_val = row_val + 1
                if(row_val >= no_of_rows):
                    break
                continue
            
            xval = minx
            xbuf = 0.005
            
            while(xval<row[0][0]):
                xval = xval + xbuf
                structured_cloud[row_val].insert(0,[0.0,0.0,0.0,0])
            if(structured_cloud[row_val] > no_of_cols):
                val = 0
                while(structured_cloud[row_val][val] == [0.0,0.0,0.0,0]):
                    val = val + 1
                start = structured_cloud[row_val][val][0]
                operating_row = structured_cloud[row_val]
                while(val<len(structured_cloud[row_val])):
                    check = abs(start - operating_row[val][0])
                    if(check > 0 and check < 0.005):
                        del operating_row[val] 
                    else:
                        start = operating_row[val][0]
                    val = val + 1

                structured_cloud[row_val] = operating_row              
            while(len(structured_cloud[row_val])<no_of_cols):
                structured_cloud[row_val].append([0.0,0.0,0.0,0])
             
            row_val = row_val + 1

        end = time.time()
        #print(structured_cloud[0])
        #print(len(structured_cloud))
        #print(cloud_full[0])
        #print(structured_cloud)
       

        print("time_taken structured cloud: ",(end - start))
        lenval = 0
        for row in structured_cloud:
            for pt in row:
                lenval = lenval + 1

        print("length of processed cloud",lenval)
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
            assert len(m)==no_of_cols""" 
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
        new_image.save('new.png')""" 
        """imgarr = np.array(img,dtype=np.uint8)
        new_image = Image.fromarray(imgarr)
        new_image.save('new.png') """
       
        points = [item for sublist in structured_cloud for item in sublist]
        #points = [item for sublist in cloud_full for item in sublist]
        #points = [item for sublist in inter_cloud for item in sublist]
        return points,axes
        #return mn,axes
     

    def cloud2bgr(self,cloud):
        
        generator_trav = pc2.read_points(cloud, field_names = ['x','y','z','rgb'], skip_nans = True)
      
        points,axes = self.bgr_gen(generator_trav)
        print("processed")
       
        return points,axes

        
if __name__ == '__main__':
    try:
        rospy.init_node('create_structured_cloud_xyzrgb', anonymous=True)
    
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


