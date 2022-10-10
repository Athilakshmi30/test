#!/usr/bin/env python
import rospy
import numpy as np
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
import matplotlib.pyplot as plt
import math
from std_msgs.msg import Header
import struct
from itertools import compress
from scipy.spatial import Delaunay
from numpy.polynomial import polynomial as P
import scipy.optimize as sc

class AreaSelect():

    def __init__(self):
        self.flag = 0
        self.points = []
        
        self.axes = []
        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                ]
        rospy.Subscriber("lidar_data", PointCloud2, self.on_new_cloud)    
        self.stop = True
        self.msg_count = 0
        
    def on_new_cloud(self,msg):
        ctrlflg = rospy.get_param('axalta/ccscore/dashboard/PAINTJOBPROCESS')
        
        if(ctrlflg and self.stop):
            rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Area Identification in progress....")
            print 'got something!!'
            
            bgr,abc = self.cloud2bgr(msg)
            
            self.points = bgr
            
            self.axes = abc
            self.flag = 1
   
            self.stop = False
            rospy.set_param("axalta/ccscore/dashboard/LIDAR_OFF_TRIGGER",True)
            rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE",65) 

    def percentile_based_outlier(self,data, threshold=99):

        diff = (100 - threshold) / 2.0
        minval, maxval = np.percentile(data, [diff, 100 - diff])
        print(minval,maxval)
        return minval,maxval

    def percentile_based_outlier_max(self,data, threshold=85):
        minval, maxval = self.percentile_based_outlier(data, threshold) 
        print(minval,maxval)   
        return (data > maxval)

    def in_hull(self,p, hull):
        """
        Test if points in `p` are in `hull`

        `p` should be a `NxK` coordinates of `N` points in `K` dimensions
        `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the 
        coordinates of `M` points in `K`dimensions for which Delaunay triangulation
        will be computed
        """
        
        if not isinstance(hull,Delaunay):
            hull = Delaunay(hull)
        #print(type(hull))#<class 'scipy.spatial.qhull.Delaunay'>
        #print(hull)#<scipy.spatial.qhull.Delaunay object at 0x7fc77e844810>

        return hull.find_simplex(p)>=0

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

        H = json.loads(self.ouster_config_client())["data_format"]["pixels_per_column"]
        W = json.loads(self.ouster_config_client())["data_format"]["columns_per_frame"]

        maxinten = -1.0

        for row in range(H):
            for col in range(W):

                index = row * W + col
                if(l[index][3]>20):
                    l1.append(l[index][3])
                new_l.append([l[index][0],l[index][1],l[index][2]])
                if(l[index][3]>maxinten):
                    maxinten = l[index][3]
        
        print(len(new_l))
        
        print("input length",len(l))
       
        thresh = st.pstdev(l1)
        print("maxintensity",maxinten)
        print("threshold",thresh)
        mn = []
        for row in range(H):
            for col in range(W):

                index = row * W + col
                if (l[index][0]<1.2 and l[index][0]>0.6 and abs(l[index][1])<1.1 and l[index][3] > thresh):
               
                    x = l[index][0]
                    y = l[index][1]
                    z = l[index][2]
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
                    
                    if(z > -0.2):
                        tot = tot+1 
                        pt = [x, y, z]
                        points.append(pt)
                    else:
                        continue 
                    
        
        
        xpts = np.random.uniform(minx-0.25,maxx,len(points)).tolist()
        
        mn = points
        border = points 
        points = []
        c = 0
        for val in mn:
            
            if(val[0]>1.3):
                points.append([1.3,val[1],val[2]])
            else:    
                points.append(val)
            
            val[0] = xpts[c]
            
            if(val[0]>1.3):
                points.append([1.3,val[1],val[2]])
            else:    
                points.append(val)
            
            c = c+1
      


        
        print("length of points",len(points)) 
        print("input length",len(l))  
        
        bool_list = self.in_hull(np.array(new_l),np.array(points))
        
      
        
        axes = list(compress(new_l,bool_list))
        axes_maxy = max(axes, key=lambda x: x[1])[1]
        axes_miny = min(axes, key=lambda x: x[1])[1]
        axes_maxz = max(axes, key=lambda x: x[2])[2]
        axes_minz = min(axes, key=lambda x: x[2])[2]

        new_axes = axes
        for point_border in border:
            for point_xyz in axes:
                if(abs(point_border[1]-point_xyz[1])<=0.0001):
                    new_axes.remove(point_xyz)
                        
        rospy.set_param('axalta/ccscore/paintjobprocess/MINY',axes_miny)
        rospy.set_param('axalta/ccscore/paintjobprocess/MAXY',axes_maxy)
        rospy.set_param('axalta/ccscore/paintjobprocess/MAXZ',axes_maxz)
        rospy.set_param('axalta/ccscore/paintjobprocess/MINZ',axes_minz)
        
        """ print("removed new value")
        new_axes_maxz = max(new_axes, key=lambda x: x[2])[2]
        new_axes_minz = min(new_axes, key=lambda x: x[2])[2] 
        new_axes_maxy = max(new_axes, key=lambda x: x[1])[1]
        new_axes_miny = min(new_axes, key=lambda x: x[1])[1]
        new_axes_maxx = max(new_axes, key=lambda x: x[0])[0]
        new_axes_minx = min(new_axes, key=lambda x: x[0])[0]  
        print(new_axes_maxz,new_axes_minz,new_axes_maxy,new_axes_miny)
        
        zwise = new_axes_maxz  
        new_points = [] 
        new_line = []
        set_point = new_axes[0][2]  
        print("set_points",set_point) 
        rownum = 0
        for it in range(len(new_axes)-1):
                
                 
                if(abs(set_point-new_axes[it][2]) < 0.01):
                                        
                    new_line.append([new_axes[it][0],new_axes[it][1],new_axes[it][2]])  
                else:
                    set_point = new_axes[it][2]
                    
                    if(len(new_line)>0):
                        new_points.append(new_line)
                        rownum = rownum+1  
                        new_line = []
                      
                    
                        
        print("rownum",rownum)            
        y_new = (np.arange(axes_miny, axes_maxy, 0.005)).tolist() 
        polyfit_points = []
        #print("new_points",new_points)
        xtake = new_points 
        prev = [pt[0] for row in xtake for pt in row if (pt[2]>-0.001 and pt[2]<0.001)]"""
       
        return points,axes



    def ouster_config_client(self):
        rospy.wait_for_service('/os_node/os_config')
        try:
            cfg = rospy.ServiceProxy('/os_node/os_config', OSConfigSrv)
            resp = cfg()
            return resp.metadata
        except rospy.ServiceException as e:
            print("Service call failed: %s") 

    def cloud2bgr(self,cloud):
        
        generator_trav = pc2.read_points(cloud, field_names = ['intensity','x','y','z'], skip_nans = True)
        
        points,axes = self.bgr_gen(generator_trav)
      
        return points,axes

        
if __name__ == '__main__':
    try:
            rospy.init_node('create_cloud_xyzrgb', anonymous=True)
        
            
            obj = AreaSelect()
            header = Header()
            header.frame_id = "os_sensor"
            rate = rospy.Rate(10)
            pub = rospy.Publisher("point_cloud2", PointCloud2, queue_size=2)
            
            pub2 = rospy.Publisher("axes_points", PointCloud2, queue_size=2)
         
            
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

                  
                rate.sleep()
            rospy.spin()

    except rospy.ROSInterruptException:
        pass


