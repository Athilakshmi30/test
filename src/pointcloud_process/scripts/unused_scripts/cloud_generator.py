#!/usr/bin/env python
from __future__ import print_function

import rospy
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2 
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
import math

    
def reconstructedcld():
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              ]

    header = Header()
    header.frame_id = "mir_link"
    header.stamp = rospy.Time.now()


    points_list = []


    """yval = 1.158
    zstep = 0.5/50
    xstep = 0.5/50
    z = -0.25
    height = 1.0
    for i in range(50):
        x = -0.25
        for j in range(50):
            y = 1.158
            
            points_list.append([x,y,z+height])
            x = x + xstep
        z = z + zstep"""
    

    
    """radius = 0.2
    theta = 330.0
    z = 0.75
    step_z = 0.01
    centre_y = 1.5
    
    step_theta = 5.0

    while(z <= 1.5):
        theta = 330.0 
        while(theta > 210.0): 
            x = radius * math.cos(theta*3.14/180.0)
            y = radius * math.sin(theta*3.14/180.0)
            points_list.append([x,y+centre_y,z])
            theta = theta - step_theta
        
        z = z + step_z    
    """

    """count = 0    
    zstep = 0.5/50
    xstep = 0.5/50
    z = -0.25
    height = 1.0
    for i in range(50):
        x = -0.25
        for j in range(50):
            if(z==0.0):
                break 
            if(z > 0.0):
                y = 1.158
            if(z < 0.0):
                y = 1.058
            points_list.append([x,y,z+height])
            x = x + xstep
        z = z + zstep

    
    for k in range(5):
        x = -0.25
        for i in range(50):
            y = 1.058 + 0.02 * k  
            points_list.append([x,y,height])
            x = x + xstep"""

    """count = 0    
    zstep = 0.5/50
    xstep = 0.5/50
    z = -0.25
    height = 1.0
    for i in range(50):
        x = -0.25
        for j in range(50):
            if(z > -0.05 and z < 0.1): 
                y = (0.5)*(z+height) + 0.605
            elif(z <= -0.05):
                y = 1.08
            else:
                y = 1.15
            points_list.append([x,y,z+height])
            x = x + xstep
        z = z + zstep"""

    """step_psi = 2.0
    step_theta = 2.0
    radius = 1.0
    theta = 0.0
    psi = 0.0
    while(theta<180.0):
        psi = 0.0
        while(psi<360.0):
            x = radius * math.sin(psi*math.pi/180.0) * math.cos(theta*math.pi/180.0)
            y = radius * math.sin(psi*math.pi/180.0) * math.sin(theta*math.pi/180.0) +2.06
            z = radius * math.cos(psi*math.pi/180.0) + 0.6
            if(y<1.12):
                points_list.append([x,y,z])
            psi = psi + step_psi
        theta = theta + step_theta """

    """count = 0    
    zstep = 0.5/50
    xstep = 0.5/50
    z = -0.20
    height = 0.85
    for i in range(35):
        x = -0.25
        for j in range(50):
            if(z > 0.0): 
                y = (0.5)*(z+height) + 0.755
            
            #else:
            #    y = -(0.5)*(z+height) + 1.605

                points_list.append([x,y,z+height])
            x = x + xstep
        z = z + zstep"""

    points_list = []
  
    zstep = 0.5/50
    xstep = 0.5/50
    z = -0.20
    height = 0.75
    for rowcount in range(35):  
        x = -0.25
        for pointcount in range(50):   
            if(z+height == 0.89):
                print("yes")
                y = 0.8
                step_y = 0.01
                while(y < 1.1):
                    points_list.append([x,y,z+height])
                    y = y + step_y
            else:
                points_list.append([x,0.8,z+height])
            x = x + xstep
        z = z + zstep  

    print("-----------------------cloud creation started------------------------------")
    pc2_roi = point_cloud2.create_cloud(header, fields, points_list)
    print("--------------cloud created-----------------")
  
    
    pub.publish(pc2_roi)

if __name__ == '__main__':
    rospy.init_node('cloud_generator')
    #pub = rospy.Publisher('points2', PointCloud2, queue_size=100)
    rate = rospy.Rate(10)
        
    pub = rospy.Publisher("/filtered_data",PointCloud2,queue_size=1)
    
    rospy.Subscriber("/roi_points",PointCloud2,reconstructedcld)
    while(not rospy.is_shutdown()):
        reconstructedcld()
        rate.sleep()
    rospy.spin()
    
