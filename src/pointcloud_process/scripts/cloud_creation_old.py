#!/usr/bin/env python
from __future__ import print_function

import rospy
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2 
from sensor_msgs.msg import PointField
from std_msgs.msg import Header


def reconstructedCallBack(msg):
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgb', 12, PointField.FLOAT32, 1)]

    header = Header()
    header.frame_id = "camera_aligned_depth_to_color_frame_rec"
    header.stamp = rospy.Time.now()

    cloud=msg
    points_list = []
   
    maxy = -1.0
    miny = 10.0
    maxx = -1.0
    minx = 10.0
    minz = 10.0
    maxz = -1.0 
    pc2 = list(point_cloud2.read_points(cloud, skip_nans=True))
    pc2_copy = pc2
    print("-------------pointcloud read completed-------------------")

    for data in pc2:
        x = data[0]
        y = data[1]
        z = data[2]   # - depth
        if data[3] !=0 and z < 1.3 and x > 0.15 and y <0.1 and x < 1.2:   
            z = z - 0.25
            points_list.append([x, y, z,data[3]])
            

    points_list_offsetted = []
    #print(minx,maxx,miny,maxy,minz,maxz)
    offset_top = 0.1
    offset_bottom = 0.1
    offset_left = 0.1
    offset_right = 0.1    


    for pt in points_list:
          
        if(maxy<pt[1]):
            maxy = pt[1]
        if(miny>pt[1]):
            miny = pt[1] 
        if(maxx<pt[0]):
            maxx = pt[0]
        if(minx>pt[0]):
            minx = pt[0]
        if(maxz<pt[2]):
            maxz = pt[2]
        if(minz>pt[2]):
            minz = pt[2]

    print(minx,maxx,miny,maxy,minz,maxz)

    print("maxx + offset_right",maxx + offset_right)
    print("maxy + offset_top",maxy + offset_top)
    print("minx + offset_left",minx + offset_left)
    print("miny + offset_bottom",miny + offset_bottom)

    print(len(pc2))

    for pt in pc2_copy:
        if(pt[0] <= maxx + offset_right and pt[1] <= maxy + offset_top and pt[0] >= minx + offset_left and pt[1] >= miny + offset_bottom and pt[2] <= maxz and pt[2] >= minz):
            points_list_offsetted.append(pt) 
    
    print("-----------------------cloud creation started------------------------------")
    pc2_roi = point_cloud2.create_cloud(header, fields, points_list)
    print("--------------cloud created-----------------")
  
    
    pub.publish(pc2_roi)

if __name__ == '__main__':
    rospy.init_node('cloud_creation')
    #pub = rospy.Publisher('points2', PointCloud2, queue_size=100)
    rate = rospy.Rate(10)
        
    pub = rospy.Publisher("/rgb_pointcloud",PointCloud2,queue_size=1)
    
    rospy.Subscriber("/roi_points",PointCloud2,reconstructedCallBack)
    rospy.spin()
    
