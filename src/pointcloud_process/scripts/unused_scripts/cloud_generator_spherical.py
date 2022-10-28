#!/usr/bin/env python
from __future__ import print_function

import rospy
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2 
from sensor_msgs.msg import PointField
from std_msgs.msg import Header


    
def reconstructedcld():
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              ]

    header = Header()
    header.frame_id = "mir_link"
    header.stamp = rospy.Time.now()


    points_list = []

    
    yval = 1.158
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
    
