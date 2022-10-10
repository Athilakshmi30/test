#!/usr/bin/env python
import rospy
from arm_ctrl_navigate.msg import Path
from geometry_msgs.msg import PoseArray , Pose 
from trajectory_planning.msg import *
from trajectory_planning.srv import *
import numpy as np
from tf.transformations import quaternion_from_euler




def handle_type_change(req):

    path = req.path.path
    P = CustomTrajectory()
    
    for row in path:
          for col in row.path_msg:
            
             pt = CustomTrajectoryPoint()
             pt.poseval.position.x = col.x
             pt.poseval.position.y = col.y
             pt.poseval.position.z = col.z
            
           
             pt.poseval.orientation.x = col.ox
             pt.poseval.orientation.y = col.oy
             pt.poseval.orientation.z = col.oz
             pt.poseval.orientation.w = col.ow 
             pt.is_index = col.is_index
             P.trajectory.append(pt)
              
    return CustomTrajectoryServiceResponse(P)         

def handle_type_change_without_index(req):
    print("got service request")
    path = req.path.path
    P = PoseArray()
    
    for row in path:
          for col in row.path_msg:
            
             pt = Pose()
             pt.position.x = col.x
             pt.position.y = col.y 
             pt.position.z = col.z
             
           
             pt.orientation.x = col.ox
             pt.orientation.y = col.oy
             pt.orientation.z = col.oz
             pt.orientation.w = col.ow 
             P.poses.append(pt)
    print("successfully converted sending response")  
    return CustomTrajectoryServiceWithoutIndexResponse(P) 

  
          
if __name__ == '__main__':
    rospy.init_node('trajectory_type_convertor')
    
    #pub = rospy.Publisher('points2', PointCloud2, queue_size=100)
    rate = rospy.Rate(10)
        
  
    s = rospy.Service("trajectory_type_conversion",CustomTrajectoryService,handle_type_change)

    s1 = rospy.Service("trajectory_type_conversion_without_index",CustomTrajectoryServiceWithoutIndex,handle_type_change_without_index)

    while not rospy.is_shutdown():
     
      rate.sleep()
    
    rospy.spin()
   
