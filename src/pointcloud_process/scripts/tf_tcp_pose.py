#!/usr/bin/env python

from arm_ctrl_navigate.msg import Plannedpath,PathStamped,Path
from arm_ctrl_navigate.srv import *
import rospy
import math
from tf import TransformListener
from geometry_msgs.msg import Pose,PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pointcloud_process.srv import ApplyTF, ApplyTFResponse
from dashboard.msg import *
from status_check.msg import stats
from mir.srv import *

from ccs_lite_communicate.srv import *
from ccs_lite_msgs.msg import CcsLiteCmd,EnclosureStatus,Door
import time

print("reached_here")

def apply_tf(path,speed,coat,delay):

    tf_listener_ = TransformListener()
    transformed_path = Path()

    tf_listener_.waitForTransform('mir_link','base_link',rospy.Time(),rospy.Duration(100.0))
    
    for path_msg in path.path:
        row = PathStamped() 
       
        for pose in path_msg.path_msg:
            
            new_point = Plannedpath()
     
            pt = PoseStamped()
           
             
            pt.header.frame_id = "mir_link"
            pt.header.stamp =rospy.Time(0)
            pt.pose.position.x = pose.x
            pt.pose.position.y = pose.y
            pt.pose.position.z = pose.z
            
            pt.pose.orientation.x = pose.ox
            pt.pose.orientation.y = pose.oy
            pt.pose.orientation.z = pose.oz
            pt.pose.orientation.w = pose.ow


            p_in_arm_base = tf_listener_.transformPose("/base_link", pt)
          
          
          
            new_point.x = p_in_arm_base.pose.position.x
            new_point.y = p_in_arm_base.pose.position.y
            new_point.z = p_in_arm_base.pose.position.z
           
            new_point.ox = p_in_arm_base.pose.orientation.x
            new_point.oy = p_in_arm_base.pose.orientation.y
            new_point.oz = p_in_arm_base.pose.orientation.z
            new_point.ow = p_in_arm_base.pose.orientation.w
            new_point.point_flag = pose.point_flag

           
            row.path_msg.append(new_point)
            
        transformed_path.path.append(row)
    
    transformed_path.CoatSpeed = speed
    transformed_path.CoatName = coat
    transformed_path.IndexDelay = int(delay*1000)
    return transformed_path    

def handle_apply_tf_server(req):
    out = ApplyTFResponse()
    out.out_path = apply_tf(req.in_path,req.speed,req.coat_name,req.delay)
    return out

if __name__ == "__main__":
    try:
        rospy.init_node('tf_tcp_pose')


        s = rospy.Service('apply_tf_server', ApplyTF,
                      handle_apply_tf_server)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():

            rate.sleep() 
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(e)
