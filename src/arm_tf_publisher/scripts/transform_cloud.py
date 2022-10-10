#!/usr/bin/env python
import rospy
import math
import tf
from tf import TransformListener
from geometry_msgs.msg import Pose,PoseStamped
import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2, PointField

rospy.init_node('transform_cloud')

def pc2CallBack(msg):
    print("In callback")
    trans = tf_buffer.lookup_transform("mir_link", msg.header.frame_id, msg.header.stamp, rospy.Duration(5.0))
    print(trans)
    cloud_out = do_transform_cloud(msg, trans)
    pub.publish(cloud_out)
    #print("In callback End") 

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
   
trans = []
rot = []
rospy.Subscriber("/filtered_data",PointCloud2,pc2CallBack)
pub = rospy.Publisher("/filtered_data_transformed", PointCloud2, queue_size=2)
print("Declarations done")
cloud_out = PointCloud2()


if __name__ == '__main__':

    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
       
       #listener.waitForTransform('/base_link','/camera_aligned_depth_to_color_frame',rospy.Time(),rospy.Duration(100.0))
       #(trans,rot) = listener.lookupTransform('/base_link', '/camera_aligned_depth_to_color_frame', rospy.Time(0))  
       
       #pub.publish(cloud_out)
       rate.sleep()

    rospy.spin()

