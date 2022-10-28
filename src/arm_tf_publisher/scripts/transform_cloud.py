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
import time

rospy.init_node('transform_cloud')
cloud_out = PointCloud2()
data_flag = False
restart_flag = False

def pc2CallBack(msg):
    if(not data_flag):
        global cloud_out, data_flag
        print("In callback")
        trans = tf_buffer.lookup_transform("mir_link", msg.header.frame_id, msg.header.stamp, rospy.Duration(5.0))
        print(trans)
        cloud_out = do_transform_cloud(msg, trans)
        data_flag = True
        #print("In callback End") 

def restartNode():

    global cloud_out, data_flag, restart_flag
    cloud_out = PointCloud2()
    
    data_flag = False
    if(rospy.has_param('axalta/ccscore/dashboard/restart_transform_cloud_node_trigger')):
        rospy.set_param('axalta/ccscore/dashboard/restart_transform_cloud_node_trigger',False)
        restart_flag = False
        time.sleep(1)
        rospy.set_param('axalta/ccscore/dashboard/restart_range_filter_after_tf_node_trigger',True)

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
   

if __name__ == '__main__':

    rospy.Subscriber("/filtered_data",PointCloud2,pc2CallBack)
    pub = rospy.Publisher("/filtered_data_transformed", PointCloud2, queue_size=1) 
    print("Declarations done")   
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
       if(rospy.has_param('axalta/ccscore/dashboard/restart_transform_cloud_node_trigger') and rospy.get_param('axalta/ccscore/dashboard/restart_transform_cloud_node_trigger')):  
           print("-------------------Restarting transform cloud node-------------------")
           restart_flag = True
           restartNode()  
       if(data_flag and not restart_flag):
       
           pub.publish(cloud_out)
       rate.sleep()

    rospy.spin()

