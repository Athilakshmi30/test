#!/usr/bin/env python
import cv2
import rospy
import base64
from std_msgs.msg import String




rospy.init_node("video_publisher", anonymous=True)

def image_callback(data):
    global jpgtxt1
    jpgtxt1 = data

def map_callback(data):
    global map_relay
    map_relay = data

img_pub = rospy.Publisher("map_mir_to_main",String,queue_size=2)
img_pub1 = rospy.Publisher("obj_identify",String,queue_size=2)
rospy.Subscriber("obj_identify_comm", String, image_callback)
rospy.Subscriber("map_mir_in_ccs", String, map_callback)
jpgtxt1 = String()
map_relay = String()



rate = rospy.Rate(5)

while not rospy.is_shutdown():
  
    # Publish image.
        
    img_pub.publish(map_relay)
    img_pub1.publish(jpgtxt1)
        
     

    rate.sleep()
    
rospy.spin()
