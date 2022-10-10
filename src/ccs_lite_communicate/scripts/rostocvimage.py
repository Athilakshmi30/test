#! /usr/bin/python
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import base64
from std_msgs.msg import String

# Instantiate CvBridge
bridge = CvBridge()
cnt = 0

rospy.init_node('rostocvimage')
img_pub = rospy.Publisher("obj_identify_comm",String,queue_size=2)

def image_callback(msg):
    #print("Received an image!")
    try:
        global cnt
        if(rospy.has_param('axalta/ccscore/dashboard/PAINTJOBPROCESS') and not rospy.get_param('axalta/ccscore/dashboard/PAINTJOBPROCESS')):
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2_img = cv2_img[:,854:1400,:]
            cv2_img = cv2.resize(cv2_img,(500,500),interpolation = cv2.INTER_NEAREST)
            cv2.imwrite('/home/axalta_ws/img/image_to_ui.png', cv2_img)
            ret,bufferval = cv2.imencode(".jpg",cv2_img)
            jpgtxt = base64.b64encode(bufferval)
            img_pub.publish(jpgtxt)
            cnt = cnt+1
    except CvBridgeError(e):
        print(e)
    
def rostocvimage_restart():
    global cnt
    cnt = 0

def main():
    
    # Define your image topic
    image_topic = "/img_node/intensity_image"
    
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
