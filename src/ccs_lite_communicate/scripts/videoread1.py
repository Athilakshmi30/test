#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
import base64
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node("video_publisher1", anonymous=True)
img_pub = rospy.Publisher("map_mir_in_ccs", String,queue_size=2)
# Opens the Video file

rate = rospy.Rate(5)
#rospy.Subscriber("/map_image",Image,image_callback)
frame = cv2.imread('/home/axalta_ws/img/imageintensity.png')
bridge = CvBridge()

def image_callback(data):
    try:
        #cv2_img = bridge.imgmsg_to_cv2(data, "bgr8")
        cv2_img = bridge.imgmsg_to_cv2(data,"bgr8")
        global frame
        frame = cv2_img
    except CvBridgeError(e):
        print(e)

rospy.Subscriber("/map_image",Image,image_callback)

while not rospy.is_shutdown():
    
    # Publish image.
    
    ret,bufferval = cv2.imencode(".jpg",frame)
  #  jpgtxt = base64.b64encode(bufferval)
    jpgtxt = base64.b64encode(bufferval)
   # objarea_string = base64.b64encode(img_file.read())
    jpgtxt = jpgtxt.decode("UTF-8")
    img_pub.publish(jpgtxt)
    rate.sleep()
    
rospy.spin()
