#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import urllib3
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
from move_mir_to_destination import delete_mission_queue
from mir_state import pause_mir

class Door_Detection():
    
    def __init__(self):
        self.detect="False"
        self.init_node()
          
    def init_node(self):
        rospy.init_node("Door_Detection", anonymous=False)
        rospy.Subscriber("/RFID_Data", String, self.callback)
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            if(self.detect=="True"):
                pause_mir()
                print("Door Detected")
                delete_mission_queue()
            else:
                print("Scanning for Door...")    
            rate.sleep()
        rospy.spin()         

    def callback(self,data):
        self.detect = data.data
            
if __name__ == '__main__':
    try:
        Door_Detection()
    except rospy.ROSInterruptException:
        pass