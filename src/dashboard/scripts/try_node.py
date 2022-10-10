#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def get_try_data():
    print("Inside get_try_data")
    rate = rospy.Rate(10000)
    try_pub = rospy.Publisher('try_publish',Bool,queue_size=10)
    while not rospy.is_shutdown():
        if(rospy.has_param("testParam")):
            #print()
            if(rospy.get_param("testParam")):
                    try_pub.publish(True)
            rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('try_node', anonymous=True)
        get_try_data()
    except rospy.ROSInterruptException:
        pass