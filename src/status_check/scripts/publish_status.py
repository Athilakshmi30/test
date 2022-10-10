#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import rospy.rostime
from status_check.msg import msg_01


def talker():
    pub = rospy.Publisher('topicA', msg_01, queue_size=10)
    #pub = rospy.Publisher('topicB', msg_01, queue_size=10)
    #pub = rospy.Publisher('topicC', msg_01, queue_size=10)
    rate = rospy.Rate(10)
   
    msg=msg_01()
    while not rospy.is_shutdown():
        #hello_str1 = "hello world1 %s" % rospy.get_time()
        msg.a="hello world1"
        
        #dat1=msg.a
        #rospy.loginfo(dat1)
        #pub1.publish(dat1)
        #hello_str2 = "hello world2 %s" % rospy.get_time()
        msg.b="hello world2"
        #dat2=msg.b
        #rospy.loginfo(dat2)
        #pub2.publish(dat2)
        msg.c= "hello world3"
        #dat3=msg.c
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
if __name__ == '__main__':

    try:
        rospy.init_node('talker', anonymous=True)
        talker()
    except rospy.ROSInterruptException:
        pass
