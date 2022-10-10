#!/usr/bin/env python3
from __future__ import print_function

import rospy
from std_msgs.msg import *
from ai_module.msg import *
global sam_a
global sam_b
sam_a = Bool()
sam_b = Bool()

def Sample_fun():
    pub1 = rospy.Publisher("sample_publisher_1",Bool,queue_size =3)
    pub2 = rospy.Publisher("sample_publisher_2",Bool,queue_size =3)
    sam_a = False
    sam_b = True
    while not rospy.is_shutdown():
        pub1.publish(sam_a)
        pub2.publish(sam_b)

    print("im here")


    

if __name__ == "__main__":
    rospy.init_node("sample_node")
    Sample_fun()
    rospy.spin()
    #test()
     

