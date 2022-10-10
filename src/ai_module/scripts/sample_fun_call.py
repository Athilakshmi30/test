#!/usr/bin/env python3

import rospy
from std_msgs.msg import *
from ai_module.msg import *

# global s_a_a
# global s_b_b
# global samp
# s_a = Bool()
# s_b = Bool()
samp = SampleMsg()
# s_a_a = False
# s_b_b = False

def sample_call_back_1(data):
   samp.a = data.data
   #   global s_a_a
   #   s_a = data
   #   print(s_a)
   #   if(s_a == True):
   #      print("Im true")
   #      s_a_a = True
   #   else:
   #      print("Im fasle")
   #      s_a_a = False
   #   #print(data)
   #   #print(s_a)
   #  # print(bool(s_a))
    
def sample_call_back_2(data):
   samp.b = data.data
   #   global s_b_b
   #   s_b = data
   #   if(s_b):
   #      s_b_b = True
   #   else:
   #      s_b_b = False

def Sample_fun_call():
    rospy.Subscriber("sample_publisher_1",Bool,sample_call_back_1)
    rospy.Subscriber("sample_publisher_2",Bool,sample_call_back_2)
    pub1 = rospy.Publisher("sample_publisher_main",SampleMsg,queue_size =3)
    while not rospy.is_shutdown():
      #   samp.a = s_a_a
      #   samp.b = s_b_b
        pub1.publish(samp)
        


    

if __name__ == "__main__":
    rospy.init_node("try_module_node")
    Sample_fun_call()
    rospy.spin()
    #test()