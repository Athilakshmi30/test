#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import rospy.rostime
from status_check.msg import stats

#from msg.msg import string_dat
data_self1 = ""
data_self2 = ""
data_self3 = "" 
data_self4 = ""
 
def callback1(msg):
    global data_self1 
    data_self1 = msg

def callback2(msg):
    global data_self2
    data_self2 = msg

def callback3(msg):
    global data_self3
    data_self3 = msg

def callback4(msg):
    global data_self4
    data_self3 = msg

def status_check():
    
    pub_1=rospy.Publisher('topicD',msg_01, queue_size=10)
    parameter_names= rospy.get_param_names()
    pub_2=rospy.Publisher('topicE',String, queue_size=10)

    rospy.Subscriber("mir_status", stats, callback1)
    rospy.Subscriber("lidar_status", stats, callback2)
    rospy.Subscriber("model_status", stats, callback3)
    rospy.Subscriber("arm_status", stats, callback4)                  
    jobid_values = rospy.get_param('JOBID')
    status_values = rospy.get_param('status_code')
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        msg.d=data_self1
        msg.e=data_self2
        msg.f=data_self3
        #msg.status2 = data_self2
        #msg.status3 = data_self3
        parameter_str="Job id : "+jobid_values+ " Status code : "+ status_values+ "time : %s " % rospy.get_time()
        rospy.loginfo(parameter_str)
        pub_2.publish(parameter_str)
        #status_str="status 1 : "+ status1 +" status 2 :  "+status2+" status 3 :  "+status3 + "time : %s " % rospy.get_time()

        rospy.loginfo(msg)
        pub_1.publish(msg)
        rate.sleep()
    


def parameter_setting():
    
    
    input_value1=rospy.get_param("/JOBID")
    rospy.set_param('JOBID', input_value1)

    input_value2=rospy.get_param("/status_code")
    rospy.set_param('status_code', input_value2)


if __name__ == '__main__':
    msg=stats()
    rospy.init_node('Status_check_node',anonymous=True)
    #__init__(self, secs=0, nsecs=0)
    global start
    start=1
    try:
        if start==1:
            parameter_setting()
            
            start=2
        status_check()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
