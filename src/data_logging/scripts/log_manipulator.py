#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from rosgraph_msgs.msg import Log 
from datetime import date
today = date.today()
date_ = today.strftime("%d_%m_%Y")
levels = {'DEBUG':1, #debug level
'INFO':2 , #general level
'WARN':4 , #warning level
'ERROR':8, #error level
'FATAL':16}
data_self = ""
level = -1
current = 1 

def filewrite():
    global current
    today = date.today()
    date_ = today.strftime("%d_%m_%Y") 
    current = str(date_)+"_"+str(rospy.get_param("JOBID"))
    current ="/home/axalta/axalta_ws/src/data_logging/"+str(current)+".txt"
    #print(type(current))
    if level != -1:
        l = [k for k,v in levels.items() if v == level][0]
        global data_self
        data_new = l+" "+data_self
        file1 = open(current, "a") 
        file1.write(data_new) 
        file1.write("\n")

def log_callback(data):
    global data_self
    global level 
    data_self = str(data.name+" "+data.msg+" "+data.file)
    level = (data.level)
    #print(data_self)

def get_log():

    rospy.Subscriber("rosout_agg",Log, log_callback)#all data will be available in rosout for logging
 
def log_manipulator_restart():
    today = date.today()
    date_ = today.strftime("%d_%m_%Y")
    data_self = ""
    level = -1
    current = 1

if __name__ == '__main__':
    try:
        rospy.init_node('log_manipulator', anonymous=True) 
        get_log()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown(): 
            filewrite()
            
            rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


