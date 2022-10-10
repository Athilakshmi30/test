#!/usr/bin/env python
import math
import time
import rospy
from std_msgs.msg import Int32

class Distance_Measurement():

    def __init__(self):
        self.counter=0
        self.dist1 =0.0
        self.dist2 =0.0
        self.dist3 =0.0
        self.avgdist1=0.0
        self.avgdist2=0.0
        self.avgdist3=0.0
        self.dist1list=[]
        self.dist2list=[]
        self.dist3list=[]
        self.init_node()

    def moving_avg(self):
        if(len(self.dist1list)==100 and len(self.dist2list)==100 and len(self.dist3list)==100):
                
            self.avgdist1 = sum(self.dist1list)/len(self.dist1list)
            self.avgdist2 = sum(self.dist2list)/len(self.dist2list) 
            self.avgdist3 = sum(self.dist3list)/len(self.dist3list)
            self.dist1list.pop(0)
            self.dist2list.pop(0)
            self.dist3list.pop(0)    
        self.dist1list.append(self.dist1)
        self.dist2list.append(self.dist2)
        self.dist3list.append(self.dist3)   


    def init_node(self):    
        rospy.init_node("Distance_Measurement", anonymous=False)
        rospy.Subscriber("/Distance1", Int32, self.callback1)
        rospy.Subscriber("/Distance2", Int32, self.callback2)
        rospy.Subscriber("/Distance3", Int32, self.callback3)
        rate=rospy.Rate(10)
        while(self.counter<100):
            self.moving_avg()
            self.counter+=1   
            print("Reading data :%d"%self.counter)    
            print("distance1:",self.avgdist1,"distance2:",self.avgdist2,"distance3:",self.avgdist3)
            rate.sleep()
        self.counter=0    
        return self.avgdist1,self.avgdist2,self.avgdist3
        rospy.spin()         
        

    def callback1(self,data):
        self.dist1 = int(data.data*500/1023)         
    def callback2(self,data):
        self.dist2 = int(data.data*500/1023)     
    def callback3(self,data):
        self.dist3 = int(data.data*500/1023)                         
            




