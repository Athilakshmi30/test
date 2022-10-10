#!/usr/bin/env python
import math
import time
from cmath import acos
import rospy
from std_msgs.msg import Float64MultiArray

class Distance_Measurement():
    
    def __init__(self):
        self.dist1 =0.0
        self.dist2 =0.0
        self.dist3 =0.0
        self.avgdist1=0.0
        self.avgdist2=0.0
        self.avgdist3=0.0
        self.refdist1 =0.0
        self.refdist2 =0.0
        self.refdist3 =0.0
        self.refdist1list=[]
        self.refdist2list=[]
        self.refdist3list=[]
        self.dist1list=[]
        self.dist2list=[]
        self.dist3list=[]
        self.alpha = 0.0
        self.counterflag = 0
        self.init_node()
        

    def refdist_calc(self):
        self.refdist1list.append(self.dist1)
        self.refdist2list.append(self.dist2)
        self.refdist3list.append(self.dist3)

        self.dist1list.append(self.dist1)
        self.dist2list.append(self.dist2)
        self.dist3list.append(self.dist3)

        self.refdist1 =sum(self.refdist1list)/len(self.refdist1list)
        self.refdist2 =sum(self.refdist2list)/len(self.refdist2list)
        self.refdist3 =sum(self.refdist3list)/len(self.refdist3list)

        self.avgdist1 = sum(self.dist1list)/len(self.dist1list)
        self.avgdist2 = sum(self.dist2list)/len(self.dist2list)
        self.avgdist3 = sum(self.dist3list)/len(self.dist3list)

          
        #print(self.refdist1,self.refdist2,self.refdist3)
                

    def error_calc(self):
        #print(self.refdist1,self.dist1)

        if(self.refdist1<=self.avgdist1+0.5):
            #print(self.refdist1)
            #print(self.avgdist1)
            if(int(self.avgdist2)>int(self.avgdist3)):
                print("postive Error")
                self.alpha=round((float(self.refdist1)/self.avgdist1),3)
             #!/usr/bin/python
                print("Negative Error")
                self.alpha=round((float(self.refdist1)/self.avgdist1),2)
                #print(self.alpha)
                self.alpha=math.acos(self.alpha)
                #print(self.alpha)
                self.alpha=math.degrees(self.alpha)
                print(self.alpha)
                #self.alpha=-1*self.alpha
            else:
                print(" No Error")
        else:
            print("Invalid")   

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

           # print(self.dist1list)

          
    def init_node(self):
        rospy.init_node("Distance_Measurement", anonymous=False)
        rospy.Subscriber("/Distance", Float32MultiArray, self.callback)
        rate=rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if(self.counterflag>=2 and self.counterflag<=101):
               print(self.refdist1,self.refdist2,self.refdist3)
               self.refdist_calc()
            elif(self.counterflag>101):   
               print(self.avgdist1,self.avgdist2,self.avgdist3) 
               self.moving_avg() 
               self.error_calc()
            self.counterflag=self.counterflag+1
            rate.sleep()
        rospy.spin()         

    def callback(self,data):
        
        self.dist1 = float(data.data[0]*(500/1023.0))
        self.dist2 = float(data.data[1]*(500/1023.0))
        self.dist3 = float(data.data[2]*(500/1023.0))     
        #print("Dist 1",self.dist1,"Dist 2",self.dist2,"Dist 3",self.dist3)
            
if __name__ == '__main__':
    
    try:
        Distance_Measurement()
    except rospy.ROSInterruptException:
        pass




