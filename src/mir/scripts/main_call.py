#!/usr/bin/env python
import rospy
import MIR_DistanceCollector_Node
obj=MIR_DistanceCollector_Node.Distance_Measurement()

class Error_Correction():
    def __init__(self):
        self.counter=0
        self.currdist1=0
        self.currdist2=0
        self.currdist3=0
        self.refdist1=0
        self.refdist2=0
        self.refdist3=0
        self.store_reff_dist()
        self.store_curr_dist()

    def store_reff_dist(self):  
        refdist=obj.init_node() 
        self.refdist1=refdist[0]
        self.refdist2=refdist[1]
        self.refdist3=refdist[2]
        print("ref Dist1",self.refdist1," ref Dist2",self.refdist3,"ref Dist3",self.refdist3)  

    def store_curr_dist(self): 
        currdist=obj.init_node() 
        self.currdist1=currdist[0]
        self.currdist2=currdist[1]
        self.currdist3=currdist[2]
        print("ref Dist1",self.currdist1," ref Dist2",self.currdist3,"ref Dist3",self.currdist3)
        
if __name__ == '__main__':
    try:
        Error_Correction()
    except rospy.ROSInterruptException:
        pass