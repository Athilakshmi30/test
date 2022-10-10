#!/usr/bin/env python3
from __future__ import print_function

import rospy
from std_msgs.msg import *
from status_check.srv import *
from status_check.msg import *
from dashboard.srv import *
from main_package.msg import *
from move_ur10e.srv import *

# global SealerCoat_isCompleted 
# global BaseCoat1_isCompleted
# global BaseCoat2_isCompleted
# global ClearCoat1_isCompleted
# global ClearCoat2_isCompleted
# global com
# com = CompletionStatus()
SealerCoat_isCompleted = False
BaseCoat1_isCompleted = True
BaseCoat2_isCompleted = False
ClearCoat1_isCompleted = False
ClearCoat2_isCompleted = False  
Feasibility_done = False  
Trajectory_done = False 
Painting_done = False 


class test_cls:
    def __init__(self):
     s1 = rospy.Service('Start_Trajectory_calculation_server', StartTrajCalculation, self.handle_strt_traj_callback)
     s2 = rospy.Service('Start_Painting_server', StartPainting, self.handle_painting_callback)
     Feasible = False
     painting = True
     traj = True
     
     pub2 = rospy.Publisher("sealer_coat_completion_check", Bool, queue_size=3)
     pub3 = rospy.Publisher("base_coat_1_completion_check", Bool, queue_size=3)
     pub4 = rospy.Publisher("base_coat_2_completion_check", Bool, queue_size=3)
     pub5 = rospy.Publisher("clear_coat_1_completion_check", Bool, queue_size=3)
     pub6 = rospy.Publisher("clear_coat_2_completion_check", Bool, queue_size=3)
     pub7 = rospy.Publisher("feasibility_check_completion_status", Bool, queue_size=3)
     pub8 = rospy.Publisher("trajectory_calculation_completion_check", Bool, queue_size=3)
     pub9 = rospy.Publisher("painting_done_check", Bool, queue_size=3)
     while not rospy.is_shutdown():
        # com.SealerCoat_isCompleted = SealerCoat_isCompleted
        # com.BaseCoat1_isCompleted = BaseCoat1_isCompleted
        # com.BaseCoat2_isCompleted = BaseCoat2_isCompleted
        # com.ClearCoat1_isCompleted = ClearCoat1_isCompleted
        # com.ClearCoat2_isCompleted = ClearCoat2_isCompleted 
        # com.Feasibility_done = Feasibility_done
        # com.Trajectory_done = Trajectory_done
        # com.Painting_done = Painting_done
        pub2.publish(SealerCoat_isCompleted)
        pub3.publish(BaseCoat1_isCompleted)
        pub4.publish(BaseCoat2_isCompleted)
        pub5.publish(ClearCoat1_isCompleted)
        pub6.publish(ClearCoat2_isCompleted)
        pub7.publish(Feasible)
        pub8.publish(traj)
        pub9.publish(painting)

    def handle_painting_callback(self,req):
        print("________________inside strt painting server________________")
        resp = TestSrvResponse()
        if(req.start == "sealer"):
            resp.sealer = True
            resp.base1 = False
            resp.base2 = False
            resp.clear1 = False
            resp.clear2 = False
            global SealerCoat_isCompleted
            SealerCoat_isCompleted = True
            
        if(req.start == "base1"):
            resp.sealer = False
            resp.base1 = True
            resp.base2 = False
            resp.clear1 = False
            resp.clear2 = False
            global BaseCoat1_isCompleted
            BaseCoat1_isCompleted = True
        if(req.start == "base2"):
            resp.sealer = False
            resp.base1 = False
            resp.base2 = True
            resp.clear1 = False
            resp.clear2 = False
            global BaseCoat2_isCompleted
            BaseCoat2_isCompleted = True
        if(req.start == "clear1"):
            resp.sealer = False
            resp.base1 = False
            resp.base2 = False
            resp.clear1 = True
            resp.clear2 = False
            global ClearCoat1_isCompleted
            ClearCoat1_isCompleted = True
        if(req.start == "clear2"):
            resp.sealer = False
            resp.base1 = False
            resp.base2 = False
            resp.clear1 = False
            resp.clear2 = True
            global ClearCoat2_isCompleted
            ClearCoat2_isCompleted = True
        
        

        return resp

    def handle_strt_traj_callback(self,req):
        return True

     

     


if __name__ == "__main__":
    rospy.init_node("test_node")
    test_cls()
    rospy.spin()
