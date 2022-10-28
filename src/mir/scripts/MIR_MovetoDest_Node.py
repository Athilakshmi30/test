#!/usr/bin/env python
import urllib3
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

from mir_status import get_mir_state
from mir_state import start_mir,pause_mir
from move_mir_to_destination import move_to_destination
from mir.srv import MoveToDestination, MoveToDestinationResponse
import rospy
import time

def move_to_destination_server():

    rospy.init_node('move_to_destination_node')
    s1 = rospy.Service('move_to_destination',MoveToDestination,handle_move_to_destination)
    print("Ready to move to destination")
    rospy.spin()

def handle_move_to_destination(req):
    print("Destination :",req.destination)
    status="Waiting....."
    
    if(get_mir_state()=="Pause" or get_mir_state()=="Ready"): 
        
        movetodest_status=move_to_destination(req.destination)
        start_mir()
        time.sleep(1)
        while(get_mir_state()=="Executing"):
            rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Robot is in motion..")
            rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE",50)
            print("MIR moving to "+ req.destination +"....")
        
       # status="MIR Reached "+ req.destination completed
        status="completed"
        
        print("MIR Reached "+req.destination)
        pause_mir()

    else:
        print("MIR not moving!")    
               
    return status 

if __name__ == '__main__':
    try:
        move_to_destination_server()
    except rospy.ROSInterruptException:
        pass

