#!/usr/bin/env python
import urllib3
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
from std_msgs.msg import String
from mir_status import get_mir_battery,get_mir_state
import rospy

def get_mir_status():
    
    rospy.init_node("mir_status_node", anonymous=False)
    pub1 = rospy.Publisher("/mir_battery_percentage", String, queue_size=10)
    pub2 = rospy.Publisher("/mir_state", String, queue_size=10)
 
    rate=rospy.Rate(10)  
    while not rospy.is_shutdown():
        try:
            battery_percentage = str(get_mir_battery())
            pub1.publish(battery_percentage)
                
            robot_state = get_mir_state()
            pub2.publish(robot_state)
            rate.sleep()
              
        except KeyboardInterrupt:
            print("terminated from command line")
        except:
            print("Unknown exception in MIR status node")
            
    rospy.spin()              
    
        
if __name__ == '__main__':
    try:
        get_mir_status()
    except rospy.ROSInterruptException:
        pass

