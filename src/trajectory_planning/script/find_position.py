#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg 
import geometry_msgs.msg 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface',anonymous=True)

robot = moveit_commander.RobotCommander()
#scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander('manipulator')

#wpose = group.get_current_joint_values()
#wpose=group.get_joint_value_target()
#quaternion = [wpose.orientation.x,wpose.orientation.y,wpose.orientation.z,wpose.orientation.w]
#roll, pitch, yaw = euler_from_quaternion(quaternion)
#print("wpose")
#print(wpose)
'''print(wpose)
print("eurel form")
print(roll,pitch,yaw)'''

print(group.get_current_pose())
print(group.get_current_joint_values())
#print(group.get_current_rpy())

rospy.sleep(5)

moveit_commander.roscpp_shutdown()
