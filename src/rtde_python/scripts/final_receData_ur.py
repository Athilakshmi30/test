#!/usr/bin/env python
from curses import flash
from fcntl import F_SETFL
from glob import glob
#from enum import Flag
from multiprocessing import Process
import os
from traceback import print_tb
import rtde_receive
import rtde_control
import rtde_io
import time
import rospy
import moveit_commander
from std_msgs.msg import String, Int32MultiArray, MultiArrayLayout
from arm_ctrl_navigate.msg import Plannedpath, PathStamped, Path, ListEachPoint, EachPoint
from rtde_python.srv import ReceiveDataUR, ReceiveDataURResponse
#from tf import transformations as tft
from geometry_msgs.msg import Pose, PoseArray
import numpy as np
import tf
import tf_conversions
from tf import transformations as t
#from pyquaternion import Quaternion
#from tinyquaternion import quaternion
rospy.set_param("Execute_coat", "None")
rospy.set_param("Z_Level","None")
rospy.set_param("robot_state","power_off")
rospy.set_param("isEmergencyStopped","not_pressed")
list_result = []
trigger_points_ = []
point_flag_list_all_ = ListEachPoint()
start_validate = False
connect_rtde_recv = False


rospy.set_param("validating_coat", "None")


def trigger_on(rtde_IO):
    print("Trigger On ")
    rtde_IO.setStandardDigitalOut(0, True)
    rtde_IO.setStandardDigitalOut(1, True)
    return 1


def trigger_off(rtde_IO):
    print("Trigger Off ")
    rtde_IO.setStandardDigitalOut(0, False)
    rtde_IO.setStandardDigitalOut(1, False)
    return 1


def compare(moveit_ik_path, trajectory_path, updated_trajectory_path):
    point_flag_list_only = []
    list_result = []
    trigger_points = []
    point_flag_list = EachPoint()
    point_flag_list_all = ListEachPoint()
    add_pt = False
    print("reach for service")
    count = 0
    first_point_to_trigger = False
    point_flag_list_all = ListEachPoint()
    for moveit_point in moveit_ik_path.poses:
        for (traj_point_path, up_traj_point_path) in zip(trajectory_path.path, updated_trajectory_path.path):

            for (traj_point, up_traj_point) in zip(traj_point_path.path_msg, up_traj_point_path.path_msg):

                if traj_point.point_flag:
                    #print("moveit_point",moveit_point.position)
                    #print("traj_point 1",traj_point)
                    #time.sleep(3)
                    #if int(traj_point.x * 1000) == int(moveit_point.position.x * 1000) and int(traj_point.y * 1000) == int(moveit_point.position.y * 1000) and int(traj_point.z * 1000) == int(moveit_point.position.z * 1000):
                    
                    # testing needed
                    
                    if abs(traj_point.x-moveit_point.position.x)<0.002 and abs(traj_point.y-moveit_point.position.y)<0.002 and abs(traj_point.z-moveit_point.position.z)<0.002:
                        #print("traj_point 2",traj_point)
                        #list_result.append(count)
                        pt = [up_traj_point.x, up_traj_point.y, up_traj_point.z]
                        #print("pt : ", pt)
                        #print("traj_point.point_flag : ", traj_point.point_flag)
                        #print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
                        #time.sleep(4)
                        trigger_points.append(pt)
                        point_flag_list = EachPoint()
                        if traj_point.point_flag == 2:
                            point_flag_list_only.append(2)
                            point_flag_list.points = point_flag_list_only
                            point_flag_list_all.list_pt_msg.append(
                                point_flag_list)
                            point_flag_list_only = []
                            point_flag_list_only.append(2)
                            add_pt = True
                        elif traj_point.point_flag == 1:
                            point_flag_list_only.append(1)
                            point_flag_list.points = point_flag_list_only
                            point_flag_list_all.list_pt_msg.append(
                                point_flag_list)
                            point_flag_list_only = []
                            point_flag_list_only.append(1)
                            add_pt = True
                        
                        # point_flag_list_all.list_pt_msg.append(point_flag_list)
                        # trigger_points.append()
                # else:
                #    if(not first_point_to_trigger and int(traj_point.x * 1000) == int(moveit_point.position.x * 1000) and int(traj_point.y * 1000) == int(moveit_point.position.y * 1000) and int(traj_point.z * 1000) == int(moveit_point.position.z * 1000)):
                #        list_result.append(count)
                #       first_point_to_trigger = True
            # print("traj_point_path.pa",traj_point_path.path_msg[-1])
        if not add_pt:
            point_flag_list_only.append(0)
        else:
            add_pt = False
        #count += 1
    # com_point_flag_list_all.all_list_pt_msg.append(point_flag_list_all)

    # print("list_result[0]",list_result)
    # print("point_flag_list",point_flag_list)
    # print("point_flag_list_all",point_flag_list_all)
    # print("point_flag_list_all",len(point_flag_list_all))

    print("trigger_points len", trigger_points)
    return trigger_points, point_flag_list_all



def rece_process():

    rospy.init_node('final_receData_ur', anonymous=True)

    s = rospy.Service('receive_data_ur_server', ReceiveDataUR,
                      handle_receive_data_ur_server)

    rate = rospy.Rate(10)

    rospy.loginfo("final_receData_ur node up and running... ")
    while not rospy.is_shutdown():
        global start_validate,connect_rtde_recv
        check_robot_on = False
        if rospy.get_param("robot_state")=='power_on':
            rtde_IO = rtde_io.RTDEIOInterface("192.168.12.30")  
            print("Robot rtde_IO conntected")
            rospy.set_param("robot_state","checked")          
        trigger_status = False
        
        validate_pos_offset = 0.01

        count = 0
        trig_count = -1
        trig_flag = False
        extra_point_skip_flag = False
        forced_trigger_off = False
        forced_trigger_off_timer_flag = False

        skip_point =0
        emergency_stopped = False
        if not connect_rtde_recv:
            rtde_r = rtde_receive.RTDEReceiveInterface("192.168.12.30")
            print("Robot rtde_receive conntected")
            connect_rtde_recv = True            
        if start_validate:
            rtde_IO.setSpeedSlider(1.0)
            prev_pose = rtde_r.getActualTCPPose()
            for trigger_point in trigger_points_:
    
                while True:
                    emergency_stopped = rtde_r.isEmergencyStopped()
                    # Check for EmergencyStopped
                    if emergency_stopped:
                        rospy.set_param("isEmergencyStopped","pressed")
                        trigger_off(rtde_IO)
                        break
                    
                    #current_seconds = rospy.get_time()    
                    robot_pose = rtde_r.getActualTCPPose()
                    # Skipping the second and third trigger point from trigger on
                    if extra_point_skip_flag:
                        print("inside extra skip point")
                        count+=1
                        forced_trigger_off_timer_flag = True
                        skip_point+=1
                        if skip_point>1:
                            extra_point_skip_flag = False                    
                            skip_point = 0
                        break


                    elif((abs(robot_pose[0]+trigger_points_[-1][0]) < validate_pos_offset) and (abs(robot_pose[1]+trigger_points_[-1][1]) < validate_pos_offset) and (abs(robot_pose[2]-trigger_points_[-1][2]) < validate_pos_offset) and forced_trigger_off_timer_flag):
                        print("Forced Trigger Off")
                        forced_trigger_off = True
                        trigger_off(rtde_IO)
                        break
                    elif (abs(robot_pose[0]+trigger_point[0]) < validate_pos_offset) and (abs(robot_pose[1]+trigger_point[1]) < validate_pos_offset) and (abs(robot_pose[2]-(trigger_point[2])) < validate_pos_offset):
                        print("inside the loop")
                        count += 1
    
                        if count == 1:
                            start_seconds = rospy.get_time()
                            trigger_on(rtde_IO)
                            #print("start_seconds",start_seconds)
                            extra_point_skip_flag = True
    
                            trig_count = 4
                        elif trig_count == count and not trig_flag:
                            print("Normal Trigger Off")    
                            trigger_off(rtde_IO)
                            forced_trigger_off_timer_flag = False
                            trig_count = trig_count + 1
                            trig_flag = True
                        elif trig_count == count and trig_flag:
    
                            trigger_on(rtde_IO)
                            start_seconds = rospy.get_time()    
                            #print("start_seconds",start_seconds)
                            trig_count = trig_count + 3
                            trig_flag = False
                            extra_point_skip_flag = True
    
                        #print("Recevied point : ", trigger_point)
                        print("len trigger_points", len(trigger_points_))
                        print("count-1", count-1)
    
                        break
                    prev_pose = robot_pose
                if forced_trigger_off:
                    break

                if emergency_stopped:
                    print("e Stop")
                    #print("rtde_r.isEmergencyStopped()",rtde_r.isEmergencyStopped())
                    #print("rtde_r.getRobotMode()",rtde_r.getRobotMode())

                    while rtde_r.isEmergencyStopped(): #or rtde_r.getRobotMode()==5: # rtde_r.getRobotMode()==5 for ideal state 
                        print("Waiting for Emergency to release")
                        
                    rospy.set_param("isEmergencyStopped","released")                    
                    break    
                
            start_validate = False
            new_robot_pose = rtde_r.getActualTCPPose() 
            print("Z_Level",new_robot_pose[2])
            #time.sleep(5)
            rospy.set_param("Z_Level",new_robot_pose[2])    
            print("Loop is completed")
            rtde_IO.setSpeedSlider(0.67) 
            #rtde_IO.disconnect()
            rtde_r.disconnect()
            connect_rtde_recv = False
        elif rtde_r.isEmergencyStopped():
            rospy.set_param("isEmergencyStopped","pressed")
            while rtde_r.isEmergencyStopped(): #or rtde_r.getRobotMode()==5: # rtde_r.getRobotMode()==5 for ideal state 
                print("Waiting for Emergency to release")     
            rospy.set_param("isEmergencyStopped","released")  
            rtde_r.disconnect()
            connect_rtde_recv = False
                              

        rate.sleep()
    rospy.spin()


def handle_receive_data_ur_server(req):
    global trigger_points_ ,point_flag_list_all_,start_validate
    trigger_points_, point_flag_list_all_ = compare(
        req.trajectory_fk, req.current_coat_path, req.current_coat_path_transformed)
      
    start_validate = True
    return ReceiveDataURResponse(point_flag_list_all_)


if __name__ == '__main__':
    rece_process()
