#!/usr/bin/env python
from enum import EnumMeta
from traceback import print_tb
import rtde_control
import rospy
import time
from std_msgs.msg import String, Int32MultiArray
import rtde_receive
from arm_ctrl_navigate.msg import Plannedpath, PathStamped, Path, JointPath, JointValues, ListEachPoint, EachPoint
from rtde_python.srv import SendDataUR,SendDataURResponse
import rtde_io
"""
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.12.16")

/home/axalta/axalta_ws/totg_cord/AfterTOTGtimeparameterization_movep.txt
actual_q = rtde_r.getActualTCPPose()
print("actual_TCP_pose",actual_q)"""

"""position: 
  x: -0.666976759591
  y: 0.66444990386
  z: 1.70112719929
orientation: 
  x: -0.00947138065397
  y: -0.826186272304
  z: -0.536478947726
  w: 0.171804758525
 x: -1.9604349, y: -0.2772618, z: -2.9321337"""

#rtde_c = rtde_control.RTDEControlInterface("192.168.12.16")

rospy.set_param("user_req_speed", 200.00)
rospy.set_param("Execute_coat", "None")
rospy.set_param("validating_coat", "None")
# rtde_io
#rtde_c.moveJ([0, -1.57,0, -1.57,  3.14,0], 0.1, 0.5,False)



#rtde_io = rtde_io.RTDEIOInterface("192.168.12.16")
#rtde_c.moveJ([1.35275 , -1.47851 , 1.49124 , -2.50856 , -1.35773 , -0.139422], 0.49, 2,False)

#-1.66477  -1.70912  -1.16322  -3.74821  -1.6016  -3.13374
# home 1
#rtde_c.moveJ([1.5721356868743896, -1.7770129642882289, 1.6181653181659144, -2.861213823358053, -1.4771245161639612, 0.0], 0.49, 2,False)

# /home/axalta/axalta_ws/totg_cord/vid_AfterTOTGtimeparameterization_movep.txt
#file1 = open('/home/axalta/axalta_ws/totg_cord/demo_inti/AfterTOTGtimeparameterization_movep.txt', 'r')

#lines = file1.readlines()

pt_flag_list_seq = []
recv_callback_point_flag_list = False
data_send = False


pt_flag_recev = False
# for 490 mm/s
# v = 3.1 rad/s
# a = 2.2 rad/s^2

# for 200 mm/s
# v =    rad/s
# a =    rad/s^2
#d,e,f = 1.9223265776494836e-05, -0.0013521065486013259, 0.13120863101353641


def cartesian_speed_to_angluar_accel_relation(x, d, e, f):
    return d*(x**2)+e*x + f


def send_data(pt_flag_list_seq, lines):
    rtde_c = rtde_control.RTDEControlInterface("192.168.12.30")
    d, e, f = 1.9223265776494836e-05, -0.0013521065486013259, 0.13120863101353641
    user_req_speed = float(rospy.get_param("user_req_speed"))
    a = cartesian_speed_to_angluar_accel_relation(user_req_speed, d, e, f)
    #print("Calculated angular acceleration is : ", a)
    v = 1.57
    a = 2.3
    total_path = []
    # sequence of points are as follow 000002 001 000000000001 002 002 0001 00000000000001
    # blend radius for intermediate points 
    br_11 = 0.08
    br_21 = 0.08#0.03
    br_12 = 0.01#0.03     
    br_22 = 0.01
    
    # blend radius for point
    br_2 = 0.0
    # blend radius for end point
    br_start_2_end_1 = 0.08 #0.03             
    br_start_1_end_1 = 0.08
        
    #print("trigger_pt",trigger_pt)
    #print("pt_flag_list_seq",pt_flag_list_seq)
    val_br = br_11#0.09
    br_list = []
    step_count = val_br/(len(pt_flag_list_seq[0])-1)
    #print("len(pt_flag_list_seq[0]) : ",len(pt_flag_list_seq[0]))
    #print("---------------------------------------------------------------------------------------------")    
    #print("pt_flag_list_seq : ",pt_flag_list_seq)
    for idx in pt_flag_list_seq[0]:
        br_list.append(round(val_br,2))
        val_br = val_br - step_count
        if(val_br < 0):
            val_br = 0.0    
    for pt in pt_flag_list_seq[1:]:
        if (pt[0]== 2) and (pt[-1]==1):
            #print("--------------------21---------------------")
            if len(pt)>2:
                for i in pt[1:-1]:
                    br_list.append(br_21)#0.01
            br_list.append(br_start_2_end_1)#0.02
            
        elif (pt[0]== 1) and (pt[-1]==2):
            #print("--------------------12---------------------")
            if len(pt)>2:
                new_val_br = br_11
                new_step_count = new_val_br/(len(pt)-2)
                for i in pt[1:-1]:
                    new_val_br = new_val_br - new_step_count
                    br_list.append(new_val_br)#new_val_br
            br_list.append(br_2)

                
        elif (pt[0]== 2) and (pt[-1]==2):
            #print("--------------------22---------------------")
            if len(pt)>2:
                for i in pt[1:-1]:
                    br_list.append(br_22)#br_22
            br_list.append(br_2) 
            
        elif (pt[0]== 1) and (pt[-1]==1):
            #print("--------------------11---------------------")
            for i in pt[1:-1]:
                br_list.append(br_11)#0.09
            br_list.append(br_start_1_end_1)#0.01

         
    for (bd_radius, point_val) in zip(br_list, lines):
        path = [float(point_val[0]), float(point_val[1]), float(point_val[2]), float(
            point_val[3]), float(point_val[4]), float(point_val[5]), v, a, bd_radius, False]
        total_path.append(path)
    #print("total_path: ", total_path)
    print("Sending joint value to ur ...")
    rtde_c.moveJ(total_path)
    # rtde_c.disconnect()
    data_send = True
    
    
    while rospy.get_param("Z_Level") == "None":
        print("Didn't received updated z_vel")
    print("Received updated z_vel")
    z_vel = rospy.get_param("Z_Level")
    home_z_lev = 0.8573396491929234
    l_path = []
    l_total_path = []
    if (z_vel>home_z_lev):
        print("Logic is not added for z_vel>home_z_lev")
    else:
        while z_vel<home_z_lev:
            l_path = [-0.07220063689110005, -0.6352368820749019, z_vel, -0.027486296000136207, 2.415684011057212, -1.9647414787826962,0.3,0.25,0.03,False]
            l_total_path.append(l_path)
            z_vel += 0.1
    l_path = [-0.07220063689110005, -0.6352368820749019, home_z_lev, -0.027486296000136207, 2.415684011057212, -1.9647414787826962,0.3,0.2,0.0,False]
    l_total_path.append(l_path) 
    rtde_c.moveL(l_total_path)   
    #0.8573396491929234
    rospy.set_param("Z_Level","None")  
    home_path_total_path = []
    home_path = []
    #if (rospy.get_param("exec_clear2")==True):
    #    print("reaced for clear2 point")  
    home_path=[-0.059201013422566895, -2.2724411416161576, 2.463431733600692, -3.497628681519415, 0.18430169841810295, 0.23541727599859907,0.3,0.2,0.0,False]
    home_path_total_path.append(home_path) 
    rtde_c.moveJ(home_path_total_path)   

    print("Sending Data Completed")
    rtde_c.disconnect()

def send_process():
    global executed_sealer, executed_base1, executed_base2, executed_clear1, executed_clear2
    rospy.init_node('final_sendData_ur', anonymous=True)

    s = rospy.Service('send_data_ur_server', SendDataUR,
                      handle_send_data_ur_server)
    rate = rospy.Rate(10)
    rospy.loginfo("final_sendData_ur node up and running... ")

    while not rospy.is_shutdown():

        rate.sleep()
    rospy.spin()


def handle_send_data_ur_server(req):
    joint_path = []
    #print("req.trajectory.joint_trajectory.points : ",req.trajectory.joint_trajectory.points)
    #print("------------------------------------------------------------------------------------------------")
    for pt in req.trajectory.joint_trajectory.points:
        #print("pt.positions : ", pt.positions)
        joint_val = []
        for jval in pt.positions:
            #print("jval : ",jval)
            joint_val.append(jval)
        joint_path.append(joint_val)
    #print("------------------------------------------------------------------------------------------------")   
    _pt_flag_list_seq = []    
    for pt_trigger in req.list_each_point.list_pt_msg:
        _pt_flag_list_seq.append(pt_trigger.points)        
    #print("_pt_flag_list_seq : ",_pt_flag_list_seq)
    #print("joint_path : ",joint_path)
    #while rospy.get_param("robot_state")=='off':
    #    "Waiting for Robot to turn on in Receive"
    send_data(_pt_flag_list_seq, joint_path)

    return SendDataURResponse(True)


if __name__ == '__main__':
    send_process()
