#!/usr/bin/env python
# from ctypes.wintypes import BOOL

import time
import dashboard_client
import script_client
import rtde_control
import rospy
import os
import subprocess as sp
import rtde_control
from rtde_python.srv import *
import threading
from std_msgs.msg import Bool
hostename_robot = "192.168.12.30"
dashboard_Client = dashboard_client.DashboardClient(hostename_robot)
rospy.set_param("robot_state","power_off")

def start_capture():
     
    # cmd ='python /home/axalta/Desktop/Pavan_Axalta/Version2/ReconstructionSystem/sensors/realsense_recorder.py --record_imgs --output_folder /home/axalta/Desktop/Pavan_Axalta/Version2/Data/'
    cmd_image_rec = os.system("/home/axalta_ws/src/ai_module/sdk_launcher.sh")
    if(rospy.has_param("Windows/Reconstruction_check") and rospy.get_param("Windows/Reconstruction_check")):
        cmd_image_copy = os.system("cp /mnt/c/Users/axalta/Desktop/shared/image/ReconstructedImage.png /home/axalta_ws/img/")
        os.rename("/home/axalta_ws/img/ReconstructedImage.png","/home/axalta_ws/img/reconstructed_image.png")
        rospy.set_param("axalta/ccscore/dashboard/ReconstructionDoneCheck",True)
    
     

      
def call_ur_program(program_name):
    rospy.loginfo("Program loading...")
    rospy.loginfo(program_name)
    # f_program_load = "load "+program_name+" \n" 
    # dashboard_client.send(f_program_load)
    # result = dashboard_client.receive()
    # rospy.loginfo(result)
    dashboard_Client.loadURP(program_name)
    rospy.loginfo("Program loaded")
    rospy.loginfo(program_name)
    if(program_name == "normal_home_scanning_start.urp"):
        rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Robotic Arm is Moving to the Scanning Position..")
        rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 50)

    if(program_name == "Scanning_last_paint_home.urp"):
        rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Robotic Arm is Moving to the Painting Position..")
        rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 50)

    dashboard_Client.play()
                
    time.sleep(0.5)
    while dashboard_Client.running():
        pass
        #print("Scanning Program Running ...")



def handle_start_robot(req):
    res = False
    print("req",req)
    if (req.request):
        while True:
            try:
                dashboard_Client.connect(2000)
                if dashboard_Client.isConnected():
                    print("connected and started")
                    dashboard_Client.powerOn()
                    time.sleep(5)
                    dashboard_Client.brakeRelease()
                    print("Ensure it is in normal home")
                    while (dashboard_Client.robotmode() != u'Robotmode: RUNNING'):
                        pass 
                    #time.sleep(25)
                    try:
                        
                        call_ur_program("normal_home_scanning_start.urp")
                        print("Program finished")
                        time.sleep(1)
                        res = True
                        rospy.set_param("axalta/ccscore/dashboard/ARMScanningPositionReached",True)  
                        break              
                    except Exception as e: 
                        rospy.loginfo("Error",e)
                        res = False  
            except RuntimeError:
                print("Waiting for Robot to turn on")        

        return StartRobotResponse(res)

def handle_start_scanning(req):
            res = False
            try:
                t1 = threading.Thread(target=start_capture)
                t1.start()
                
                time.sleep(20)
                rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Scanning is in progress..")
                rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 50)
                #extProc = sp.Popen(['python','/home/robotics/Open3D-0.9.0/examples/Python/ReconstructionSystem/sensors/realsense_recorder.py','--record_imgs']) 
                abc_jajaja = call_ur_program("try6_Updated_Without170_scanning_end.urp")
                print("Program finished")
                #sp.Popen.terminate(extProc) 
                
                time.sleep(1)
                #rtde_io.setSpeedSlider(1.0)
                time.sleep(1)
                rospy.set_param("axalta/ccscore/dashboard/SCANNINGDONECHECK",True)
                rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Scanning has completed")
                rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 100) 
                res = True

            except Exception as e: 
                rospy.loginfo("Error",e)
                res = False
            print("response in robot dashboard for start scanning is",res)
            print("response type in robot dashboard for start scanning is",type(res))
            return StartRobotScanningResponse(res) 



def handle_move_to_painting(req):
            res = False
            try:
                time.sleep(1)  

                call_ur_program("Scanning_last_paint_home.urp")
                print("Program finished")
                rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Robotic Arm has Reached Painting Position")
                rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 100)
                time.sleep(2)
                print("Please attach paintgun and wait for trajectory calculation and painting to complete--------------------") 
               
                # call_ur_program("Paint_end_normal_home.urp")
                # print("Program finished") 
             

                if rospy.get_param("robot_state")=='power_off':
                    rospy.set_param("robot_state","power_on")

                res = True

            except Exception as e: 
                rospy.loginfo("Error",e)
                res = False

            print("response in robot dashboard for paint pos is",res)
            print("response type in robot dashboard for paint pos is",type(res))
            return RobotMovePaintPosResponse(res)


def start_robot():
    rospy.init_node('robot_dashboard')
    pub_scanning_done_status = rospy.Publisher('/rtde_python/scanning_done_status_to_main', Bool, queue_size=3)
    pub_reconstruction_done_status = rospy.Publisher('/rtde_python/reconstruction_done_status_to_main', Bool, queue_size=3)
    pub_arm_scanning_pos_status = rospy.Publisher('/rtde_python/arm_scanning_pos_reached_status_to_main', Bool, queue_size=3)
    pub_arm_painting_pos_status = rospy.Publisher('/rtde_python/arm_painting_pos_reached_status_to_main', Bool, queue_size=3)
         
    rate = rospy.Rate(10)
    s = rospy.Service('start_ur_service', StartRobot, handle_start_robot)
    scanning_to_end_pos = rospy.Service('/rtde_python/arm_start_scanning', StartRobotScanning, handle_start_scanning)
    end_to_painting_pos = rospy.Service('/rtde_python/end_to_paint_pos', RobotMovePaintPos, handle_move_to_painting)

    while not rospy.is_shutdown():
        if(rospy.get_param("axalta/ccscore/dashboard/SCANNINGDONECHECK")):
            scanning_status = True
        else:
            scanning_status = False
        if(rospy.get_param("axalta/ccscore/dashboard/ReconstructionDoneCheck")):
            reconstruction_status = True
        else:
            reconstruction_status = False
        if(rospy.get_param("axalta/ccscore/dashboard/ARMScanningPositionReached")):
            arm_scanning_pos_status = True
        else:
            arm_scanning_pos_status = False
        if(rospy.get_param("axalta/ccscore/dashboard/ARMPaintingPositionReached")):
            arm_painting_pos_status = True
        else:
            arm_painting_pos_status = False

        
        pub_scanning_done_status.publish(scanning_status)
        pub_reconstruction_done_status.publish(reconstruction_status)
        pub_arm_scanning_pos_status.publish(arm_scanning_pos_status)
        pub_arm_painting_pos_status.publish(arm_painting_pos_status)


        print("rospy.get_param(",rospy.get_param("isEmergencyStopped"))
        if rospy.get_param("isEmergencyStopped")=='released':
            try:
                dashboard_Client.connect(2000)
                if dashboard_Client.isConnected():
                    print("contect")
                    #time.sleep(10)
                    dashboard_Client.powerOff()
                    print("Shut down")         
                    time.sleep(5)       
                    dashboard_Client.powerOn()
                    print("Power On")
                    time.sleep(10)
                    try:   
                        dashboard_Client.connect(2000)
                        if dashboard_Client.isConnected():
                            print("dashboard_Client.isConnected()")
                            dashboard_Client.brakeRelease()
                            while (dashboard_Client.robotmode() != u'Robotmode: RUNNING'):
                                pass 
                            rospy.set_param("isEmergencyStopped","not_pressed")
                    except RuntimeError:
                        print("Waiting for Robot to turn on")
                else:
                    print("dashboard_Client.isConnected() False")
            except Exception as e: 
                rospy.loginfo("Error",e)

        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    start_robot()
