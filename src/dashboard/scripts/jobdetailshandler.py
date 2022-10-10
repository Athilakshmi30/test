#!/usr/bin/env python
from __future__ import print_function
from concurrent.futures import thread
import threading
from dashboard.srv import *
from dashboard.msg import *
import rospy
from mir.srv import *
import time


class JobDetailsHandler():

    def __init__(self):
        self.jobID = '1'
        

    def set_robot_operating_mode(self, mode):

        try:

            if mode == 1:  # autonomous
                rospy.set_param('axalta/ccscore/dashboard/MANUAL_MODE', False)
                rospy.set_param(
                    'axalta/ccscore/dashboard/AUTONOMOUS_MODE', True)
            elif mode == 0:  # manual
                rospy.set_param('axalta/ccscore/dashboard/MANUAL_MODE', True)
                rospy.set_param(
                    'axalta/ccscore/dashboard/AUTONOMOUS_MODE', False)
            return True
        except Exception as e:
            print("Error : ", e)
            return False

    def set_robot_to_manualmode(self):
        print("set_robot_to_manualmode()")
        rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","please use joystick to move the robot")
        status = self.set_robot_operating_mode(0)  # 1 = autonomous, 0= manual
        print("response :", status)
        #return JobDetailsResponse(status)
        return status
                
    def move_to_target(self, destination):
        mir_resp = False
        if(destination == "leftfront"):
            mir_resp = self.move_to_destination_client("leftfront")
            rospy.set_param(
            'axalta/ccscore/dashboard/MIR_PAINTBOOTH_LOCATION', "LeftFront")
            print("target reached")
            return mir_resp
        elif(destination == "leftback"):
            mir_resp = self.move_to_destination_client("LeftBack")
            rospy.set_param(
            'axalta/ccscore/dashboard/MIR_PAINTBOOTH_LOCATION', "LeftBack")
            print("target reached")
            return mir_resp
        elif(destination == "rightfront"):
            mir_resp = self.move_to_destination_client("RightFront")
            rospy.set_param(
            'axalta/ccscore/dashboard/MIR_PAINTBOOTH_LOCATION', "RightFront")
            print("target reached")
            return mir_resp
        elif(destination == "rightback"):
            mir_resp = self.move_to_destination_client("RightBack")
            rospy.set_param(
            'axalta/ccscore/dashboard/MIR_PAINTBOOTH_LOCATION', "RightBack")
            print("target reached")
            return mir_resp
                
        else:
            # add other cases in future or use switch case
            return False

    def init_paint_process_params(self, destination):
        #rospy.set_param(
        #    'axalta/ccscore/dashboard/MIR_PAINTBOOTH_LOCATION', destination)
        rospy.set_param(
            'axalta/ccscore/dashboard/JOBID', self.jobID)
        # rospy.set_param("axalta/ccscore/dashboard/NEWJOB", True)
        rospy.set_param(
            "axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 50)
        self.jobID = str(int(self.jobID) + 1)
        rospy.set_param(
            'axalta/ccscore/dashboard/PAINTING_DONE', False)

    def robot_auto_functions(self, destination):
        try:
            print('--Inside Autonomous--')
            # setting robot to auto mode
            self.set_robot_operating_mode(1)  # 1 = autonomous, 0= manual
            # moving robot to destination
            mir_resp = self.move_to_target(destination)
            #mir_resp = True  
            if(mir_resp):
                rospy.set_param("axalta/ccscore/dashboard/MIRTargetPositionCheck",True)
                # safety delay until vibrations die out to enable LIDAR take better pointclouds
                time.sleep(2)
                # initializing parameters for painting process
                self.init_paint_process_params(destination)
                print('response:', True)
                return True
            else:
                print('response:', False)
                return False
        except Exception as e:
            print("exception message :", e)
            return False

    def move_to_destination_client(self, destination):
        rospy.wait_for_service('move_to_destination')
        print("move_to_destination_client(), destination received :", destination)
        try:

            move_to_destination = rospy.ServiceProxy(
                'move_to_destination', MoveToDestination)
            resp1 = move_to_destination(destination)

            print(resp1)
            retry_count = 0
            while((str(resp1.status).lower().strip() != "completed") and retry_count < 3):
                resp1 = move_to_destination(destination)
                print(resp1)
                retry_count = retry_count + 1
            if(retry_count >= 3):
                return False
            else:
                rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Robot has reached destination")
                rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE",100)
                return True
            # return True
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def function_job_details(self,op_mode,mir_paint_location):
        if(op_mode.lower() == 'manual'):
                return self.set_robot_to_manualmode()

            #  robot  autonomous mode operations
        elif(op_mode.lower() == 'autonomous'):
                destination = str(mir_paint_location).lower().replace(" ","")
                print("destination:",destination)
                status = self.robot_auto_functions(destination)
                print("response :",status)

    
    def handle_job_details(self, req):
        try:
            print('request:', req)
            print('time:', time.time())
            rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","New job has started")
            rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 50)
            #  robot manual mode operation
            op_mode = req.operating_mode
            mir_paint_location = req.mir_paintbooth_location
            thread_job_details = threading.Thread(target=self.function_job_details,args=(op_mode,mir_paint_location))
            thread_job_details.start()
            # if(op_mode.lower() == 'manual'):
            #     return self.set_robot_to_manualmode()

            # #  robot  autonomous mode operations
            # elif(op_mode.lower() == 'autonomous'):
            #     destination = str(mir_paint_location).lower().replace(" ","")
            #     status = self.robot_auto_functions(destination)
            #     print("response :",status)
            #     return JobDetailsResponse(status)
            return True

        except Exception as e:
            print(e, "has occurred")
           # return JobDetailsResponse(False)
            return False
            
            
if __name__ == '__main__':
    rospy.init_node('JobDetails_Handler')
    JobDetailsHandler()
    rospy.spin()
