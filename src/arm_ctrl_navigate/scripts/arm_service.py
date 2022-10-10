#!/usr/bin/env python

from arm_ctrl_navigate.msg import Plannedpath,PathStamped,Path
from arm_ctrl_navigate.srv import *
import rospy
import math
from tf import TransformListener
from geometry_msgs.msg import Pose,PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from dashboard.msg import *
from status_check.msg import stats
from mir.srv import *

from ccs_lite_communicate.srv import *
from ccs_lite_msgs.msg import CcsLiteCmd,EnclosureStatus,Door
import time

class ArmService():
    
    def __init__(self):
        self.flag = False
        self.flag1 = False
        self.flag2 = False
        self.flag3 = False
        self.flag4 = False
        print("reached_here")
        self.path = Path()
        self.path_base_coat_1 = Path()
        self.path_base_coat_2 = Path()
        self.path_clear_coat_1 = Path()
        self.path_clear_coat_2 = Path()
        self.transformed_path = Path()
        
        s = rospy.Service('path_service', PathMessage, self.handle_path)
        s1 = rospy.Service('path_ack', PathExecConfirm, self.handle_path_ack)
        s2 = rospy.Service('arm_paintgun_cmd', ArmPaintgunCmd, self.handle_arm_paintgun_cmd)
        s3 = rospy.Service('arm_startup', ArmStartup, self.handle_arm_startup)
        s4 = rospy.Service('arm_lock_situation', ArmLockSituation, self.handle_arm_lock_situation)
        s5 = rospy.Service('arm_homed_after_reset', ArmHomedAfterReset, self.handle_arm_homed_after_reset)
        
        rospy.Subscriber('trajectory_points_sealer',Path, self.path_callback)
        
        
        rospy.Subscriber('trajectory_points_base_coat_1',Path, self.path_base_coat_1_callback)
    
        rospy.Subscriber('trajectory_points_base_coat_2',Path, self.path_base_coat_2_callback)
        
        rospy.Subscriber('trajectory_points_clear_coat_1',Path, self.path_clear_coat_1_callback)
     
        rospy.Subscriber('trajectory_points_clear_coat_2',Path, self.path_clear_coat_2_callback) 
      
        rospy.Subscriber('coreenclosure', EnclosureStatus, self.enclosure_status_callback)
        
        self.arm_pub = rospy.Publisher('arm_status', stats, queue_size=10)
        self.st = stats()
        self.st.ready = 0
        self.st.running = 0 
        self.st.fatalerr = 0
        self.st.remarks = "ARM status" 
        self.count = 0
        self.current_paintgun_cmd = False
        self.lidar_door_action = True
        self.mir_door_action = True
        self.pathServiceFlag = False 
        self.enc_open = True
        self.prev_proc = "exit"
        #self.arm_startup = False
         
        print("reached_here 4") 
        rospy.set_param("axalta/ccscore/paintjobprocess/TRAJECTORY_PLANNED",False)
        rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE",55)
        rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Process Started")
        rospy.set_param("axalta/ccscore/arm_service/BASE_COAT_1",False)
        rospy.set_param("axalta/ccscore/arm_service/BASE_COAT_2",False)
        rospy.set_param("axalta/ccscore/arm_service/SEALER_COAT_1",False)
        rospy.set_param("axalta/ccscore/arm_service/SEALER_COAT_2",False)
        rospy.set_param("axalta/ccscore/arm_service/CLEAR_COAT_1",False)
        rospy.set_param("axalta/ccscore/arm_service/CLEAR_COAT_2",False)
        rospy.set_param('axalta/ccscore/arm_service/LAST_JOB',False)
        rospy.set_param('axalta/ccscore/arm_service/ACK_CALLED_SEALER',False)
        rospy.set_param('axalta/ccscore/arm_service/ACK_CALLED_BASE2',False)
        rospy.set_param('axalta/ccscore/arm_service/ACK_CALLED_CLEAR2',False)
        rospy.set_param('axalta/ccscore/arm_service/ARM_LOCK_SITUATION',False)
        rospy.set_param('axalta/ccscore/arm_service/ARM_LOCK',False)
        rospy.set_param("axalta/ccscore/ccs_lite_communicate/EMERGENCY_RESET",False)
        rospy.set_param("axalta/ccscore/arm_service/NEED_WAIT",False) 
        print("calling arm start function")
        self.arm_start_function()

    def path_callback(self,data):
        #print("in path callback")
        try:
            if(not self.flag):
                self.flag = True
                self.path = data
        except Exception as e:
            print("path1",e)        

    def path_base_coat_1_callback(self,data):
        #print("in path callback")
        try:
            if(not self.flag1):
                self.flag1 = True
                self.path_base_coat_1 = data
        except Exception as e:
            print("path2",e)        

    def path_base_coat_2_callback(self,data):
        #print("in path callback")
        try: 
            if(not self.flag2):
                self.flag2 = True
                self.path_base_coat_2 = data
        except Exception as e:
            print("path3",e)        

    def path_clear_coat_1_callback(self,data):
        #print("in path callback")
        try:
            if(not self.flag3):
                self.flag3 = True
                self.path_clear_coat_1 = data

        except Exception as e:
            print("path4",e)        

    def path_clear_coat_2_callback(self,data):
        #print("in path callback")
        try:
            if(not self.flag4):
                self.flag4 = True
                self.path_clear_coat_2 = data 
                rospy.set_param("axalta/ccscore/paintjobprocess/TRAJECTORY_PLANNED",True)    
        except Exception as e:
            print("path5",e)                   

    def enclosure_status_callback(self,data):
        try:
            self.enc_open = data.lidarDoor.doorAction and data.mirDoor.doorAction
            
        except Exception as e:
            print("core_enclosure",e)    


    def handle_path(self,req):
        try:
            
            rospy.set_param('axalta/ccscore/dashboard/PAINTING_DONE',False)
            rospy.loginfo("request received")
            #print(PathMessageResponse(self.transformed_path))
           
            
            while(rospy.get_param('axalta/ccscore/dashboard/PAINTJOBPROCESS')): 
                continue
            while(not rospy.get_param('axalta/ccscore/dashboard/GOTCONFIRMATION')):
                continue

            if(rospy.has_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS') and rospy.has_param('axalta/ccscore/dashboard/CURRENT_COAT_NUMBER')):
                    if(rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS') == 'sealer' and rospy.get_param('axalta/ccscore/dashboard/CURRENT_COAT_NUMBER') == 1):       
                    #arm.transformed_path=arm.apply_tf(arm.path)
                        self.pathServiceFlag = True
                        print("path for sealer coat 1")
                        #speed = 490.0
                        #speed = rospy.get_param('axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_sealercoat')
                        #delay = rospy.get_param('axalta/ccscore/dashboard/SPRAYGUN_DELAY_sealercoat')
                        speed = 490.0
                        delay = 1 
                        #print(self.apply_tf(self.path,speed,"sealer"))
                        print("completed")
                        return PathMessageResponse(self.apply_tf(self.path,speed,"sealer",delay))
                    
                    elif(rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS') == 'base' and rospy.get_param('axalta/ccscore/dashboard/CURRENT_COAT_NUMBER') == 1):
                        self.pathServiceFlag = True
                        print("path for coat 1 base")
                        #speed = 454.0
                        speed = rospy.get_param('axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_basecoat1')
                        delay = rospy.get_param('axalta/ccscore/dashboard/SPRAYGUN_DELAY_basecoat1')
                        return PathMessageResponse(self.apply_tf(self.path_base_coat_1,speed,"basecoat1",delay))

                    elif(rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS') == 'base' and rospy.get_param('axalta/ccscore/dashboard/CURRENT_COAT_NUMBER') == 2):
                        self.pathServiceFlag = True
                        print("path for coat 2 base")
                        #speed = 423.0
                        speed = rospy.get_param('axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_basecoat2')
                        delay = rospy.get_param('axalta/ccscore/dashboard/SPRAYGUN_DELAY_basecoat2')
                        return PathMessageResponse(self.apply_tf(self.path_base_coat_2,speed,"basecoat2",delay))
                        
                    elif(rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS') == 'clear' and rospy.get_param('axalta/ccscore/dashboard/CURRENT_COAT_NUMBER') == 1):
                        self.pathServiceFlag = True
                        print("path for coat 1 clear")
                        #speed = 390.0
                        speed = rospy.get_param('axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_clearcoat1')
                        delay = rospy.get_param('axalta/ccscore/dashboard/SPRAYGUN_DELAY_clearcoat1')
                        return PathMessageResponse(self.apply_tf(self.path_clear_coat_1,speed,"clearcoat1",delay))
                        
                    elif(rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS') == 'clear' and rospy.get_param('axalta/ccscore/dashboard/CURRENT_COAT_NUMBER') == 2):
                        self.pathServiceFlag = True
                        print("path for coat 2 clear")
                        #speed = 368.0
                        speed = rospy.get_param('axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_clearcoat2') 
                        delay = rospy.get_param('axalta/ccscore/dashboard/SPRAYGUN_DELAY_clearcoat2')
                        return PathMessageResponse(self.apply_tf(self.path_clear_coat_2,speed,"clearcoat2",delay))
                        
                    else:
                        self.pathServiceFlag = True
                        print("path if sealer 2 is triggered")
                        #speed = 490.0
                        speed = rospy.get_param('axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_sealercoat')
                        delay = rospy.get_param('axalta/ccscore/dashboard/SPRAYGUN_DELAY_sealercoat')
                        return PathMessageResponse(self.apply_tf(self.path,speed,"sealer"))
                        

            rospy.set_param("axalta/ccscore/arm_service/END2",rospy.get_time())   
            return PathMessageResponse()
            
            
        except:
            rospy.loginfo("Error occured")  

    def handle_path_ack(self,req):
        try:
            self.count = self.count+1
            out = False
            print("path_ack called")
            
            if(rospy.has_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS')):
                if (rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS') == "base"):
                    if(rospy.get_param('axalta/ccscore/dashboard/CURRENT_COAT_NUMBER')==2):
                        print("basecoat2 done")
                        
                        self.prev_proc = "base"
                        self.st.ready = 0
                        self.st.running = 0 
                        self.st.fatalerr = 0
                        self.st.remarks = "ARM status"
                        
                        rospy.set_param('axalta/ccscore/dashboard/GOTCONFIRMATION',False)
                        rospy.set_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS',"wait")
                        
                        self.lidar_door_action = True
                        self.mir_door_action = True
                        
                        rospy.set_param('axalta/ccscore/arm_service/ACK_CALLED_BASE2',True)
                        
                        #rospy.set_param('axalta/ccscore/arm_service/mir_pos_to_home_move_second',True)
                        self.pathServiceFlag = False
                        out = True
                        
                    else:    
                        print("basecoat1 done")
                        self.prev_proc = "base"
                        rospy.set_param("axalta/ccscore/arm_service/BASE_COAT_1",True)
                        rospy.set_param('axalta/ccscore/dashboard/GOTCONFIRMATION',False)
                        rospy.set_param("axalta/ccscore/dashboard/NEWJOB",False)
                        self.st.ready = 0
                        self.st.running = 0 
                        self.st.fatalerr = 0
                        self.st.remarks = "ARM status"
                        self.lidar_door_action = False
                        self.mir_door_action = False
                        
                        self.paint_gun_action = False
                        
                        self.pathServiceFlag = False
                        out = True
 
                elif (rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS') == "sealer"):
                    if(rospy.get_param('axalta/ccscore/dashboard/CURRENT_COAT_NUMBER')==2):
                        print("sealercoat2 done")
                        
                        self.prev_proc = "sealer"
                        self.st.ready = 0
                        self.st.running = 0 
                        self.st.fatalerr = 0
                        self.st.remarks = "ARM status"
                        self.lidar_door_action = True
                        self.mir_door_action = True
                        
                        rospy.set_param('axalta/ccscore/dashboard/GOTCONFIRMATION',False)
                        rospy.set_param('axalta/ccscore/arm_service/ACK_CALLED_SEALER',True)
                        
                        rospy.set_param('axalta/ccscore/arm_service/mir_pos_to_home_move_second',True)
                        rospy.set_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS',"wait")
                        self.pathServiceFlag = False
                        out = True
              
                    else:    
                        print("sealercoat1 done")
                        self.prev_proc = "sealer"
                        
                        rospy.set_param('axalta/ccscore/dashboard/GOTCONFIRMATION',False)
                        
                        self.st.ready = 0
                        self.st.running = 0 
                        self.st.fatalerr = 0
                        self.st.remarks = "ARM status"
                        rospy.set_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS',"wait")
                        self.paint_gun_action = False
                        self.lidar_door_action = True
                        self.mir_door_action = True
                         
                        rospy.set_param('axalta/ccscore/arm_service/ACK_CALLED_SEALER',True)
                        rospy.set_param('axalta/ccscore/arm_service/mir_pos_to_home_move_first',True)
                        self.pathServiceFlag = False
                        out = True
                     
                elif (rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS') == "clear"):
                    if(rospy.get_param('axalta/ccscore/dashboard/CURRENT_COAT_NUMBER')==2):
                        print("clearcoat2 done")
                        
                        self.prev_proc="clear"#
                        self.st.ready = 0
                        self.st.running = 0 
                        self.st.fatalerr = 0
                        self.st.remarks = "ARM status"
                        rospy.set_param('axalta/ccscore/dashboard/GOTCONFIRMATION',False)
                        rospy.set_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS',"wait")
                        
                        self.lidar_door_action = True
                        self.mir_door_action = True
                        
                        
                        rospy.set_param('axalta/ccscore/arm_service/ACK_CALLED_CLEAR2',True)
                        
                        rospy.set_param('axalta/ccscore/arm_service/mir_pos_to_home_move_finish',True)
                        rospy.set_param('axalta/ccscore/arm_service/LAST_JOB',True)
                        
                        self.pathServiceFlag = False
                        out = True

                    else:    
                        print("clearcoat1 done")
                        self.prev_proc="clear"
                        rospy.set_param("axalta/ccscore/arm_service/CLEAR_COAT_1",True)
                        rospy.set_param('axalta/ccscore/dashboard/GOTCONFIRMATION',False)
                        self.st.ready = 0
                        self.st.running = 0 
                        self.st.fatalerr = 0
                        self.st.remarks = "ARM status"
                        
                        self.paint_gun_action = False
                        self.lidar_door_action = False
                        self.mir_door_action = False
                        
                        self.pathServiceFlag = False
                        out = True

                

            #if(self.count >= 5):
                #rospy.set_param('axalta/ccscore/dashboard/PAINTING_DONE',True)
                
            print("output: ",out) 
            return PathExecConfirmResponse(out)    
        except:

            rospy.loginfo("Error occured")
            return PathExecConfirmResponse(False)
            

    def handle_arm_paintgun_cmd(self,req):
        try:
            print("----------------------im in arm paint gun command server---------------")
            self.current_paintgun_cmd = req.cmd
            print(self.current_paintgun_cmd)
            return ArmPaintgunCmdResponse(True)
        except:
            return ArmPaintgunCmdResponse(False)
            rospy.loginfo("Error occured")    
    
    def handle_arm_startup(self,req):
        try:
            print("----------------------im in arm startup server---------------")
            #self.arm_startup = True
            rospy.set_param('axalta/ccscore/arm_service/ARM_STARTUP',True)
            return ArmStartupResponse(True)
        except:
            return ArmStartupResponse(False)
            rospy.loginfo("Error occured")

    def handle_arm_lock_situation(self,req):  
        try:
            print("----------------------im in arm lock situation server---------------")
            
            rospy.set_param('axalta/ccscore/arm_service/ARM_LOCK_SITUATION',True)
            rospy.set_param('axalta/ccscore/arm_service/ARM_LOCK',True)
            return ArmLockSituationResponse(True)
        except Exception as e:
            print("service call failed: %s" %e)   
            return ArmLockSituationResponse(False) 
    
    def handle_arm_homed_after_reset(self,req):
        try:
            print("----------------------im in arm homed after reset server---------------")
            rospy.set_param('axalta/ccscore/arm_service/ARM_HOMED_AFTER_RESET',True)
            if(rospy.has_param('axalta/ccscore/arm_service/ARM_LOCK_SITUATION') and rospy.get_param('axalta/ccscore/arm_service/ARM_LOCK_SITUATION')):
                rospy.set_param('axalta/ccscore/arm_service/ARM_HOMED_AFTER_RESET',True)
                cmdd = CcsLiteCmd()
                cmdd.lidar_door_action = True
                cmdd.mir_door_action = True
                cmdd.paint_gun_action = False

                self.ccs_lite_command_client(cmdd)
                rospy.set_param("axalta/ccscore/arm_service/NEED_WAIT",True)
            else:
                rospy.set_param("axalta/ccscore/arm_service/NEED_WAIT",False) 
                pass    
            return ArmHomedAfterResetResponse(True)
        except Exception as e:
            print("service call failed: %s" %e)
            return ArmHomedAfterResetResponse(False)

    def ccs_lite_command_client(self,command):
        #print("----------------------im in ccs_lite_cmd server---------------") 
        rospy.wait_for_service('ccs_lite_command')
        try:
            ccs_lite_command = rospy.ServiceProxy(
                'ccs_lite_command', CcsLiteCommand)
            resp1 = ccs_lite_command(command)
            #print(resp1)
            return True
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)   
    
      

    def apply_tf(self,path,speed,coat,delay):
        tf_listener_ = TransformListener()
        transformed_path = Path()
        v = [] 
        output_path = Path()
        tf_listener_.waitForTransform('lidar_centric','arm_base_centric',rospy.Time(),rospy.Duration(100.0))
        #print("--------- both frames OK----------------") 
        it = 0        
        for count,path_msg in enumerate(path.path):
            row = PathStamped() 
            m = []
            for cnt,pose in enumerate(path_msg.path_msg):
                
                new_point = Plannedpath()
                #print("lidar_centric: ",pose)
                pt = PoseStamped()
                #qt = quaternion_from_euler(pose.u*math.pi/180.0, pose.v*math.pi/180.0,pose.w*math.pi/180.0)
                qt = quaternion_from_euler(pose.u, pose.v,pose.w)   
                pt.header.frame_id = "lidar_centric"
                pt.header.stamp =rospy.Time(0)
                pt.pose.position.x = pose.x
                pt.pose.position.y = pose.y
                pt.pose.position.z = pose.z
                
                pt.pose.orientation.x = qt[0]
                pt.pose.orientation.y = qt[1]
                pt.pose.orientation.z = qt[2]
                pt.pose.orientation.w = qt[3]
                p_in_arm_base = tf_listener_.transformPose("/arm_base_centric", pt)

                #if(pose.is_index):
                       
                #print("arm_base_centric",p_in_arm_base)
                new_point.x = p_in_arm_base.pose.position.x*1000
                new_point.y = p_in_arm_base.pose.position.y*1000
                new_point.z = p_in_arm_base.pose.position.z*1000
                eu = euler_from_quaternion([p_in_arm_base.pose.orientation.x,p_in_arm_base.pose.orientation.y,p_in_arm_base.pose.orientation.z,p_in_arm_base.pose.orientation.w])
                new_point.u = eu[0]*180.0/math.pi
                new_point.v = eu[1]*180.0/math.pi
                new_point.w = eu[2]*180.0/math.pi
                new_point.is_index = pose.is_index
                #new_point.is_index = 
                row.path_msg.append(new_point)
                if(it%2 == 0):
                    m.append(eu[1]*180.0/math.pi)
     
            transformed_path.path.append(row)
            v.append(m)
        transformed_path.CoatSpeed = speed
        transformed_path.CoatName = coat
        transformed_path.IndexDelay = int(delay*1000)

        for count,path_row in enumerate(transformed_path.path):
            row = PathStamped()
            row_v = sum(v[count])/len(v[count]) 
            for cnt,pose_in in enumerate(path_row.path_msg):
                new_point = Plannedpath() 
                new_point.x = pose_in.x
                new_point.y = pose_in.y
                new_point.z = pose_in.z

                new_point.u = 0
                new_point.v = row_v
                new_point.w = pose_in.w

                #if(new_point.w<0.5 and new_point.w>-0.5):
                #    new_point.w = 0 
                row.path_msg.append(new_point)

            output_path.path.append(row)
        output_path.CoatSpeed = speed
        output_path.CoatName = coat
        output_path.IndexDelay = int(delay*1000)
        print(output_path)   
        return output_path    
            
    def arm_start_function(self):
        try: 
            rate = rospy.Rate(100)
            print("In arm start function")
            waiting_for_reset = False
            
            cmdd = CcsLiteCmd()
            while not rospy.is_shutdown():
                #if(waiting_for_reset):
                
                if((rospy.has_param('axalta/ccscore/ccs_lite_communicate_EMERGENCY') and not rospy.get_param('axalta/ccscore/ccs_lite_communicate_EMERGENCY')) and (rospy.has_param("axalta/ccscore/ccs_lite_communicate/EMERGENCY_RESET") and not rospy.get_param("axalta/ccscore/ccs_lite_communicate/EMERGENCY_RESET"))):
                    #and (rospy.has_param('axalta/ccscore/arm_service/ARM_LOCK') and not rospy.get_param('axalta/ccscore/arm_service/ARM_LOCK'))
                    #print("Normal Flow")
                    
                    cmdd.paint_gun_action = self.current_paintgun_cmd
                        
                    if(rospy.has_param('axalta/ccscore/dashboard/GOTCONFIRMATION') and rospy.get_param('axalta/ccscore/dashboard/GOTCONFIRMATION') and rospy.has_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS') and (rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS')!= 'exit') and (rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS')!= 'wait')):
                        self.lidar_door_action = False
                        self.mir_door_action = False

                    cmdd.lidar_door_action = self.lidar_door_action
                    cmdd.mir_door_action = self.mir_door_action
                    if(self.prev_proc == "exit" and rospy.has_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS') and rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS')!= 'exit'):
                        cmdd.lidar_door_action = False
                        cmdd.mir_door_action = False
                    if(rospy.has_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS') and (rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS')!= 'exit')  and (rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS')!= 'wait') and rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS') == self.prev_proc):
                        cmdd.lidar_door_action = False
                        cmdd.mir_door_action = False
                    if(rospy.has_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS') and (rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS') == 'exit') and self.prev_proc != "exit"):
                        rospy.set_param('axalta/ccscore/dashboard/PAINTING_DONE',False)
                        rospy.set_param('axalta/ccscore/dashboard/SURFACE_MAPPING_DONE',False)
                        rospy.set_param('axalta/ccscore/dashboard/OBJECT_IDENTIFICATION_DONE',False)
                        rospy.set_param("axalta/ccscore/arm_service/BASE_COAT_1",False)
                        rospy.set_param("axalta/ccscore/arm_service/BASE_COAT_2",False)
                        rospy.set_param("axalta/ccscore/arm_service/SEALER_COAT_1",False)
                        rospy.set_param("axalta/ccscore/arm_service/SEALER_COAT_2",False)
                        rospy.set_param("axalta/ccscore/arm_service/CLEAR_COAT_1",False)
                        rospy.set_param("axalta/ccscore/arm_service/CLEAR_COAT_2",False)
                        #rospy.set_param('axalta/ccscore/dashboard/CURRENT_COAT_NUMBER',0)
                        #rospy.set_param('axalta/ccscore/dashboard/GOTCONFIRMATION',False)
                        #rospy.set_param('axalta/ccscore/dashboard/PAINTJOBPROCESS',False)
                        self.pathServiceFlag = False
                        
                    if(self.flag and self.flag1 and self.flag2 and self.flag3 and self.flag4 and not self.pathServiceFlag):
                        print("all paths received and path service hasnt been called yet")
                        if(rospy.has_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS') and (rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS')!= 'exit') and (rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS')!= 'wait') and rospy.get_param('axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS')== self.prev_proc):
                            
                            cmdd.lidar_door_action = False
                            cmdd.mir_door_action = False
                                
                        else: 
                            self.st.ready = 0
                            self.st.running = 0 
                            self.st.fatalerr = 0
                            self.st.remarks = "ARM status"   
                        
                            #check for all coats  
                            
                        if(rospy.has_param('axalta/ccscore/dashboard/GOTCONFIRMATION') and rospy.get_param('axalta/ccscore/dashboard/GOTCONFIRMATION') and not self.enc_open):
                            self.st.ready = 1
                            self.st.running = 1 
                            self.st.fatalerr = 0
                            self.st.remarks = "ARM status"
                            print("arm_ready is true")   

                        rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE",100)
                        rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Completed Trajectoryplanning")
                    self.arm_pub.publish(self.st)
                    self.ccs_lite_command_client(cmdd)
                else:
                            
                    print("In emergency_state")
                    #waiting_for_reset = True  
                rate.sleep()    
        except Exception as e: 
            print(e)                        

if __name__ == "__main__":
    try:
        rospy.init_node('arm_service')
        a = ArmService()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(e)
