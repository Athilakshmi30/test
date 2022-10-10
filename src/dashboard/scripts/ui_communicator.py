#!/usr/bin/env python

from tkinter import Image
import rospy
from std_msgs.msg import String
from status_check.msg import *
from dashboard.msg import Jobs
from status_check.srv import *
from sensor_msgs.msg import Image

status = Status_data()
global job_prog
job_prog = Jobs()
map_image_ui = String()

js = ""

def UI_send():
    print("Inside UI send")
    #pub = rospy.Publisher('UI_data', String, queue_size=10)
    stat_pub = rospy.Publisher('core_status',Status_data,queue_size=10)
    joystick_pub = rospy.Publisher('joystick_command',String,queue_size=3)
    joystick_pub_act = rospy.Publisher('joystick_cmd',String,queue_size=3)
    progress_pub = rospy.Publisher('job_progress',Jobs,queue_size=10)
    mir_map_img_pub = rospy.Publisher('map_mir', String,queue_size=10)
    rate = rospy.Rate(1) # 100hz
    global js
    while not rospy.is_shutdown():
       # rospy.sleep(1)
        #print("Im in")
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #print(js)
        #pub.publish(hello_str)
        stat_pub.publish(status)
        # rospy.getParamCached()
        try:
            if(rospy.has_param("axalta/ccscore/dashboard/MANUAL_MODE")):
                #print("polling parametrer server")
                m = rospy.get_param("axalta/ccscore/dashboard/MANUAL_MODE")
                if(m):
                    joystick_pub.publish(js)
                    joystick_pub_act.publish(js)
                
                    js = "" 
                else:
                    js = ""
        except:
            print("Inside Try Cache exception set to 10")
            #rospy.sleep(10)     
        progress_pub.publish(job_prog)
        mir_map_img_pub.publish(map_image_ui)
        rate.sleep()
    rospy.spin()



def status_check_data_callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
   # print("IM inside callback")
    global status
    status = data

def joystick_data_callback(data):
   # if(rospy.get_param('axalta/ccscore/dashboard/MANUAL_MODE')):
        global js
        js = data
    
#def report_data_callback(data):
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


    
# def status_update():
#     sts = get_status_client()

#     val = Jobs()
#     if(rospy.has_param('axalta/ccscore/dashboard/OBJECT_IDENTIFICATION_DONE')):
#         val.ObjectIdentificationDone = rospy.get_param('axalta/ccscore/dashboard/OBJECT_IDENTIFICATION_DONE')
#     else:
#         val.ObjectIdentificationDone = False    
#     if(rospy.has_param('axalta/ccscore/dashboard/SURFACE_MAPPING_DONE')):
#         val.SurfaceMappingDone = rospy.get_param('axalta/ccscore/dashboard/SURFACE_MAPPING_DONE')    
#     else:
#         val.SurfaceMappingDone = False
        
#     if(rospy.has_param('axalta/ccscore/paintjobprocess/TRAJECTORY_PLANNED')):
#         val.TrajectoryPlanningDone = rospy.get_param('axalta/ccscore/paintjobprocess/TRAJECTORY_PLANNED')    
#     else:
#         val.TrajectoryPlanningDone = False
            
#     if(rospy.has_param('axalta/ccscore/dashboard/PAINTING_DONE')):
#         val.PaintingDone = rospy.get_param('axalta/ccscore/dashboard/PAINTING_DONE')
#     else:
#         val.PaintingDone = False

#     if(rospy.has_param('axalta/ccscore/dashboard/RESTART_JOB_DONE')):
#         val.RestartJobDone = rospy.get_param('axalta/ccscore/dashboard/RESTART_JOB_DONE')
#     else:
#         val.RestartJobDone = False

#     if(rospy.has_param('axalta/ccscore/dashboard/EXIT_JOB_DONE')): 
#         if(rospy.get_param('axalta/ccscore/dashboard/EXIT_JOB_DONE')):   
#             val.ExitJobDone = True
#         else:
#             val.ExitJobDone = False   
            
#     if(rospy.has_param('axalta/ccscore/dashboard/IMAGE_STITCHING_DONE')): 
#         val.ImageStitchingDone = rospy.get_param('axalta/ccscore/dashboard/IMAGE_STITCHING_DONE')  
#     else:
#         val.ImageStitchingDone = False
            
            
#     if(rospy.has_param('axalta/ccscore/dashboard/IMAGE_SEGMENTATION_DONE')): 
#         val.ImageSegmentationDone = rospy.get_param('axalta/ccscore/dashboard/IMAGE_SEGMENTATION_DONE')  
#     else:
#         val.ImageSegmentationDone = False
            
            
#     if(rospy.has_param('axalta/ccscore/dashboard/FEASIBILITY_CHECK_DONE')): 
#         val.FeasibilityCheckDone = rospy.get_param('axalta/ccscore/dashboard/FEASIBILITY_CHECK_DONE')  
#     else:
#         val.FeasibilityCheckDone = False 
    
#     if(rospy.has_param("axalta/ccscore/dashboard/IsManuallyCropped")):
#          val.isManuallyCropped = rospy.get_param("axalta/ccscore/dashboard/IsManuallyCropped")
#     else:
#          val.isManuallyCropped = False
    
#     if(rospy.get_param("axalta/ccscore/dashboard/FeasibilityCheckDone")):
#          val.BaseCoat1.isFeasible = True
#          val.BaseCoat2.isFeasible = True
#          val.SealerCoat.isFeasible = True
#          val.ClearCoat1.isFeasible = True
#          val.ClearCoat2.isFeasible = True
#     else:    
#          val.BaseCoat1.isFeasible = False            
#          val.BaseCoat2.isFeasible = False            
#          val.SealerCoat.isFeasible = False          
#          val.ClearCoat1.isFeasible = False             
#          val.ClearCoat2.isFeasible = False
    
    
    
#    # val.BaseCoat1.isCompleted = sts.current_process_status.base_coat1.process_completed
#    # val.BaseCoat2.isCompleted = sts.current_process_status.base_coat2.process_completed
#    # val.SealerCoat.isCompleted = sts.current_process_status.sealer_coat.process_completed
#    # val.ClearCoat1.isCompleted = sts.current_process_status.clear_coat1.process_completed
#    # val.ClearCoat2.isCompleted = sts.current_process_status.clear_coat2.process_completed
#     return val

# def get_status_client():    
#     rospy.wait_for_service('get_process_status')
#     try:
#         sts = rospy.ServiceProxy(
#             'get_process_status', GetProcessStatus)
#         a = GetProcessStatusRequest()    
#         progress_status = sts(a)
#         return progress_status
#     except rospy.ServiceException as e:
#         print("get status Service call failed: %s" % e)

def job_progress_callback(data):
    global job_prog
    job_prog = data

def map_mir_callback(data):
    global map_image_ui
    map_image_ui = data

def get_UI_data():
   
    
    rospy.Subscriber("status_check_from_main", Status_data, status_check_data_callback)
    #rospy.Subscriber("report_data", String, report_data_callback)
    rospy.Subscriber("core_joystick_from_main", String, joystick_data_callback)
    rospy.Subscriber("job_progress_from_main", Jobs, job_progress_callback)
    rospy.Subscriber("/main_package/map_image_to_ui", String, map_mir_callback)
    
    

if __name__ == '__main__':
    try:
        rospy.init_node('ui_communicator', anonymous=True)
        get_UI_data()
        UI_send()
        
    except rospy.ROSInterruptException:
        pass


