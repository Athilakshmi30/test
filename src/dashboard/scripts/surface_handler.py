#!/usr/bin/env python
from __future__ import print_function
from dashboard.srv import *
from dashboard.msg import *
import rospy
#from report.msg import Report
from mir.srv import *
from std_msgs.msg import *
import base64
#from jobdetailshandler import JobDetailsHandler
#from main_package.main_call import call_modules
from main_package.srv import MainTrigger
from ccs_lite_communicate.srv import *
from ccs_lite_msgs.msg import *

painting_done = False
cmd_ = CcsLiteCmd()

class SurfaceHandler():
    def __init__(self):
        pass

    """def handle_surfaceMapping(self, req):
        try:
            print(
                "----------------surface mapping image service------------------- ")
            print("request: ", req)
            objectArea = SurfaceMappingImageResponse()
            with open("/home/axalta/axalta_ws/img/image_to_ui.png", "rb") as img_file:
                objarea_string = base64.b64encode(img_file.read())
            objectArea.image_data = objarea_string
            points = ContourPoints()
            pts = ImageCoord()
            pts1 = ImageCoord()
            pts2 = ImageCoord()
            pts3 = ImageCoord()
            pts.x = 1198
            pts.y = 43
            pts1.x = 1090
            pts1.y = 42
            pts2.x = 1093
            pts2.y = 49
            pts3.x = 1190
            pts3.y = 46
            points.point_list.append(pts)
            points.point_list.append(pts1)
            points.point_list.append(pts2)
            points.point_list.append(pts3)
            objectArea.boundary = points
            print(points)
            rospy.set_param(
                'axalta/ccscore/dashboard/OBJECT_IDENTIFICATION_DONE', True)
            return objectArea
        except Exception as e:
            print(e, "has occurred")"""

    def surface_confirm_set_params(self, mir_resp, paint_proc, coat_num):
        if(mir_resp or coat_num == 2):
            rospy.set_param(
                'axalta/ccscore/dashboard/GOTCONFIRMATION', True)
            rospy.set_param(
                'axalta/ccscore/dashboard/CURRENT_COAT_NUMBER', coat_num)
            rospy.set_param(
                'axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS', paint_proc)
            rospy.set_param(
                'axalta/ccscore/dashboard/SURFACE_MAPPING_DONE', True)
            return True
        else:
            return False

    # def initiate_first_coats(self, paint_proc):
    #     mir_resp = False
    #     mir_move_job_details_handler = JobDetailsHandler()
    #     if(paint_proc == "base" or paint_proc == "clear"):
    #         mir_resp = mir_move_job_details_handler.move_to_destination_client(str(rospy.get_param('axalta/ccscore/dashboard/MIR_PAINTBOOTH_LOCATION')))
    #     else:
    #         mir_resp=True
    #     if(paint_proc == "base"):
    #         rospy.set_param(
    #             'axalta/ccscore/dashboard/mir_home_to_pos_move_second', True)
    #     elif(paint_proc == "clear"):
    #         rospy.set_param(
    #             'axalta/ccscore/dashboard/mir_home_to_pos_move_clear', True)
    #     return mir_resp

    def exit_current_job(self):

        rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_DONE', True)
        rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_TRIGGER', True)
        rospy.set_param('axalta/ccscore/dashboard/RESET_JOB_TRIGGER', True)

        #rospy.set_param("axalta/ccscore/dashboard/LIDARSTART", True)

    def initiate_painting(self, paint_proc, coat_num):
        #since we are adding seperate service for exit,commenting this lines
        #if(paint_proc == "exit"):
        #    self.exit_current_job()
        #    return True
        result = False
        print("inside init_painting")
        rospy.wait_for_service('main_package/Main_Trigger_server')

        start_painting = rospy.ServiceProxy('main_package/Main_Trigger_server', MainTrigger)
        if(paint_proc == "Sealer"):
            st_paint = "sealer"
            result = start_painting(st_paint)

        elif(paint_proc == "Base" and coat_num == 1):
            print("inside base1")
            st_paint = "base1"
            result = start_painting(st_paint)

        elif(paint_proc == "Base" and coat_num == 2):
            st_paint = "base2"
            result = start_painting(st_paint)

        elif(paint_proc == "Clear" and coat_num == 1):
            st_paint = "clear1"
            result = start_painting(st_paint)

        elif(paint_proc == "Clear" and coat_num == 2):
            st_paint = "clear2"
            result = start_painting(st_paint)

        # set_status=self.surface_confirm_set_params(
        #     mir_resp, paint_proc, coat_num)
        # print(result.response)
        # return result.response
        return result.response

    def handle_exit_job(self, req):
        try:
           print("----------------Exit_Job service------------------- ")
          # rospy.Subscriber("painting_done_check", Bool,painting_done_callback)
          # if not rospy.get_param("axalta/ccscore/dashboard/PAINTING_DONE"):
          #          rospy.set_param("axalta/ccscore/dashboard/PAINTING_DONE",True)
           #if not painting_done:
                   
           self.exit_current_job()   
           return True       
           
        except Exception as e:
            print(e, "has occurred")  
            return False
    
    
    
    
    def handle_surface_selected(self, req):
        try:
            print("im going")
            print("------------------surface confirm----------------------")

            surface_confirmation=self.initiate_painting(req.paint_process, req.coat_number)
            print(type(surface_confirmation))

            print("response: ", surface_confirmation)

            return StartPaintingResponse(surface_confirmation)

        except Exception as e:
            #print(req)
            print(e, "has occurred")
            print("response: ", False)
            return StartPaintingResponse(False)
            
         
def painting_done_callback(req):
        painting_done = req.data
        
