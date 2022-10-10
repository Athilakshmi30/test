#!/usr/bin/env python
from __future__ import print_function
from subprocess import Popen, PIPE
import os
import rospy
import time

sudo_password = 'axalta'
#command = 'systemctl stop bridgeconnect'.split()
#command1 = 'systemctl stop paintjobprocess'.split()
command2 = 'systemctl stop ccscore'.split()


command3 = 'systemctl start ccscore'.split()
#command4 = 'systemctl start paintjobprocess'.split()
#command5 = 'systemctl start bridgeconnect'.split()


shutdowncommand = "shutdown".split()



if __name__ == "__main__":
    try:
        rospy.init_node('exit_job')
        rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_LITE_TRIGGER',False)
        rospy.set_param('axalta/ccscore/dashboard/RESTART_JOB_DONE',False)
        #rate = rospy.Rate(20)
        while (not rospy.is_shutdown()):
            if(rospy.has_param('axalta/ccslite/CORE_SHUTDOWN_TRIGGER') and rospy.get_param('axalta/ccslite/CORE_SHUTDOWN_TRIGGER')):
                print("-----------SHUTTING DOWN CCSCORE-----------") 
                sp = Popen(['sudo','-S']+shutdowncommand,stdin=PIPE,stderr=PIPE,universal_newlines=True)
                sudo_prompt_shut = sp.communicate(sudo_password+'\n')[1]
                rospy.set_param("axalta/ccslite/CORE_SHUTDOWN_TRIGGER",False)
      
            if(rospy.has_param('axalta/ccscore/dashboard/EXIT_JOB_TRIGGER') and rospy.get_param('axalta/ccscore/dashboard/EXIT_JOB_TRIGGER')):
                rospy.set_param('axalta/ccscore/dashboard/RESTART_JOB_DONE',False)
                print("----------RESTARTING-----------")  
                time.sleep(2)
                #p = Popen(['sudo','-S']+command,stdin=PIPE,stderr=PIPE,universal_newlines=True)
                #sudo_prompt = p.communicate(sudo_password+'\n')[1]
                #print("---------------> stopped rosbridge .... ")
                #p1 = Popen(['sudo','-S']+command1,stdin=PIPE,stderr=PIPE,universal_newlines=True)
                #sudo_prompt1 = p1.communicate(sudo_password+'\n')[1]
                #print("---------------> stopped paintjobprocess .... ")

                p2 = Popen(['sudo','-S']+command2,stdin=PIPE,stderr=PIPE,universal_newlines=True)
                sudo_prompt2 = p2.communicate(sudo_password+'\n')[1]
                print("---------------> stopped ccscore .... ")
                #rospy.set_param("axalta/ccscore/dashboard/LIDARSTART",True)
                time.sleep(3)
                rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_DONE',False)
                p3 = Popen(['sudo','-S']+command3,stdin=PIPE,stderr=PIPE,universal_newlines=True)
                sudo_prompt3 = p3.communicate(sudo_password+'\n')[1]
                print("---------------> started ccscore .... ")
                #p4 = Popen(['sudo','-S']+command4,stdin=PIPE,stderr=PIPE,universal_newlines=True)
                #sudo_prompt4 = p4.communicate(sudo_password+'\n')[1]
                #print("---------------> started paintjobprocess .... ")
                #p5 = Popen(['sudo','-S']+command5,stdin=PIPE,stderr=PIPE,universal_newlines=True)
                #sudo_prompt5 = p5.communicate(sudo_password+'\n')[1]
                #print("---------------> started rosbridge .... ")
                rospy.set_param("axalta/ccscore/dashboard/RESTART_LIDARNODE_TRIGGER",True)
                time.sleep(6)
                rospy.set_param("axalta/ccscore/dashboard/RESTART_JOB_DONE",False)
                #rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_DONE',True)
                rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_TRIGGER',False)
                rospy.set_param('axalta/ccscore/dashboard/EXIT_JOB_LITE_TRIGGER',True)
            
            #rate.sleep()
        rospy.spin() 
    except rospy.ROSInterruptException:
        sys.exit(0)
    except Exception as e:
        print(e)

