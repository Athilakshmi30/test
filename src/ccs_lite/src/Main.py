#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def tempHumFunction(temp_data):
    rospy.loginfo("I heard %s", temp_data.data)


def sprayFunction(spray_data):
    rospy.loginfo("I heard %s", spray_data.data)


def encFunction(enc_data):
    rospy.loginfo("I heard %s", enc_data.data)


def joystickFunction(joystick_data):
    rospy.loginfo("I heard %s", joystick_data.data)


def surfaceResponseFunction(sr_data):
    rospy.loginfo("I heard %s", sr_data.data)


def gpsFunction(gps_data):
    rospy.loginfo("I heard %s", gps_data.data)


def powerFunction(power_data):
    rospy.loginfo("I heard %s", power_data.data)


def estopFunction(estop_data):
    rospy.loginfo("I heard %s", estop_data.data)


def ccsCmdFunction(ccs_data):
    rospy.loginfo("I heard %s", ccs_data.data)
    surfaceRequest = "Surface Measurement Request"
    sprayCmd = "Spray command"
    enclosureCmd = "Enclosure command"
    ccs_data = "CCS lite data"

    pub1.publish(surfaceRequest)
    pub2.publish(sprayCmd)
    pub3.publish(enclosureCmds)
    pub4.publish(ccs_data)


def listener():
    rospy.init_node('ccs_lite', anonymous=False)
    pub1 = rospy.Publisher('/surface_measurement_request', String, queue_size=10)
    pub2 = rospy.Publisher('/spray_cmd', String, queue_size=10)
    pub3 = rospy.Publisher('/enc_cmd', String, queue_size=10)
    pub4 = rospy.Publisher('/ccs_lite_data', String, queue_size=10)
    rospy.Subscriber("/temp_hum_data", String, tempHumFunction)
    rospy.Subscriber("/spray_stat", String, sprayFunction)
    rospy.Subscriber("/enc_stat", String, encFunction)
    rospy.Subscriber("/joystick_data", String, joystickFunction)
    rospy.Subscriber("/surface_measurement_response", String, surfaceResponseFunction)  
    rospy.Subscriber("/gps_data", String, gpsFunction)
    rospy.Subscriber("/power_mgmt_data", String, powerFunction)
    rospy.Subscriber("/estop_cmd", String, estopFunction)  
    rospy.Subscriber("/ccs_lite_cmd", String, ccsCmdFunction)
    rospy.spin()


if __name__ == '__main__':
    listener()
