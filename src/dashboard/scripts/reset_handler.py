#!/usr/bin/env python


import rospy

def reset_params():
    rospy.set_param("WSL/Windows/LAB", [1111])
    rospy.set_param("WSL/Windows/LAB_SET", False)
    rospy.set_param("Windows/Reconstruction_check", False)
    rospy.set_param("Windows/Segmentation_check", False)
    rospy.set_param("axalta/ccscore/dashboard/AUTONOMOUS_MODE", False)
    rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE", 0)
    rospy.set_param("axalta/ccscore/dashboard/ChargingStation", "home")
    rospy.set_param("axalta/ccscore/dashboard/EXIT_JOB_DONE", False)
    rospy.set_param("axalta/ccscore/dashboard/EXIT_JOB_TRIGGER", False)
    rospy.set_param("axalta/ccscore/dashboard/GOTCONFIRMATION", False)
    rospy.set_param("axalta/ccscore/dashboard/HSV_LOWER", '[40,114,125]')
    rospy.set_param("axalta/ccscore/dashboard/HSV_UPPER", '[43,220,241]')
    rospy.set_param("axalta/ccscore/dashboard/Home", "home")
    rospy.set_param("axalta/ccscore/dashboard/IMAGE_SEGMENTATION_DONE", False)
    rospy.set_param("axalta/ccscore/dashboard/IMAGE_STITCHING_DONE", False)
    rospy.set_param("axalta/ccscore/dashboard/IsManuallyCropped", False)
    rospy.set_param("axalta/ccscore/dashboard/LAB_VALUE", '[0,0,0]')
    rospy.set_param("axalta/ccscore/dashboard/LeftBack", "target")
    rospy.set_param("axalta/ccscore/dashboard/LeftFront", "target")
    rospy.set_param("axalta/ccscore/dashboard/MANUAL_MODE", False)
    rospy.set_param("axalta/ccscore/dashboard/MIRTargetPositionCheck", False)
    rospy.set_param("axalta/ccscore/dashboard/PointCloudGenerated", False)
    rospy.set_param("axalta/ccscore/dashboard/PAINTING_DONE", False)
    rospy.set_param("axalta/ccscore/dashboard/PAINTJOBPROCESS", False)
    rospy.set_param("axalta/ccscore/dashboard/REQUIRED_PAINT_PROCESS", "exit")
    rospy.set_param("axalta/ccscore/dashboard/RESTART_JOB_DONE", False)
    rospy.set_param("axalta/ccscore/dashboard/RightBack", "target")
    rospy.set_param("axalta/ccscore/dashboard/RightFront", "target")
    rospy.set_param("axalta/ccscore/dashboard/SCANNINGDONECHECK", False)
    rospy.set_param("axalta/ccscore/dashboard/SETTINGS_APPLIED", False)
    rospy.set_param("axalta/ccscore/dashboard/SOFTWARE_EMERGENCY_STOP", False)
    rospy.set_param("axalta/ccscore/dashboard/SURFACE_MAPPING_DONE", False)
    rospy.set_param("axalta/ccscore/dashboard/TAPE_COLOR", "#8FB74B")
   # rospy.set_param("axalta/ccscore/dashboard/ccs_lite_communicate_EMERGENCY", False)
    rospy.set_param("axalta/ccscore/dashboard/password", "admin")
    rospy.set_param("axalta/ccscore/dashboard/username", "admin")
    rospy.set_param("start_trajectory_calculation", False)
    rospy.set_param("axalta/ccscore/dashboard/ARMScanningPositionReached", False)
    rospy.set_param("axalta/ccscore/dashboard/ARMPaintingPositionReached", False)

if __name__ == '__main__':
    try:
        rospy.init_node('reset_handler', anonymous=True)
        while not rospy.is_shutdown():
            if(rospy.get_param("axalta/ccscore/ccs_lite_communicate_EMERGENCY") or rospy.get_param("axalta/ccscore/dashboard/EXIT_JOB_TRIGGER")):
                reset_params()
    except rospy.ROSInterruptException:
        pass


   