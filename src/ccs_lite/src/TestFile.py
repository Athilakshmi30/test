#!/usr/bin/env python

import rospy
import time
import sys
from ccs_lite_msgs.msg import CcsLiteCmd


def talker():
    rospy.init_node('talkerNode', anonymous=False)
    pub = rospy.Publisher('/ccs_lite_cmd', CcsLiteCmd, queue_size=10)
    time.sleep(2.0)
    rospy.loginfo('\npublishing ccs lite command')
    dataObj = CcsLiteCmd()
    dataObj.endActuatorIdentifier.data = 3
    dataObj.action.data = True
    pub.publish(dataObj)


if __name__ == '__main__':
    try:
        talker()
    except(KeyboardInterrupt, SystemExit):
        sys.exit(0)