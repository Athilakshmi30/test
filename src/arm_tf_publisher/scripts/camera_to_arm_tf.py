#!/usr/bin/env python
import rospy
import math
import tf


if __name__ == '__main__':
    rospy.init_node('camera_to_arm_tf')

    listener = tf.TransformListener()
    trans = []
    rot = []

    rate = rospy.Rate(10.0)
    while not trans:
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/camera_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) 
            continue
        rate.sleep()
    print 'Translation: ' , trans
    print 'Rotation: ' , rot
