#!/usr/bin/env python  

import rospy
import tf
import math

if __name__ == '__main__':
    rospy.init_node('add_frame')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() * math.pi
        #br.sendTransform((-0.700, 0.425, 1.050),(0.874, -0.054, 0.144, -0.461),rospy.Time.now(),"camera_aligned_depth_to_color_frame_rec","base_link") #from base_link to camera_aligned_depth_to_color_frame (old scanning starting waypoint)
        #br.sendTransform((-0.656, 0.417, 1.023),(-0.716, -0.038, -0.015, 0.697),rospy.Time.now(),"camera_aligned_depth_to_color_frame_rec","base_link")
        #br.sendTransform((-0.47998127087616915, 0.7760614545949691, 0.7652098586969447),(-0.8410717085658216, 0.08973716228114144, -0.08277369651346443, 0.5269669229860587),rospy.Time.now(),"camera_aligned_depth_to_color_frame_rec","base_link") #-- try7new_06-07-2022 
        #br.sendTransform((-0.5454152628821011, 0.6282915866319, 0.8233082816095288),(-0.8285671564659823, 0.08093418366049984, -0.012384469872223313, 0.5538706979495023),rospy.Time.now(),"camera_aligned_depth_to_color_frame_rec","base_link") #-- try7_without_170 - base , camera in addframe.py
        #br.sendTransform((0.565796200335821, 1.1428549989919579, 0.46787407071686493),(0.8723376600152274, -0.06358247815965962, 0.14063606582937607, 0.46390276177047046),rospy.Time.now(),"camera_aligned_depth_to_color_frame_rec","base_link") #from base_link to camera_aligned_depth_to_color_frame
        #br.sendTransform((-0.7035660655162335, 0.4080374136423553, 1.0879611850485775),(-0.8723255830434613, 0.06364737986108435, -0.1406301006513929, 0.4639183796698316),rospy.Time.now(),"camera_aligned_depth_to_color_frame_rec","base_link") #from base_link to camera_aligned_depth_to_color_frame
        br.sendTransform((0.17220850162695425, -0.03516441938551822, 0.8936423997533117),(0.8452499263890393, 0.0056484697988488436, -0.02958744060386142, -0.533521546037991),rospy.Time.now(),"camera_aligned_depth_to_color_frame_rec","base_link") #-- try7new_06-07-2022 

        rate.sleep()



""" from /camera_aligned_depth_to_color_frame to /base_link 
At time 1654599059.454
- Translation: [0.563, 1.081, -0.408]
- Rotation: in Quaternion [0.716, 0.038, 0.015, 0.697]
            in RPY (radian) [1.600, 0.031, 0.076]
            in RPY (degree) [91.652, 1.777, 4.371]"""

"""from /base_link to /camera_aligned_depth_to_color_frame
At time 1654599043.162
- Translation: [-0.656, 0.417, 1.023]
- Rotation: in Quaternion [-0.716, -0.038, -0.015, 0.697]
            in RPY (radian) [-1.600, -0.075, 0.033]
            in RPY (degree) [-91.655, -4.318, 1.901]
At time 1654599043.806"""
"""
Translation:  [-0.5454152628821011, 0.6282915866319, 0.8233082816095288]
Rotation:  [-0.8285671564659823, 0.08093418366049984, -0.012384469872223313, 0.5538706979495023]"""

"""Translation:  [0.6878385690759894, 0.9262358644320202, -0.19737692887364994]
Rotation:  [0.828600038859957, -0.08089868769430927, 0.012416097090372656, 0.5538259821132071]"""

"""Translation:  [0.565796200335821, 1.1428549989919579, 0.46787407071686493]
Rotation:  [0.8723376600152274, -0.06358247815965962, 0.14063606582937607, 0.46390276177047046] - try6 updated without 170""" 

"""Translation:  [-0.7035660655162335, 0.4080374136423553, 1.0879611850485775]
Rotation:  [-0.8723255830434613, 0.06364737986108435, -0.1406301006513929, 0.4639183796698316]- try6 updated without 170""" 
'''
Translation:  [-0.13115243352514835, 0.7949249495884623, 0.42468756875543345]                        
Rotation:  [-0.8452504212211204, -0.005668765319500924, 0.029584428626030494, -0.5335207138510565]    ----  single location ply camera , base in cameratoarmtf py

Translation:  [0.17220850162695425, -0.03516441938551822, 0.8936423997533117]                          
Rotation:  [0.8452499263890393, 0.0056484697988488436, -0.02958744060386142, -0.533521546037991]        ------------ single location ply base , camera  in camera to arm tf py

'''

