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

        # tf for 250 MiR
        br.sendTransform((-0.704, 0.408, 1.088),(0.873, -0.065, 0.141, -0.463),rospy.Time.now(),"camera_aligned_depth_to_color_frame_rec","base_link")

        
        # tf for 200 MiR
        #br.sendTransform((-0.700, 0.425, 1.050),(0.874, -0.054, 0.144, -0.461),rospy.Time.now(),"camera_aligned_depth_to_color_frame_rec","base_link") #from base_link to   camera_aligned_depth_to_color_frame (old scanning starting waypoint) ---- correct tf red door demo client



        #br.sendTransform((-0.656, 0.417, 1.023),(-0.716, -0.038, -0.015, 0.697),rospy.Time.now(),"camera_aligned_depth_to_color_frame_rec","base_link")
        #br.sendTransform((-0.47998127087616915, 0.7760614545949691, 0.7652098586969447),(-0.8410717085658216, 0.08973716228114144, -0.08277369651346443, 0.5269669229860587),rospy.Time.now(),"camera_aligned_depth_to_color_frame_rec","base_link") #-- try7new_06-07-2022 
        #br.sendTransform((-0.5454152628821011, 0.6282915866319, 0.8233082816095288),(-0.8285671564659823, 0.08093418366049984, -0.012384469872223313, 0.5538706979495023),rospy.Time.now(),"camera_aligned_depth_to_color_frame_rec","base_link") #-- try7_without_170 - base , camera in addframe.py
        #br.sendTransform((0.565796200335821, 1.1428549989919579, 0.46787407071686493),(0.8723376600152274, -0.06358247815965962, 0.14063606582937607, 0.46390276177047046),rospy.Time.now(),"camera_aligned_depth_to_color_frame_rec","base_link") #from base_link to camera_aligned_depth_to_color_frame
        #br.sendTransform((-0.7035660655162335, 0.4080374136423553, 1.0879611850485775),(-0.8723255830434613, 0.06364737986108435, -0.1406301006513929, 0.4639183796698316),rospy.Time.now(),"camera_aligned_depth_to_color_frame_rec","base_link") #from base_link to camera_aligned_depth_to_color_frame # current
        #br.sendTransform((0.022, 0.325, 1.124),(0.890, -0.019, 0.055, -0.452),rospy.Time.now(),"camera_aligned_depth_to_color_frame_rec","base_link") #from base_link to camera_aligned_depth_to_color_frame # current
        #br.sendTransform((-0.148, -0.024, 1.023),(-0.849, 0.058, -0.031, 0.524),rospy.Time.now(),"camera_aligned_depth_to_color_frame_rec","base_link") #from base_link to   camera_aligned_depth_to_color_frame - new tf waypoint1
        
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

""" [-0.086, 1.099, 0.395]
Rotation:  [0.890, -0.018, 0.054, 0.452] ---- camera_link , base_link"""


"""At time 1658216161.456
- Translation: [0.022, 0.325, 1.124]
- Rotation: in Quaternion [0.890, -0.019, 0.055, -0.452]
            in RPY (radian) [-2.198, -0.080, -0.083]
            in RPY (degree) [-125.960, -4.604, -4.748]"""

"""0.023, 0.325, 1.125  , 0.890, -0.018, 0.054, -0.452     ------------- base_link , camera_link

At time 1662547349.320
- Translation: [0.564, 1.143, 0.468]
- Rotation: in Quaternion [0.873, -0.062, 0.143, 0.463]
            in RPY (radian) [2.163, -0.311, 0.024]
            in RPY (degree) [123.903, -17.844, 1.391]
"""


