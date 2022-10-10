#!/usr/bin/env python
from __future__ import print_function
from pyquaternion import Quaternion
import rospy
import numpy as np
import tf.transformations as t
from std_msgs.msg import Header
import open3d as o3d
from pointcloud_process.srv import *
#from scipy.spatial.transform import Rotation as R
import math



def get_cross_prod_mat(pVec_Arr):
    # pVec_Arr shape (3)
    qCross_prod_mat = np.array([
        [0, -pVec_Arr[2], pVec_Arr[1]], 
        [pVec_Arr[2], 0, -pVec_Arr[0]],
        [-pVec_Arr[1], pVec_Arr[0], 0],
    ])
    return qCross_prod_mat

def get_rotation_offset_matrix(rotation_angle):
    '''x_rot_mat = np.array([
        [1, 0,0], 
        [0, math.cos(rotation_angle), -1*math.sin(rotation_angle)],
        [0, math.sin(rotation_angle), math.cos(rotation_angle)],
    ])'''
    '''x_rot_mat_dash = np.array([
        [1, 0,0], 
        [0, math.cos(rotation_angle), math.sin(rotation_angle)],
        [0, -1*math.sin(rotation_angle), math.cos(rotation_angle)],
    ])'''

    y_rot_mat = np.array([
        [math.cos(rotation_angle), 0, math.sin(rotation_angle)], 
        [0, 1, 0],
        [-1*math.sin(rotation_angle), 0, math.cos(rotation_angle)],
    ])

    '''z_rot_mat = np.array([
        [math.cos(rotation_angle), -1*math.sin(rotation_angle), 0], 
        [math.sin(rotation_angle), math.cos(rotation_angle), 0],
        [0, 0, 1],
    ])'''
    return y_rot_mat

def caculate_align_mat(pVec_Arr, rotation_angle):

    global count
    print("rotation_angle : ",rotation_angle)
    scale = np.linalg.norm(pVec_Arr)
    pVec_Arr = pVec_Arr/ scale
    # must ensure pVec_Arr is also a unit vec. 
    z_unit_Arr = np.array([0,1,0])
    z_mat = get_cross_prod_mat(z_unit_Arr)
    print("zmat : ",z_mat)

    if(rotation_angle != 0):
        z_mat = np.matmul(get_rotation_offset_matrix(rotation_angle), z_mat)
      
    z_c_vec = np.matmul(z_mat, pVec_Arr)
    z_c_vec_mat = get_cross_prod_mat(z_c_vec)

    if np.dot(z_unit_Arr, pVec_Arr) == -1:
        qTrans_Mat = -np.eye(3, 3)
    elif np.dot(z_unit_Arr, pVec_Arr) == 1:   
        qTrans_Mat = np.eye(3, 3)
    else:
        qTrans_Mat = np.eye(3, 3) + z_c_vec_mat + np.matmul(z_c_vec_mat,
                                                    z_c_vec_mat)/(1 + np.dot(z_unit_Arr, pVec_Arr))

    qTrans_Mat *= scale
    x90 = np.matrix('1.0, 0.0, 0.0; 0.0, 0.0, 1.0; 0.0, -1.0, 0.0')
    qTrans_Mat = np.matmul(qTrans_Mat,x90)
    return qTrans_Mat


def handle_quaternion_calculation(req):

    Vec_Arr = [req.x_normal, req.y_normal, req.z_normal]
    print("Vec_Arr",Vec_Arr)
    rot_mat = caculate_align_mat(Vec_Arr,req.rotation_angle)
    #print(rot_mat)
    print("reached",Quaternion(matrix=rot_mat)) 
    R_0 = np.eye(4,4)
    R_0[ 0:3,0:3 ] = rot_mat
      
    quat = t.quaternion_from_matrix(R_0)
    return NormalToQuaternionResponse([quat[0],quat[1],quat[2],quat[3]])

if __name__ == '__main__':
    rospy.init_node('quaternion_calculation')
        
    rate = rospy.Rate(10)
          
    s = rospy.Service("normal_to_quaternion",NormalToQuaternion,handle_quaternion_calculation)

   
    while not rospy.is_shutdown():
     
      rate.sleep()
    
    rospy.spin()
   
