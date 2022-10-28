#!/usr/bin/env python
import rospy
import math
import tf
from tf import TransformListener
from geometry_msgs.msg import Pose,PoseStamped
import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

import open3d as o3d
import numpy as np

rospy.init_node('norm_est_test')


cloud_out = PointCloud2()

def convertCloudFromRosToOpen3d(ros_cloud):
    
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud


def pc2CallBack(msg):
    global cloud_out
    cloud_out = msg


rospy.Subscriber("/filtered_data_transformed",PointCloud2,pc2CallBack)
rospy.sleep(2)
outpcd = convertCloudFromRosToOpen3d(cloud_out)

outpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
outpcd.normalize_normals()
print(outpcd.has_normals())
print(np.asarray(outpcd.points))
print(np.asarray(outpcd.normals))
#o3d.visualization.draw_geometries([outpcd], window_name= 'Open3D', width = 1920, height= 1080, left = 50, top = 50,point_show_normal = True)

rospy.spin()
