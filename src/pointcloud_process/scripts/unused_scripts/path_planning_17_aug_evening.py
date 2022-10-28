#!/usr/bin/env python
from __future__ import print_function
from pickletools import float8
#from tkinter.tix import Tree
#from textwrap import indent
from traceback import print_tb
from turtle import right

import rospy
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from scipy.spatial import Delaunay
from itertools import compress, count
from ctypes import *
import time
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import random
from arm_ctrl_navigate.msg import Plannedpath, PathStamped, Path

import scipy.optimize as optimize
import numpy as np
import copy

# 5 for sealer,3 for base1, 4 for base2, 7 for clear 1 and clear 2


# approximate(index_row_distance)-2.0 (in cms) --- formula

# rospy.set_param("sealer_grid_width",0.019)#0.03175)#0.03175)#0.019
rospy.set_param("sealer_grid_height", 4)  # 0.03175)#0.03175)#0.019
# rospy.set_param("sealer_grid_depth",0.019)#0.03175)#0.03175)#0.019
# rospy.set_param("basecoat1_grid_width",0.019)#0.019
rospy.set_param("basecoat1_grid_height", 2)  # 0.019
# rospy.set_param("basecoat1_grid_depth",0.019)#0.019
# rospy.set_param("basecoat2_grid_width",0.019)#0.0254)#0.0254)#0.019
rospy.set_param("basecoat2_grid_height", 3)  # 0.0254)#0.0254)#0.019
# rospy.set_param("basecoat2_grid_depth",0.019)#0254)#0.0254)#0.019
# rospy.set_param("clearcoat1_grid_width",0.0381)#0.0381)#0.019
rospy.set_param("clearcoat1_grid_height", 6)  # 0.0381)#0.019
# rospy.set_param("clearcoat1_grid_depth",0.0381)#0.0381)#0.019
# rospy.set_param("clearcoat2_grid_width",0.0381)#0.0381)#0.019
rospy.set_param("clearcoat2_grid_height", 6)  # 0.0381)#0.019
# rospy.set_param("clearcoat2_grid_depth",0.0381)#0.0381)#0.019
rospy.set_param("coat_process", False)


class PathPlanning():

    def __init__(self):

        rospy.init_node('path_planning')

        self.griddedCloud_pc = rospy.Publisher(
            "/gridded_cloud_pc", PointCloud2, queue_size=1)

        self.sealercoat_pc = rospy.Publisher(
            "/sealercoat_pc", PointCloud2, queue_size=1)

        self.basecoat1_pc = rospy.Publisher(
            "/basecoat1_pc", PointCloud2, queue_size=1)

        self.basecoat2_pc = rospy.Publisher(
            "/basecoat2_pc", PointCloud2, queue_size=1)

        self.clearcoat1_pc = rospy.Publisher(
            "/clearcoat1_pc", PointCloud2, queue_size=1)

        self.clearcoat2_pc = rospy.Publisher(
            "/clearcoat2_pc", PointCloud2, queue_size=1)

        self.sealercoat = rospy.Publisher(
            "/sealercoat", Path, queue_size=1)

        self.basecoat1 = rospy.Publisher(
            "/basecoat1", Path, queue_size=1)

        self.basecoat2 = rospy.Publisher(
            "/basecoat2", Path, queue_size=1)

        self.clearcoat1 = rospy.Publisher(
            "/clearcoat1", Path, queue_size=1)

        self.clearcoat2 = rospy.Publisher(
            "/clearcoat2", Path, queue_size=1)

        self.horizontal_pc = rospy.Publisher(
            "/horizontal_pc", PointCloud2, queue_size=1)

        self.received_sealer = False
        self.received_base1 = True
        self.received_base2 = True
        self.received_clear1 = True
        self.received_clear2 = True
        self.processed = False

        self.sealer_row_index = 0.0635
        self.base1_row_index = 0.0381
        self.base2_row_index = 0.0508
        self.clear1_row_index = 0.0762
        self.clear2_row_index = 0.0762

        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                       PointField('y', 4, PointField.FLOAT32, 1),
                       PointField('z', 8, PointField.FLOAT32, 1)
                       ]

        self.fields_normal = [PointField('x', 0, PointField.FLOAT32, 1),
                              PointField('y', 4, PointField.FLOAT32, 1),
                              PointField('z', 8, PointField.FLOAT32, 1),
                              PointField('normal_x', 12,
                                         PointField.FLOAT32, 1),
                              PointField('normal_y', 16,
                                         PointField.FLOAT32, 1),
                              PointField('normal_z', 20, PointField.FLOAT32, 1)
                              ]

        self.fields_orientation = [PointField('x', 0, PointField.FLOAT32, 1),
                                   PointField('y', 4, PointField.FLOAT32, 1),
                                   PointField('z', 8, PointField.FLOAT32, 1),
                                   PointField('normal_x', 12,
                                              PointField.FLOAT32, 1),
                                   PointField('normal_y', 16,
                                              PointField.FLOAT32, 1),
                                   PointField('normal_z', 20,
                                              PointField.FLOAT32, 1),
                                   PointField('curvature', 24,
                                              PointField.FLOAT32, 1)
                                   ]

        self.FIELDS_XYZ = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]

        self.FIELDS_XYZNXNYNZ = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='normal_x', offset=12,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='normal_y', offset=16,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='normal_z', offset=20,
                       datatype=PointField.FLOAT32, count=1),
        ]

        self.FIELDS_XYZRGB = self.FIELDS_XYZ + \
            [PointField(name='rgb', offset=12,
                        datatype=PointField.UINT32, count=1)]

        self.FIELDS_XYZNXNYNZRGB = self.FIELDS_XYZNXNYNZ + \
            [PointField(name='rgb', offset=24,
                        datatype=PointField.UINT32, count=1)]

        self.BIT_MOVE_16 = 2**16
        self.BIT_MOVE_8 = 2**8
        self.convert_rgbUint32_to_tuple = lambda rgb_uint32: (
            (rgb_uint32 & 0x00ff0000) >> 16, (rgb_uint32 &
                                              0x0000ff00) >> 8, (rgb_uint32 & 0x000000ff)
        )
        self.convert_rgbFloat_to_tuple = lambda rgb_float: self.convert_rgbUint32_to_tuple(
            int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
        )

        self.header = Header()
        self.header.frame_id = "mir_link"
        self.header.stamp = rospy.Time.now()

        self.point_cloud_input = PointCloud2()
        self.point_cloud_list = []

        self.max_depth_added_cloud_list = []
        self.max_depth_added_cloud = PointCloud2()

        self.occlusion_removed_cloud_list = []
        self.occlusion_removed_cloud = PointCloud2()

        self.gridded_cloud = PointCloud2()

        self.horizontal_cloud = PointCloud2()
        self.sealer_data_seq_cloud = PointCloud2()
        self.basecoat1_data_seq_cloud = PointCloud2()
        self.basecoat2_data_seq_cloud = PointCloud2()
        self.clearcoat1_data_seq_cloud = PointCloud2()
        self.clearcoat2_data_seq_cloud = PointCloud2()

        self.sealer_data_row_path_list = Path()
        self.max_depth = -10.0
        self.min_depth = 10.0

        self.max_height = -10.0
        self.min_height = 10.0

        self.max_width = -10.0
        self.min_width = 10.0

        self.depth_width = 0.05  # 5 cms

        self.sealer_row_index = 0.0635
        self.base1_row_index = 0.0381
        self.base2_row_index = 0.0508
        self.clear1_row_index = 0.0762
        self.clear2_row_index = 0.0762

        self.left_right_index_offset = 0.0762  # 3 inches #0.1524 - 6 inches

        self.top_bottom_index_row_offset = 0.0762  # 3 inches #0.1524 - 6 inches

        self.top_index_row_offset = 0.0762  # 3 inches #0.1524 - 6 inches
        self.bottom_index_row_offset = 0.0762  # 3 inches #0.1524 - 6 inches

        self.grid_width = 0.01
        self.grid_height = 0.01
        self.grid_depth = 0.01

        rospy.Subscriber("/orientation_points_sealercoat",
                         PointCloud2, self.sealerCallBack)

        rospy.Subscriber("/orientation_points_basecoat_1",
                         PointCloud2, self.base1CallBack)

        rospy.Subscriber("/orientation_points_basecoat_2",
                         PointCloud2, self.base2CallBack)

        rospy.Subscriber("/orientation_points_clearcoat_1",
                         PointCloud2, self.clear1CallBack)

        rospy.Subscriber("/orientation_points_clearcoat_2",
                         PointCloud2, self.clear2CallBack)

        self.start()

    def convertCloudFromOpen3dToRos(self, open3d_cloud):

        # Set "fields" and "cloud_data" might use asarray_chkfinite?
        points = np.asarray(open3d_cloud.points)
        if not open3d_cloud.colors:  # XYZ only
            fields = self.FIELDS_XYZ
            cloud_data = points
        else:  # XYZ + RGB
            fields = self.FIELDS_XYZRGB
            # -- Change rgb color from "three float" to "one 24-byte int"
            # 0x00FFFFFF is white, 0x00000000 is black.

            colors = np.floor(np.asarray(open3d_cloud.colors)
                              * 255)  # nx3 matrix
            colors = colors[:, 0] * self.BIT_MOVE_16 + \
                colors[:, 1] * self.BIT_MOVE_8 + colors[:, 2]
            cloud_data = np.c_[points, colors]

        # create ros_cloud
        return point_cloud2.create_cloud(self.header, fields, cloud_data)

    def convertCloudFromRosToOpen3d(self, ros_cloud):

        # Get cloud data from ros_cloud
        field_names = [field.name for field in ros_cloud.fields]
        cloud_data = list(point_cloud2.read_points(
            ros_cloud, skip_nans=True, field_names=field_names))

        # Check empty
        open3d_cloud = o3d.geometry.PointCloud()
        if len(cloud_data) == 0:
            print("Converting an empty cloud")
            return None

        # Set open3d_cloud
        if "rgb" in field_names:
            IDX_RGB_IN_FIELD = 3  # x, y, z, rgb

            # Get xyz
            # (why cannot put this line below rgb?)
            xyz = [(x, y, z) for x, y, z, rgb in cloud_data]

            # Get rgb
            # Check whether int or float
            # if float (from pcl::toROSMsg)
            if type(cloud_data[0][IDX_RGB_IN_FIELD]) == float:
                rgb = [self.convert_rgbFloat_to_tuple(
                    rgb) for x, y, z, rgb in cloud_data]
            else:
                rgb = [self.convert_rgbUint32_to_tuple(
                    rgb) for x, y, z, rgb in cloud_data]

            # combine
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
            open3d_cloud.colors = o3d.utility.Vector3dVector(
                np.array(rgb)/255.0)
        else:
            xyz = [(x, y, z) for x, y, z in cloud_data]  # get xyz
            open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

        # return
        return open3d_cloud

    def convertCloudFromOpen3dToRosWithNormals(self, open3d_cloud):

        # Set "fields" and "cloud_data" might use asarray_chkfinite?
        points = np.asarray(open3d_cloud.points)
        normals = np.asarray(open3d_cloud.normals)

        if not open3d_cloud.colors:  # XYZ only
            fields = self.FIELDS_XYZNXNYNZ
            cloud_data = np.c_[points, normals]
        else:  # XYZ + RGB
            fields = self.FIELDS_XYZNXNYNZRGB
            # -- Change rgb color from "three float" to "one 24-byte int"
            # 0x00FFFFFF is white, 0x00000000 is black.

            colors = np.floor(np.asarray(open3d_cloud.colors)
                              * 255)  # nx3 matrix
            colors = colors[:, 0] * self.BIT_MOVE_16 + \
                colors[:, 1] * self.BIT_MOVE_8 + colors[:, 2]
            cloud_data = np.c_[points, normals, colors]

        # create ros_cloud
        return point_cloud2.create_cloud(self.header, fields, cloud_data)

    def sealerCallBack(self, msg):

        if (not self.received_sealer):
            print("msg.fields", msg.fields)
            self.point_cloud_input = msg
            self.sealer_grid_height = rospy.get_param("sealer_grid_height")

            print("Sealer")

            self.point_cloud_list = list(point_cloud2.read_points(
                self.point_cloud_input, field_names=['x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z'], skip_nans=True))
            #self.sealer_coat_data = self.start(self.sealer_grid_width,self.sealer_grid_height,self.sealer_grid_depth)

            common_plane_data_list, common_boolean_grid, common_gridded_cloud_list,  minx, maxx, miny, maxy, minz, maxz = self.createGrid(
                self.point_cloud_list)
            #print("selaer len",len(sealer_plane_data_list))
            horizontal_plane_list, horizontal_handled_gridded_list, horizontal_handled_gridded_list_1D, horizontal_list = self.plan_path(
                common_plane_data_list, minx, miny, maxx, maxy, self.sealer_row_index)
            sealer_data_row_list, self.sealer_data_row_path_list = self.sequencing(
                horizontal_handled_gridded_list, horizontal_plane_list, self.sealer_grid_height, minz,minx,maxx)
            self.sealer_gridded_cloud = point_cloud2.create_cloud(
                self.header, self.fields_normal, common_gridded_cloud_list)

            self.sealer_data_seq_cloud = point_cloud2.create_cloud(
                self.header, self.fields_normal, sealer_data_row_list)
            print("Process complited")
            self.received_sealer = True

    def base1CallBack(self, msg):

        if (not self.received_base1 and self.received_sealer):
            # print(msg.fields)
            self.point_cloud_input = msg
            self.basecoat1_grid_height = rospy.get_param(
                "basecoat1_grid_height")

            print("Base1")

            self.point_cloud_list = list(point_cloud2.read_points(
                self.point_cloud_input, field_names=['x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z'], skip_nans=True))
            #self.sealer_coat_data = self.start(self.sealer_grid_width,self.sealer_grid_height,self.sealer_grid_depth)

            common_plane_data_list, common_boolean_grid, common_gridded_cloud_list, minx, maxx, miny, maxy, minz, maxz = self.createGrid(
                self.point_cloud_list)
            #print("selaer len",len(sealer_plane_data_list))
            horizontal_plane_list, horizontal_handled_gridded_list, horizontal_handled_gridded_list_1D, horizontal_list = self.plan_path(
                common_plane_data_list,  minx, miny, maxx, maxy, self.base1_row_index)
            basecoat1_data_row_list, self.basecoat1_data_row_path_list = self.sequencing(
                horizontal_handled_gridded_list, horizontal_plane_list, self.basecoat1_grid_height, minz)
            self.basecoat1_gridded_cloud = point_cloud2.create_cloud(
                self.header, self.fields_normal, common_gridded_cloud_list)
            self.basecoat1_data_seq_cloud = point_cloud2.create_cloud(
                self.header, self.fields_normal, basecoat1_data_row_list)
            self.received_base1 = True

    def base2CallBack(self, msg):

        if (not self.received_base2 and self.received_base1 and self.received_sealer):
            # print(msg.fields)
            self.point_cloud_input = msg
            self.basecoat2_grid_height = rospy.get_param(
                "basecoat2_grid_height")

            print("Base2")

            self.point_cloud_list = list(point_cloud2.read_points(
                self.point_cloud_input, field_names=['x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z'], skip_nans=True))
            #self.sealer_coat_data = self.start(self.sealer_grid_width,self.sealer_grid_height,self.sealer_grid_depth)

            common_plane_data_list, common_boolean_grid, common_gridded_cloud_list, minx, maxx, miny, maxy, minz, maxz = self.createGrid(
                self.point_cloud_list)
            #print("selaer len",len(sealer_plane_data_list))
            horizontal_plane_list, horizontal_handled_gridded_list, horizontal_handled_gridded_list_1D, horizontal_list = self.plan_path(
                common_plane_data_list,  minx, miny, maxx, maxy, self.base2_row_index)
            basecoat2_data_row_list, self.basecoat2_data_row_path_list = self.sequencing(
                horizontal_handled_gridded_list, horizontal_plane_list, self.basecoat2_grid_height, minz)
            self.basecoat2_gridded_cloud = point_cloud2.create_cloud(
                self.header, self.fields_normal, common_gridded_cloud_list)
            self.basecoat2_data_seq_cloud = point_cloud2.create_cloud(
                self.header, self.fields_normal, basecoat2_data_row_list)
            self.received_base2 = True

    def clear1CallBack(self, msg):

        if (not self.received_clear1 and self.received_base2 and self.received_base1 and self.received_sealer):
            # print(msg.fields)
            self.point_cloud_input = msg
            self.clearcoat1_grid_height = rospy.get_param(
                "clearcoat1_grid_height")
            print("Clear1")

            self.point_cloud_list = list(point_cloud2.read_points(
                self.point_cloud_input, field_names=['x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z'], skip_nans=True))
            #self.sealer_coat_data = self.start(self.sealer_grid_width,self.sealer_grid_height,self.sealer_grid_depth)

            common_plane_data_list, common_boolean_grid, common_gridded_cloud_list, minx, maxx, miny, maxy, minz, maxz = self.createGrid(
                self.point_cloud_list)
            horizontal_plane_list, horizontal_handled_gridded_list, horizontal_handled_gridded_list_1D, horizontal_list = self.plan_path(
                common_plane_data_list,  minx, miny, maxx, maxy, self.clear1_row_index)
            #print("selaer len",len(sealer_plane_data_list))
            clearcoat1_data_row_list, self.clearcoat1_data_row_path_list = self.sequencing(
                horizontal_handled_gridded_list, horizontal_plane_list, self.clearcoat1_grid_height, minz)
            self.clearcoat1_gridded_cloud = point_cloud2.create_cloud(
                self.header, self.fields_normal, common_gridded_cloud_list)
            self.clearcoat1_data_seq_cloud = point_cloud2.create_cloud(
                self.header, self.fields_normal, clearcoat1_data_row_list)
            self.received_clear1 = True

    def clear2CallBack(self, msg):

        if (not self.received_clear2 and self.received_clear1 and self.received_base2 and self.received_base1 and self.received_sealer):
            # print(msg.fields)
            self.point_cloud_input = msg
            self.clearcoat2_grid_height = rospy.get_param(
                "clearcoat2_grid_height")

            print("Clear2")

            self.point_cloud_list = list(point_cloud2.read_points(
                self.point_cloud_input, field_names=['x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z'], skip_nans=True))
            #self.sealer_coat_data = self.start(self.sealer_grid_width,self.sealer_grid_height,self.sealer_grid_depth)

            common_plane_data_list, common_boolean_grid, common_gridded_cloud_list,  minx, maxx, miny, maxy, minz, maxz = self.createGrid(
                self.point_cloud_list)
            #print("selaer len",len(sealer_plane_data_list))
            horizontal_plane_list, horizontal_handled_gridded_list, horizontal_handled_gridded_list_1D, horizontal_list = self.plan_path(
                common_plane_data_list,  minx, miny, maxx, maxy, self.clear2_row_index)
            clearcoat2_data_row_list, self.clearcoat2_data_row_path_list = self.sequencing(
                horizontal_handled_gridded_list, horizontal_plane_list, self.clearcoat2_grid_height, minz)
            self.clearcoat2_gridded_cloud = point_cloud2.create_cloud(
                self.header, self.fields_normal, common_gridded_cloud_list)
            self.clearcoat2_data_seq_cloud = point_cloud2.create_cloud(
                self.header, self.fields_normal, clearcoat2_data_row_list)
            self.received_clear2 = True

    def removeHiddenPoints(self, cloud):

        pcd = self.convertCloudFromRosToOpen3d(cloud)
        diameter = (self.max_height + self.min_height)/2.0
        camera = [0, 0, diameter]
        radius = diameter * 100

        #"Get all points that are visible from given view point"
        _, pt_map = pcd.hidden_point_removal(camera, radius)

        hidden_points_removed = self.convertCloudFromOpen3dToRos(
            pcd.select_down_sample(pt_map))

        hidden_points_removed_list = list(point_cloud2.read_points(
            hidden_points_removed, field_names=['x', 'y', 'z'], skip_nans=True))

        return hidden_points_removed, hidden_points_removed_list

    def get_points_in_range(self, cloud, z_range):

        #print("[Z RANGE = ", z_range, " ]")
        cropped_cloud = [[pt[0], pt[1], pt[2]] for pt in cloud if (
            (pt[2] < (z_range + 0.0055)) and (pt[2] > (z_range - 0.0055)))]
        print("length ---------------------------- : ", len(cropped_cloud))

        return cropped_cloud

    def centroid(self, points):

        sum_cube_x = 0.0
        sum_cube_y = 0.0
        sum_cube_z = 0.0

        for pt in points:
            sum_cube_x = sum_cube_x + pt[0]
            sum_cube_y = sum_cube_y + pt[1]
            sum_cube_z = sum_cube_z + pt[2]

        return [sum_cube_x/8.0, sum_cube_y/8.0, sum_cube_z/8.0]

    def createGrid(self, cloud):

        pc2_list = copy.deepcopy(cloud)
        maxy = -1.0
        miny = 10.0
        maxx = -1.0
        minx = 10.0
        minz = 10.0
        maxz = -1.0

        for data in pc2_list:
            x = data[0]
            y = data[1]
            z = data[2]

            if (maxy < y):
                maxy = y
            if (miny > y):
                miny = y
            if (maxx < x):
                maxx = x
            if (minx > x):
                minx = x
            if (maxz < z):
                maxz = z
            if (minz > z):
                minz = z

        #*********************************max  min  for grid***************************#
        self.max_depth = maxy + self.depth_width
        self.min_depth = miny - self.depth_width
        self.max_width = maxx + self.left_right_index_offset
        self.min_width = minx - self.left_right_index_offset
        self.max_height = maxz  # + self.top_bottom_index_row_offset
        self.min_height = minz  # - self.top_bottom_index_row_offset

        total = 0.0
        boolean_grid = []
        gridded_cloud = []

        current_x = self.min_width - 0.005
        current_y = self.min_depth - 0.005
        current_z = self.min_height - 0.005
        cropped_cloud = cloud

        print("********************************************creating grid**********************************************************")
        while (current_z <= self.max_height + 0.005):

            current_y = self.min_depth - 0.005
            depth_row = []
            depth_boolean_row = []

            z_range = current_z
            print("z_range , self.max_height + 0.005 : ",
                  z_range, self.max_height + 0.005)
            cropped_cloud = self.get_points_in_range(cloud, z_range)

            print("[ITERATION] ------------------------------------ ")
            if (len(cropped_cloud) > 1):

                while (current_y <= self.max_depth + 0.005):

                    current_x = self.min_width - 0.005
                    width_row = []

                    width_boolean_row = []

                    while (current_x <= self.max_width + 0.005):

                        extreme_low_point = [current_x, current_y, current_z]
                        extreme_high_point = [
                            current_x+self.grid_width, current_y+self.grid_depth, current_z+self.grid_height]

                        start = time.time()

                        bool_list = []
                        current_list = []
                        for pt in cropped_cloud:
                            if(pt[0] >= extreme_low_point[0] and pt[0] < extreme_high_point[0] and pt[1] >= extreme_low_point[1] and pt[1] < extreme_high_point[1] and pt[2] >= extreme_low_point[2] and pt[2] < extreme_high_point[2]):
                                bool_list.append(True)
                                current_list.append(pt)
                            else:
                                bool_list.append(False)
                        ending = time.time()

                        total = total + ending - start

                        if (len(current_list) > 0):
                            #print("got hits")

                            pt_n = []
                            for pt in cloud:
                                if (abs(pt[0] - current_list[0][0]) <= 0.001 and abs(pt[1] - current_list[0][1]) <= 0.001 and abs(pt[2] - current_list[0][2]) <= 0.001):
                                    pt_n = list(pt)
                                    break

                            width_row.append(pt_n)
                            width_boolean_row.append(True)

                        else:

                            width_boolean_row.append(False)
                            width_row.append([0, 0, 0, 0, 0, 0])

                        current_x = current_x + self.grid_width

                    depth_row.append(width_row)
                    depth_boolean_row.append(width_boolean_row)

                    current_y = current_y + self.grid_depth

                gridded_cloud.append(depth_row)
                boolean_grid.append(depth_boolean_row)

            current_z = current_z + self.grid_height

        gridded_cloud_linear = [
            pt for row in gridded_cloud for col in row for pt in col]

        return gridded_cloud, boolean_grid, gridded_cloud_linear, minx, maxx, miny, maxy, minz, maxz

    def points_for_path(self, boolean_grid):

        start = time.time()
        #print("start : ",start)
        len_of_planes = len(boolean_grid)
        len_of_depthrows_in_each_plane = len(boolean_grid[0])
        len_of_each_depthrow = len(boolean_grid[0][1])

        curr_plane = len_of_planes - 1

        curr_depth_row = 0
        curr_point = 0

        path_list = []

        while (curr_plane >= 0):

            curr_depth_row = 0
            plane = []
            while ((curr_depth_row <= (len_of_depthrows_in_each_plane - 1))):

                curr_point = 0
                row = []
                while (curr_point <= (len_of_each_depthrow - 1)):

                    boolean_point = boolean_grid[curr_plane][curr_depth_row][curr_point]
                    if (boolean_point):
                        row.append([curr_plane, curr_depth_row, curr_point])

                    curr_point += 1

                if (len(row) > 0):
                    plane.append(row)

                curr_depth_row += 1

            if (len(plane) > 0):
                path_list.append(plane)
            curr_plane -= 1

        ending = time.time()

        return path_list, boolean_grid

    def two_d_curve_fit(self, data_list, z_val, horizontal=False):

        X_data = []
        Y_data = []
        Z_data = []
        check_z = []
        new_fitted_points = []

        for rows in data_list:

            for pt in rows:
                check_z.append(pt[2])
                if pt[0] == 0:
                    continue
                X_data.append(pt[0])
                Y_data.append(pt[1])
                if (not horizontal):
                    Z_data.append(z_val)
                else:
                    Z_data.append(pt[2])
        Z_data_arr = np.array(Z_data)

        if len(X_data) == 0:
            check_z_arr = np.array(check_z)
            print("No X,Y point data for Z = ", check_z_arr.mean())
            new_fitted_points = np.array(new_fitted_points)
            return new_fitted_points
        # 6deg
        a, b, c, d, e, f, g = np.polyfit(X_data, Y_data, 6)
        def fit_equation(x): return a * x ** 6 + b * \
            x ** 5 + c*x**4+d*x**3+e*x**2+f*x+g
        avg_z = Z_data_arr.mean()
        fitted_points = []

        for row in data_list:

            for indx, pt in enumerate(row):
                if pt[0] == 0:
                    continue
                #print("pt : ",pt)

                for ind, xcoord in enumerate(X_data):

                    # and indx%3 == 0):
                    if(not horizontal):
                        if(abs(xcoord - pt[0]) < 0.001 and abs(Y_data[ind] - pt[1]) < 0.001 and abs(Z_data[ind] - z_val) < 0.001):
                            fitted_points.append(
                                [xcoord, fit_equation(xcoord), avg_z, pt[3], pt[4], pt[5]])
                    else:
                        if(abs(xcoord - pt[0]) < 0.001 and abs(Y_data[ind] - pt[1]) < 0.001 and abs(Z_data[ind] - pt[2]) < 0.001):
                            fitted_points.append(
                                [xcoord, fit_equation(xcoord), pt[2], pt[3], pt[4], pt[5]])

        fitted_points = sorted(fitted_points, key=lambda x: x[0])

        prev_x = -100
        for ind, i in enumerate(fitted_points):

            if prev_x == -100:
                #print("First condition")
                prev_x = i[0]
                new_fitted_points.append(i)

            elif abs(prev_x - i[0]) > 0.02:  # 0.02
                #print("Second condition")
                prev_x = i[0]
                new_fitted_points.append(i)
            else:
                pass

            count = 0

        prev_new_fitted_points = copy.deepcopy(new_fitted_points)
        diff_f_s_xz = -500
        diff_f_t_xz = -500
        diff_f_fo_xz = -500
        diff_f_fi_xz = -500
        diff_s_t_xz = -500
        diff_s_fo_xz = -500
        diff_s_fi_xz = -500
        prev_point_exist = False
        change_threshold = 0.01  # 0.02
        equal_threshold = 0.005  # 0.01

        for ind, point in enumerate(new_fitted_points):

            first_point = new_fitted_points[ind]

            if len(new_fitted_points)-ind > 1:
                second_point = new_fitted_points[ind+1]
                diff_f_s_xz = abs(float(first_point[5])-float(second_point[5]))
                # print("second_point",second_point[5])

            if len(new_fitted_points)-ind > 2:
                third_point = new_fitted_points[ind+2]
                diff_f_t_xz = abs(float(first_point[5])-float(third_point[5]))
                diff_s_t_xz = abs(float(second_point[5])-float(third_point[5]))

            if len(new_fitted_points)-ind > 3:
                fourth_point = new_fitted_points[ind+3]
                diff_f_fo_xz = abs(
                    float(first_point[5])-float(fourth_point[5]))
                diff_s_fo_xz = abs(
                    float(second_point[5])-float(fourth_point[5]))

            if len(new_fitted_points)-ind > 4:
                fifth_point = new_fitted_points[ind+4]
                diff_f_fi_xz = abs(float(first_point[5])-float(fifth_point[5]))
                diff_s_fi_xz = abs(
                    float(second_point[5])-float(fifth_point[5]))

            if (ind == len(new_fitted_points)-1):
                prev_point = new_fitted_points[ind-1]
                first_point[3] = prev_point[3]
                first_point[4] = prev_point[4]
                first_point[5] = prev_point[5]

            elif diff_f_s_xz > change_threshold:

                # modify current point / first point by giving second point value
                if (diff_s_t_xz < equal_threshold) and (diff_s_fo_xz < equal_threshold) and (diff_s_fi_xz < equal_threshold):
                    if not prev_point_exist:

                        first_point[3] = second_point[3]
                        first_point[4] = second_point[4]
                        first_point[5] = second_point[5]
                # modify second, third and fourth point
                elif (diff_s_t_xz < equal_threshold) and (diff_s_fo_xz < equal_threshold) and (diff_s_fi_xz > equal_threshold):

                    # Modify second point
                    second_point[3] = first_point[3]
                    second_point[4] = first_point[4]
                    second_point[5] = first_point[5]

                    # Modify third point
                    third_point[3] = first_point[3]
                    third_point[4] = first_point[4]
                    third_point[5] = first_point[5]

                    # Modify fourth point
                    fourth_point[3] = first_point[3]
                    fourth_point[4] = first_point[4]
                    fourth_point[5] = first_point[5]

                elif (diff_s_t_xz < equal_threshold) and (diff_s_fo_xz > equal_threshold) and (diff_s_fi_xz > equal_threshold or diff_s_fi_xz == -500):  # modify second, third

                    # Modify second point
                    second_point[3] = first_point[3]
                    second_point[4] = first_point[4]
                    second_point[5] = first_point[5]

                    # Modify third point
                    third_point[3] = first_point[3]
                    third_point[4] = first_point[4]
                    third_point[5] = first_point[5]

                elif (diff_s_t_xz > equal_threshold or diff_s_t_xz == -500):  # modify second

                    # Modify second point
                    second_point[3] = first_point[3]
                    second_point[4] = first_point[4]
                    second_point[5] = first_point[5]

            first_point = []
            second_point = []
            third_point = []
            fourth_point = []
            fifth_point = []
            diff_f_s_xz = -500
            diff_f_t_xz = -500
            diff_f_fo_xz = -500
            diff_f_fi_xz = -500
            diff_s_t_xz = -500
            diff_s_fo_xz = -500
            diff_s_fi_xz = -500
            prev_point_exist = True

        prev_x = -100
        drop_new_fitted_points = []
        for ind, i in enumerate(new_fitted_points):

            if prev_x == -100:
                #print("First condition")
                prev_x = i[0]
                drop_new_fitted_points.append(i)

            elif abs(prev_x - i[0]) > 0.04:  # 0.02
                #print("Second condition")
                prev_x = i[0]
                drop_new_fitted_points.append(i)
            else:
                pass

        prev_x_old = -100
        prev_drop_new_fitted_points = []
        for ind, i in enumerate(prev_new_fitted_points):

            if prev_x_old == -100:

                prev_x_old = i[0]
                prev_drop_new_fitted_points.append(i)

            elif abs(prev_x_old - i[0]) > 0.04:  # 0.02

                prev_x_old = i[0]
                prev_drop_new_fitted_points.append(i)
            else:
                pass

        new_fitted_points = np.array(new_fitted_points)
        prev_new_fitted_points = np.array(prev_new_fitted_points)
        drop_new_fitted_points = np.array(drop_new_fitted_points)
        prev_drop_new_fitted_points = np.array(prev_drop_new_fitted_points)
        # print("fitted_points",fitted_points)
        return drop_new_fitted_points

    def get_horizontal_planes(self, plane_points, minx, miny, maxx, maxy):

        plane = [pt for row in plane_points for pt in row]
        '''#print("minx,miny,maxx,maxy : ", minx, miny, maxx, maxy)
        #maxplanex = max(plane,key = lambda x:x[0])[0]
        #minplanex = min(plane,key = lambda x:x[0])[0]
        #maxplaney = max(plane,key = lambda x:x[1])[1]
        #minplaney = min(plane,key = lambda x:x[1])[1]
        # print(maxplanex,maxplaney,minplanex,minplaney)
        # print(self.max_depth,self.min_depth,self.max_width,self.min_width)
        one_depth = ((maxy + 0.005) - (miny - 0.005))/2.0
        one_width = ((maxx + 0.005) - (minx - 0.005))/2.0

        divisor_val_width = 1.0
        divisor_val_depth = 1.0

        while(divisor_val_depth > 0 and divisor_val_width > 0 and (one_depth < 0.05 or one_width < 0.05)):

            if(one_depth < 0.05):
                one_depth = (maxy - miny)/divisor_val_depth
                divisor_val_depth = divisor_val_depth - 1.0

            if(one_width < 0.05):
                one_width = (maxx - minx)/divisor_val_width
                divisor_val_width = divisor_val_width - 1.0

        print("one_width,one_depth : ", one_width, one_depth)
        #print("divisor_val_width, divisor_val_depth",divisor_val_width,divisor_val_depth)

        start_depth = miny - 0.005
        start_width = minx - 0.005
        print("---------------------------------------------------------------------------------------------------------------------------------")
        #print("start_depth,start_width : ",start_depth,start_width)
        True_list_point_occupancy = []
        width_count = 0
        while(start_depth < (maxy + 0.005)):
            #print("start_depth,start_width : ",start_depth,start_width)
            start_width = minx - 0.005
            width_count = 0
            while(start_width < (maxx + 0.005)):

                #print("grid : ",start_depth,start_width)
                count = 0
                width_count = width_count + 1

                for pt in plane:
                    #print(pt[0] , start_width , (start_width+one_width) , pt[1] , start_depth , start_depth+one_depth)
                    if(pt[0] >= start_width and pt[0] < (start_width+one_width) and pt[1] >= start_depth and pt[1] < start_depth+one_depth):
                        count = count + 1
                        #print("count incremented")
                if(count > 0):
                    #print("points occupancy true")
                    True_list_point_occupancy.append(True)
                else:
                    True_list_point_occupancy.append(False)

                start_width = start_width + one_width
            start_depth = start_depth + one_depth

        #print("True_list_point_occupancy : ",True_list_point_occupancy)
        count_occupancy = 0
        for check in True_list_point_occupancy:
            if(check):
                count_occupancy = count_occupancy + 1

        total_occupancy_grids = len(True_list_point_occupancy)

        print("total_occupancy_grids, width_count, count_occupancy : ",
              total_occupancy_grids, width_count, count_occupancy)

        percentage = count_occupancy / (total_occupancy_grids * 1.0)
        th_percentage = 0.90'''

        th_percentage = 0.30
        count_of_straightup_or_down_normals = 0
        count_of_non_zero_points = 0
        for pt in plane:

            count_of_non_zero_points = count_of_non_zero_points + 1

            if((abs(pt[3]) < 0.015 or abs(pt[4]) < 0.015) and abs(pt[5]) > 0.89 and abs(pt[5]) <= 1.0):
                count_of_straightup_or_down_normals += 1

        print("---------------------------------------plane end------------------------------------")
        print("count_of_straightup_or_down_normals : ",
              count_of_straightup_or_down_normals)
        print("len : ", count_of_non_zero_points)
        if(count_of_non_zero_points > 0.0):
            percentage = (count_of_straightup_or_down_normals /
                          float(count_of_non_zero_points))
        else:
            percentage = 0.0
        print("percentage : ", percentage)
        if(percentage > th_percentage):
            return True
        else:
            return False

    def plan_path(self, gridded_cloud_list, minx, miny, maxx, maxy, row_index=0.019):

        horizontal_plane_list = []

        #print("------------------------------------plan path function---------------------------------------------")

        gridded_list_zero_removed = []

        for plane in gridded_cloud_list:
            plane_n = []
            for row in plane:
                row_n = []
                for pt in row:
                    if(pt[1] == 0.0):
                        continue
                    row_n.append(pt)

                if(len(row_n) > 0):
                    plane_n.append(row_n)

            if(len(plane_n) > 0):
                gridded_list_zero_removed.append(plane_n)

        skip_last = True
        print("len(gridded_list_zero_removed) : ",
              len(gridded_list_zero_removed))
        rows = 2
        prevboolval = False
        id = 0
        if(not (len(gridded_list_zero_removed) % rows == 0)):
            skip_last = False

        while(id < (len(gridded_list_zero_removed)-1)):

            boolval = self.get_horizontal_planes(
                gridded_list_zero_removed[id]+gridded_list_zero_removed[id+1], minx, miny, maxx, maxy)
            horizontal_plane_list.append(boolval)
            horizontal_plane_list.append(boolval)
            prevboolval = boolval

            id = id + rows

        if(not skip_last):
            print("len(gridded_list_zero_removed),id-1,id : ",
                  len(gridded_list_zero_removed), id-1, id)
            boolval = self.get_horizontal_planes(
                gridded_list_zero_removed[id-1]+gridded_list_zero_removed[id], minx, miny, maxx, maxy)
            if(prevboolval):
                horizontal_plane_list.append(prevboolval)
            else:
                horizontal_plane_list.append(boolval)

        horizontal_handled_gridded_list_inter = []

        row_approx = 0.01

        ind = 0
        contains_horizontal = False
        new_horizontal_plane_list = []
        while(ind < len(gridded_list_zero_removed)):

            plane_without_zeros = []

            while(ind < len(horizontal_plane_list) and horizontal_plane_list[ind]):
                contains_horizontal = True

                for row in gridded_list_zero_removed[ind]:
                    for pt in row:
                        if(pt[1] == 0.0):
                            continue
                        plane_without_zeros.append(pt)
                ind = ind + 1

            if(contains_horizontal and len(plane_without_zeros) > 0):
                desc_sorted = sorted(plane_without_zeros,
                                     key=lambda x: x[1], reverse=True)

                horizontal_handled_gridded_list_inter.append(desc_sorted)
                contains_horizontal = False
                new_horizontal_plane_list.append(True)
            else:

                horizontal_handled_gridded_list_inter.append(
                    gridded_list_zero_removed[ind])
                ind = ind + 1
                contains_horizontal = False
                new_horizontal_plane_list.append(False)

        print("                                                                                ")
        print("new_horizontal_plane_list : ", new_horizontal_plane_list)
        print("-------------------------------------------------------------------------------------------------------------------")

        horizontal_handled_gridded_list = []
        horizontal_plane = []
        horizontal_plane_list_sequenced = []
        for ind, plane in enumerate(horizontal_handled_gridded_list_inter):

            if(new_horizontal_plane_list[ind]):
                horizontal_plane.append(plane)
                maxy_plane = plane[0][1]

                miny_plane = plane[-1][1]
                iter = maxy_plane
                print("maxy_plane,miny_plane : ", maxy_plane, miny_plane)
                copy_plane = copy.deepcopy(plane)
                while(iter >= miny_plane):
                    row_indexBased = []

                    for pt in copy_plane:

                        if(pt[1] > (iter - row_approx)):

                            row_indexBased.append(pt)

                    print("before iter : ", iter)
                    iter = iter - row_index
                    print("after iter : ", iter)

                    print("len(row_indexBased) : ", len(row_indexBased))
                    copy_plane = [pt for pt in copy_plane if (
                        pt[1] < (iter + row_approx))]

                    if(len(row_indexBased) > 0):

                        print(len(copy_plane), len(row_indexBased))

                        horizontal_handled_gridded_list.append(
                            [row_indexBased])

                        horizontal_plane_list_sequenced.append(True)

            else:
                horizontal_handled_gridded_list.append(plane)
                horizontal_plane_list_sequenced.append(False)

        #if(len(horizontal_plane)):
        #    print(horizontal_plane)

        horizontal_plane_1D = [pt for row in horizontal_plane for pt in row]

        horizontal_handled_gridded_list_1D = [
            pt for plane in horizontal_handled_gridded_list for row in plane for pt in row]

        #print("horizontal_plane_list", horizontal_plane_list_sequenced)
        return horizontal_plane_list_sequenced, horizontal_handled_gridded_list, horizontal_handled_gridded_list_1D, horizontal_plane_1D


    def get_diffy_diffz(self, dy_dz_change_rows):
        #print("dy_dz_change_rows : ",dy_dz_change_rows)
        dy_change_row_avg = []
        dz_change_row_avg = [] 
        for point in dy_dz_change_rows:
                            
                dy_change = []
                dz_change = []
                for dy_dz in point:
                    #print(dy_dz)
                    dy_change.append(dy_dz[1])
                    dz_change.append(dy_dz[2])
                print("dy_change", dy_change)
                print("dz_change", dz_change)
                dy_change_arr = np.array(dy_change)
                dz_change_arr = np.array(dz_change)
                dy_change_row_avg.append(dy_change_arr.mean())
                dz_change_row_avg.append(dz_change_arr.mean())
        diff_dy_change = float(
            dy_change_row_avg[1]-dy_change_row_avg[0])
        diff_dz_change = float(
            dz_change_row_avg[1]-dz_change_row_avg[0])

        return diff_dy_change, diff_dz_change    

    def sequencing(self,  horizontal_handled_gridded_list, horizontal_plane_list, index, minz,minx,maxx):
        count = 1
        row_index = count + 2
        # 5 for sealer,3 for base1, 4 for base2, 6 or 7 for clear 1 and clear 2
        skip_row_count = row_index + index
        data_row_list = []

        toggle = False
        points = []
        reaced_last_point = False
        first_rows_taken = False
        first_two_rows_count = 0
        prev_to_last_point = []
        dy_dz_change_rows = [[], []]
        bottom_dy_dz_change_rows = [[], []]
        modified_horizontal_list = []
        for ind, i in enumerate(horizontal_handled_gridded_list):
            if(horizontal_plane_list[ind]):
                fitted_points = self.two_d_curve_fit(
                    i, z_val=0.0, horizontal=True)

                if toggle:
                    fitted_points = fitted_points[::-1]

                toggle = not toggle
                data_row_list.append(fitted_points.tolist())
                modified_horizontal_list.append(True)

            else:
                if len(horizontal_handled_gridded_list)-1 == count:
                    prev_to_last_point.append(i)
                    #print('dy_dz_change_rows',dy_dz_change_rows)

                if len(horizontal_handled_gridded_list) == count:
                    print("last point reached")
                    reaced_last_point = True

                if first_rows_taken:
                    print("bottom_dy_dz_change_rows : ",bottom_dy_dz_change_rows)   
                    diff_dy_change, diff_dz_change = self.get_diffy_diffz(bottom_dy_dz_change_rows)
                    bottom_index_z_mod = self.bottom_index_row_offset*(diff_dz_change /
                                                      (diff_dy_change+diff_dz_change))


                    bottom_index_y_mod = self.bottom_index_row_offset*(diff_dy_change /
                                                  (diff_dy_change+diff_dz_change))
                                              
                    for dy_dz in bottom_dy_dz_change_rows[0]:
                        dy_dz[1] = dy_dz[1] - bottom_index_y_mod
                        dy_dz[2] = dy_dz[2] - bottom_index_z_mod

                    bottom_index_fitted_points = np.array(bottom_dy_dz_change_rows[0])                            
                    
                    bottom_index_fitted_points = bottom_index_fitted_points[::-1]

                
                    data_row_list.append(bottom_index_fitted_points.tolist())   
                    print("------------ADDED BOTTOM INDEX ROW---------------")
                    first_rows_taken = False

                if count <= skip_row_count:
                    if reaced_last_point:

                        diff_dy_change, diff_dz_change = self.get_diffy_diffz(dy_dz_change_rows)
                        
                        print("diff_dy_change", diff_dy_change)
                        print("diff_dz_change", diff_dz_change)

                        print("last point reached -- not consider count")
                        points.append(i)
                        print("index", index)
                        print("count", count)
                        #print("consider_count", consider_count)
                        #print("not_consider_count", not_consider_count)

                        index_z_mod = (index+1)*0.01*(diff_dz_change /
                                                      (diff_dy_change+diff_dz_change))
                        #z_val = minz + ((consider_count+index_z_mod)*0.01)

                        index_y_mod = (index+1)*0.01*(diff_dy_change /
                                                      (diff_dy_change+diff_dz_change))

                        print(
                            "Before -----------------------   dy_dz_change_rows[1]", dy_dz_change_rows[1])
                        print("index_y_mod", index_y_mod)
                        print("index_z_mod", index_z_mod)
                        for dy_dz in dy_dz_change_rows[1]:
                            dy_dz[1] = dy_dz[1] + index_y_mod
                            dy_dz[2] = dy_dz[2] + index_z_mod

                        print(
                            "After -----------------------   dy_dz_change_rows[1]", dy_dz_change_rows[1])
                        fitted_points = np.array(dy_dz_change_rows[1])
 
                        toggle = not toggle
                        if toggle:
                            fitted_points = fitted_points[::-1]

                        
                        data_row_list.append(fitted_points.tolist())

                        ### Adding top row indexing ###

                        top_index_z_mod = self.top_index_row_offset*(diff_dz_change /
                                                      (diff_dy_change+diff_dz_change))


                        top_index_y_mod = self.top_index_row_offset*(diff_dy_change /
                                                      (diff_dy_change+diff_dz_change))
                                                      
                        for dy_dz in dy_dz_change_rows[1]:
                            dy_dz[1] = dy_dz[1] + top_index_y_mod
                            dy_dz[2] = dy_dz[2] + top_index_z_mod

                        top_index_fitted_points = np.array(dy_dz_change_rows[1])           
                        toggle = not toggle                 
                        if toggle:
                            top_index_fitted_points = top_index_fitted_points[::-1]

                        
                        data_row_list.append(top_index_fitted_points.tolist())

                        
                        modified_horizontal_list.append(False)
                        print("added last row")
                        points = []

                    elif count == skip_row_count:
                        row_index = count + 2
                        # 5 for sealer,3 for base1, 4 for base2, 6 or 7 for clear 1 and clear 2
                        skip_row_count = row_index + index

                if count < row_index:
                    points.append(i)
                    if len(points) > 1:  # or reaced_last_point:
                        z_val = minz + (count*0.01)-0.01

                        if len(points) > 1:
                            fitted_points = self.two_d_curve_fit(
                                points[0]+points[1], z_val, 0)

                        if len(fitted_points) == 0:
                            points = []
                            count += 1
                            continue
                        if toggle:
                            fitted_points = fitted_points[::-1]

                        toggle = not toggle
                        data_row_list.append(fitted_points.tolist())
                        modified_horizontal_list.append(False)
                        dy_dz_change_rows[0] = dy_dz_change_rows[1]
                        dy_dz_change_rows[1] = fitted_points.tolist()
                        if(first_two_rows_count < 2):
                            bottom_dy_dz_change_rows[0] = bottom_dy_dz_change_rows[1]
                            bottom_dy_dz_change_rows[1] = fitted_points.tolist()
                            first_two_rows_count = first_two_rows_count + 1
                            if(first_two_rows_count > 1):
                                first_rows_taken = True
                        points = []
               
                count += 1



        #print("len(data_row_list) , len(modified_horizontal_list) : ", len(data_row_list) , len(modified_horizontal_list)) 

        #print("                                             ")
        #print("modified_horizontal_list : ",modified_horizontal_list)
        '''new_data_row_list_with_bool = []
        for ind, row in enumerate(data_row_list):
            if(modified_horizontal_list[ind]):
                new_data_row_list_with_bool.append([row,True]) 
            else:
                new_data_row_list_with_bool.append([row,False])''' # will be useful for sequencing horiz and verti in future
               
        data_row_list = sorted(data_row_list , key = lambda x:x[0][2], reverse = True)      

        # horizontal = []

        # vertical = []
        '''for ind, row in enumerate(data_row_list):           # will be useful for sequencing horiz and verti in future
             #print("row[0][2] : ",row[0][2])
             actual_row = []
             if(row[1]):
                row_pts = row[0]
                actual_row = sorted(row_pts, key = )
             else:
                actual_row = row[0]'''
        # vertical.reverse()

        # for ind, row in enumerate(vertical):
        #     row.reverse()      
        order_list = []
        #print("data_row_list",data_row_list)
        new_data_row_list = copy.deepcopy(data_row_list)
        for idx, row in enumerate(data_row_list):

            if(row[0][0] > row[1][0]):
                order_list.append('desc')
            else:
                order_list.append('asc')

            '''if(len(data_row_list)-1 == idx):
                print(row)
            if(len(data_row_list)-2 == idx):
                print(row)    '''

        #print("order_list : ",order_list)            

        ord_data_row_list = [None]*(len(order_list)+len(data_row_list))
        ord_data_row_list[::2] = order_list
        ord_data_row_list[1::2] = data_row_list

        #print("ord_data_row_list",ord_data_row_list)
        ord_data_row_list_combo = []
        while len(ord_data_row_list)>0:
            ord_data_row_list_combo.append([ord_data_row_list[0],[ord_data_row_list[1]]])
            ord_data_row_list.pop(0)
            ord_data_row_list.pop(0)

        #print("ord_data_row_list_combo",ord_data_row_list_combo)        

        data_row_list_with_index_offsets = []

        for idx, row in enumerate(ord_data_row_list_combo):
            print("maxx : ",maxx)
            print("minx : ",minx)
            #print("row0",data_row_list[0])
            
            if row[0] == 'desc':
                #print("before row[1]",row[1])
                left_point = row[1][0][-1]
                left_point_adj = row[1][0][-2]
                right_point = row[1][0][0]
                right_point_adj = row[1][0][1]

                dx_left = abs(left_point[0] - left_point_adj[0])
                dy_left = abs(left_point[1] - left_point_adj[1]) 

                dx_right = abs(right_point[0] - right_point_adj[0])
                dy_right = abs(right_point[1] - right_point_adj[1])


                row_left_index_x_mod = self.left_right_index_offset*(dx_left /
                                                      (dy_left+dx_left))


                row_left_index_y_mod = self.left_right_index_offset*(dy_left /
                                                      (dy_left+dx_left))

                row_right_index_x_mod = self.left_right_index_offset*(dx_right /
                                                      (dy_right+dx_right))


                row_right_index_y_mod = self.left_right_index_offset*(dy_right /
                                                      (dy_right+dx_right))

                left_point_added = copy.deepcopy(left_point)
                left_point_added[0]= left_point_added[0] - row_left_index_x_mod
                left_point_added[1]= left_point_added[1] + row_left_index_y_mod


                right_point_added = copy.deepcopy(right_point)
                right_point_added[0]= right_point_added[0] + row_right_index_x_mod
                right_point_added[1]= right_point_added[1] + row_right_index_y_mod


                #if left_point_added[0]<maxx:
                #left_point_added[0]=maxx

                #if right_point_added[0]>minx:
                #right_point_added[0]=minx

                #row[1][0].insert(0,right_point_added)
                #print("new_data_row_list[idx]",new_data_row_list[idx])
                new_data_row_list[idx].insert(0,right_point_added)
                #row[1][0].append(left_point_added)
                new_data_row_list[idx].append(left_point_added)
                #print("after new_data_row_list[idx]",new_data_row_list[idx])


                #print("after row[1]",row[1])


            elif row[0] == 'asc':
                left_point = row[1][0][0]
                left_point_adj = row[1][0][1]
                right_point = row[1][0][-1]
                right_point_adj = row[1][0][-2]

                dx_left = abs(left_point[0] - left_point_adj[0])
                dy_left = abs(left_point[1] - left_point_adj[1]) 

                dx_right = abs(right_point[0] - right_point_adj[0])
                dy_right = abs(right_point[1] - right_point_adj[1])


                row_left_index_x_mod = self.left_right_index_offset*(dx_left /
                                                      (dy_left+dx_left))


                row_left_index_y_mod = self.left_right_index_offset*(dy_left /
                                                      (dy_left+dx_left))

                row_right_index_x_mod = self.left_right_index_offset*(dx_right /
                                                      (dy_right+dx_right))


                row_right_index_y_mod = self.left_right_index_offset*(dy_right /
                                                      (dy_right+dx_right))

                left_point_added = copy.deepcopy(left_point)
                left_point_added[0]= left_point_added[0] - row_left_index_x_mod
                left_point_added[1]= left_point_added[1] + row_left_index_y_mod

                right_point_added = copy.deepcopy(right_point)
                right_point_added[0]= right_point_added[0] + row_right_index_x_mod
                right_point_added[1]= right_point_added[1] + row_right_index_y_mod

                #if left_point_added[0]>minx:
                #left_point_added[0]=minx

                #if right_point_added[0]<maxx:
                #right_point_added[0]=maxx

                #row[1][0].insert(0,right_point_added)
                #print("new_data_row_list[idx]",new_data_row_list[idx])
                new_data_row_list[idx].insert(0,left_point_added)
                #row[1][0].append(left_point_added)
                new_data_row_list[idx].append(right_point_added)
                #print("after new_data_row_list[idx]",new_data_row_list[idx])

 



            #left_added_point = copy.deepcopy(row[0][])
            #left_added_point[0] = 

            
            #if(order_list[idx] == "asc"):
            #exit()


        #print("new_data_row_list",new_data_row_list)   
            
        data_row_list_mod = [pt for row in new_data_row_list for pt in row]
        # modifiying sequencing from top to bottom
        data_row_list_mod.reverse() 

        data_row_path_list = Path()

        for ind, row in enumerate(new_data_row_list):
            #print("row[0][2] : ",row[0][2])
            data_row_point_list = PathStamped()

            for pt in row:
                data_row_point_append = Plannedpath()

                data_row_point_append.x = pt[0]
                data_row_point_append.y = pt[1]
                data_row_point_append.z = pt[2]
                data_row_point_append.ox = pt[3]
                data_row_point_append.oy = pt[4]
                data_row_point_append.oz = pt[5]
                data_row_point_append.ow = 0.0
                data_row_point_append.is_index = False

                data_row_point_list.path_msg.append(data_row_point_append)

            data_row_path_list.path.append(data_row_point_list)

        return data_row_list_mod, data_row_path_list

    def start(self):

        rate = rospy.Rate(10)
        print("Waiting for coat_process to be true")
        while not rospy.is_shutdown():

            # if rospy.get_param("coat_process")==True and self.received_sealer and self.received_base1 and self.received_base2 and self.received_clear1 and self.received_clear2 and not self.processed:
            ######### index always greater then 2 centimeter ################################
            # 5 for sealer,3 for base1, 4 for base2, 7 for clear 1 and clear 2
            #self.sealer_grid_width = rospy.get_param("sealer_grid_width")

            #self.sealer_grid_depth = rospy.get_param("sealer_grid_depth")
            #self.basecoat1_grid_width = rospy.get_param("basecoat1_grid_width")

            #self.basecoat1_grid_depth = rospy.get_param("basecoat1_grid_depth")
            #self.basecoat2_grid_width = rospy.get_param("basecoat2_grid_width")

            #self.basecoat2_grid_depth = rospy.get_param("basecoat2_grid_depth")
            #self.clearcoat1_grid_width = rospy.get_param("clearcoat1_grid_width")

            #self.clearcoat1_grid_depth = rospy.get_param("clearcoat1_grid_depth")
            #self.clearcoat2_grid_width = rospy.get_param("clearcoat2_grid_width")

            #self.clearcoat2_grid_depth = rospy.get_param("clearcoat2_grid_depth")
            #print("inside function ")

            #self.processed = True

            if (self.received_sealer and self.received_base1 and self.received_base2 and self.received_clear1 and self.received_clear2):
                #print("publishing ... ")
                self.griddedCloud_pc.publish(self.sealer_gridded_cloud)
                self.sealercoat_pc.publish(self.sealer_data_seq_cloud)

                # self.basecoat1_pc.publish(self.basecoat1_data_seq_cloud)
                # self.basecoat2_pc.publish(self.basecoat2_data_seq_cloud)
                # self.clearcoat1_pc.publish(self.clearcoat1_data_seq_cloud)
                # self.clearcoat2_pc.publish(self.clearcoat2_data_seq_cloud)
                self.horizontal_pc.publish(self.horizontal_cloud)
                self.sealercoat.publish(self.sealer_data_row_path_list)
                # self.basecoat1.publish(self.basecoat1_data_row_path_list)
                # self.basecoat2.publish(self.basecoat2_data_row_path_list)
                # self.clearcoat1.publish(self.clearcoat1_data_row_path_list)
                # self.clearcoat2.publish(self.clearcoat2_data_row_path_list)

            rate.sleep()


if __name__ == '__main__':
    PathPlanning()
    rospy.spin()
