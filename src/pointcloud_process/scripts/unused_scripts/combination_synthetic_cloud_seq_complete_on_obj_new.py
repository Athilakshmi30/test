#!/usr/bin/env python
from __future__ import print_function
#from textwrap import indent
from traceback import print_tb

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


class SyntheticCloud():

    def __init__(self):

        rospy.init_node('synthetic_cloud')

        # self.pub = rospy.Publisher(
        #    "/max_depth_added_cloud", PointCloud2, queue_size=1)

        # self.pub1 = rospy.Publisher(
        #    "/occlusion_removed_cloud", PointCloud2, queue_size=1)

        self.griddedCloud_pc = rospy.Publisher(
            "/gridded_cloud_pc", PointCloud2, queue_size=1)

        # self.pub3 = rospy.Publisher(
        #    "/synthetic_cloud", PointCloud2, queue_size=1)

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

        #self.trajectory_path = rospy.Publisher("/test_trajectory_path", Path, queue_size=1)

        rospy.Subscriber("/points_and_normals",
                         PointCloud2, self.transformedDataCallBack)

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

        self.FIELDS_XYZ = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        self.FIELDS_XYZRGB = self.FIELDS_XYZ + \
            [PointField(name='rgb', offset=12,
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

        self.point_cloud_list = []
        self.max_depth_added_cloud_list = []
        self.max_depth_added_cloud = PointCloud2()

        self.occlusion_removed_cloud_list = []
        self.occlusion_removed_cloud = PointCloud2()

        self.gridded_cloud = PointCloud2()
        self.synthetic_cloud = PointCloud2()

        self.max_depth = -10.0
        self.min_depth = 10.0

        self.max_height = -10.0
        self.min_height = 10.0

        self.max_width = -10.0
        self.min_width = 10.0

        self.depth_width = 0.254  # 10 inches

        self.sealer_row_index = 0.0635
        self.base1_row_index = 0.0381
        self.base2_row_index = 0.0508
        self.clear1_row_index = 0.0762
        self.clear2_row_index = 0.0762

        self.left_right_index_offset = 0.0762  # 3 inches #0.1524 - 6 inches

        self.top_bottom_index_row_offset = 0.0762  # 3 inches #0.1524 - 6 inches

        self.grid_width = 0.01
        self.grid_height = 0.01
        self.grid_depth = 0.01
        self.received = False
        self.processed = False

        #self.points = Plannedpath()
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

    def inHull(self, p, hull):
        """
        Test if points in `p` are in `hull`

        `p` should be a `NxK` coordinates of `N` points in `K` dimensions
        `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the 
        coordinates of `M` points in `K`dimensions for which Delaunay triangulation
        will be computed
        """

        if not isinstance(hull, Delaunay):
            hull = Delaunay(hull)

        return hull.find_simplex(p) >= 0

    def transformedDataCallBack(self, msg):

        if(not self.received):
            # print(msg.fields)
            self.point_cloud_list = list(point_cloud2.read_points(
                msg, field_names=['x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z'], skip_nans=True))
            self.received = True

    def addDepthPlane(self, cloud):

        pc2_list = cloud
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

            if(maxy < y):
                maxy = y
            if(miny > y):
                miny = y
            if(maxx < x):
                maxx = x
            if(minx > x):
                minx = x
            if(maxz < z):
                maxz = z
            if(minz > z):
                minz = z

        #*********************************max  min  for grid***************************#
        self.max_depth = miny + self.depth_width
        self.min_depth = miny - self.depth_width
        self.max_width = maxx + self.left_right_index_offset
        self.min_width = minx - self.left_right_index_offset
        self.max_height = maxz + self.top_bottom_index_row_offset
        self.min_height = minz - self.top_bottom_index_row_offset
        #********************************************************************************#

        depth_plane = maxy + 0.0254  # 1 inch
        depth_added_cloud = cloud

        runx = minx - self.left_right_index_offset
        endx = maxx + self.left_right_index_offset

        stepx = 0.01
        stepz = 0.01

        runz = minz - self.top_bottom_index_row_offset
        endz = maxz + self.top_bottom_index_row_offset
        '''
        while(runz <= endz):
            runx = minx - self.left_right_index_offset
            while(runx <= endx):
                depth_added_cloud.append([runx, depth_plane, runz])
                runx = runx + stepx
            runz = runz + stepz
        '''
        return depth_added_cloud

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
            (pt[2] < (z_range + 0.006)) and (pt[2] > (z_range - 0.006)))]
        #print("length ---------------------------- : ", len(cropped_cloud))
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
        maxy = -1.0
        miny = 10.0
        maxx = -1.0
        minx = 10.0
        minz = 10.0
        maxz = -1.0

        for data in cloud:
            x = data[0]
            y = data[1]
            z = data[2]

            if(maxy < y):
                maxy = y
            if(miny > y):
                miny = y
            if(maxx < x):
                maxx = x
            if(minx > x):
                minx = x
            if(maxz < z):
                maxz = z
            if(minz > z):
                minz = z

        #self.grid_width = grid_width
        #self.grid_height = grid_height
        #self.grid_depth = grid_depth

        self.grid_width = 0.01
        self.grid_height = 0.01
        self.grid_depth = 0.01
        #print("start time : ",time.time())
        #print("length of input cloud : ", len(cloud))
        total = 0.0
        boolean_grid = []
        gridded_cloud = []
        synthetic_cloud = []
        current_x = self.min_width - 0.005
        current_y = self.min_depth - 0.005
        current_z = self.min_height - 0.005
        cropped_cloud = cloud

        print("********************************************creating grid**********************************************************")
        while(current_z <= self.max_height + 0.005):

            current_y = self.min_depth - 0.005
            depth_row = []
            depth_boolean_row = []
            depth_synthetic_row = []

            z_range = current_z
            print("Z_range : ", z_range)
            cropped_cloud = self.get_points_in_range(cloud, z_range)

            print("[ITERATION] ------------------------------------ ")
            if(len(cropped_cloud) > 1):
                while(current_y <= self.max_depth + 0.005):

                    current_x = self.min_width - 0.005
                    width_row = []
                    width_synthetic_row = []
                    width_boolean_row = []

                    while(current_x <= self.max_width + 0.005):
                        conv_hull = [[current_x, current_y, current_z], [current_x+self.grid_width, current_y, current_z], [current_x, current_y, current_z+self.grid_height], [current_x+self.grid_width, current_y, current_z+self.grid_height], [current_x, current_y +
                                                                                                                                                                                                                                                    self.grid_depth, current_z], [current_x+self.grid_width, current_y+self.grid_depth, current_z], [current_x, current_y+self.grid_depth, current_z+self.grid_height], [current_x+self.grid_width, current_y+self.grid_depth, current_z+self.grid_height]]

                        start = time.time()
                        #print("len(cropped_cloud) : ",len(cropped_cloud))
                        bool_list = self.inHull(cropped_cloud, conv_hull)
                        ending = time.time()

                        total = total + ending - start
                        current_list = list(compress(cropped_cloud, bool_list))

                        if(len(current_list) > 0):
                            pt_n = []
                            for pt in cloud:
                                if(abs(pt[0] - current_list[0][0]) <= 0.001 and abs(pt[1] - current_list[0][1]) <= 0.001 and abs(pt[2] - current_list[0][2]) <= 0.001):
                                    pt_n = list(pt)
                                    break
                            width_row.append(pt_n)
                            width_boolean_row.append(True)
                            width_synthetic_row.append(
                                self.centroid(conv_hull))

                        else:
                            width_boolean_row.append(False)
                            width_row.append([0, 0, 0, 0, 0, 0])
                            width_synthetic_row.append([0, 0, 0, 0, 0, 0])

                        current_x = current_x + self.grid_width

                    depth_row.append(width_row)
                    depth_boolean_row.append(width_boolean_row)
                    depth_synthetic_row.append(width_synthetic_row)
                    current_y = current_y + self.grid_depth

                gridded_cloud.append(depth_row)
                boolean_grid.append(depth_boolean_row)
                synthetic_cloud.append(depth_synthetic_row)

            current_z = current_z + self.grid_height

        gridded_cloud_linear = [
            pt for row in gridded_cloud for col in row for pt in col]
        synthetic_cloud_linear = [
            pt for row in synthetic_cloud for col in row for pt in col]

        #print("End time : ",time.time())

        #print("Total createGrid: ",total)

        return gridded_cloud, boolean_grid, gridded_cloud_linear, synthetic_cloud_linear, synthetic_cloud, minz

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

        while(curr_plane >= 0):

            curr_depth_row = 0
            plane = []
            while((curr_depth_row <= (len_of_depthrows_in_each_plane - 1))):

                curr_point = 0
                row = []
                while(curr_point <= (len_of_each_depthrow - 1)):

                    boolean_point = boolean_grid[curr_plane][curr_depth_row][curr_point]
                    if(boolean_point):
                        row.append([curr_plane, curr_depth_row, curr_point])

                    curr_point += 1

                if(len(row) > 0):
                    plane.append(row)

                curr_depth_row += 1

            if(len(plane) > 0):
                path_list.append(plane)
            curr_plane -= 1

        ending = time.time()
        #print("Ending : ",ending)
        #print("Total points for path : ",ending-start)

        return path_list, boolean_grid

    # , gridded_cloud_list, synthetic_cloud_list):

    def curve_fit(self, data_list, z_val, horizontal=False):

        X_data = []
        Y_data = []
        Z_data = []
        check_z = []
        print("z_val", z_val)
        print("curve_fit called")
        for rows in data_list:
            # print("points",points)
            for pt in rows:
                check_z.append(pt[2])
                if pt[0] == 0:
                    continue
                X_data.append(pt[0])
                Y_data.append(pt[1])

                if(not horizontal):
                    Z_data.append(z_val)
                else:
                    Z_data.append(pt[2])

        Z_data_arr = np.array(Z_data)

        print("Z_data_arr", Z_data_arr.mean())
        print("len(X_data)", len(X_data))
        if len(X_data) == 0:
            check_z_arr = np.array(check_z)
            print("No X,Y point data for Z = ", check_z_arr.mean())
        # 6deg
        a, b, c, d, e, f, g = np.polyfit(X_data, Y_data, 6)
        def fit_equation(x): return a * x ** 6 + b * \
            x ** 5 + c*x**4+d*x**3+e*x**2+f*x+g

        avg_z = Z_data_arr.mean()
        fitted_points = []
        # print("data_list",data_list)
        # print("X_data",X_data)

        for row in data_list:
            for pt in row:
                if pt[0] == 0:
                    continue
                #print("pt : ",pt)
                for ind, xcoord in enumerate(X_data):
                    if(not horizontal):
                        if(abs(xcoord - pt[0]) < 0.001 and abs(Y_data[ind] - pt[1]) < 0.001 and abs(Z_data[ind] - z_val) < 0.001):
                            fitted_points.append(
                                [xcoord, fit_equation(xcoord), avg_z, pt[3], pt[4], pt[5]])
                    else:
                        if(abs(xcoord - pt[0]) < 0.001 and abs(Y_data[ind] - pt[1]) < 0.001 and abs(Z_data[ind] - pt[2]) < 0.001):
                            fitted_points.append(
                                [xcoord, fit_equation(xcoord), pt[2], pt[3], pt[4], pt[5]])

        fitted_points = sorted(fitted_points, key=lambda x: x[0])
        fitted_points = np.array(fitted_points)
        # print("fitted_points",fitted_points)
        return fitted_points

    def get_horizontal_planes(self, plane_points):

        plane = [pt for row in plane_points for pt in row]

        percentage = 0.0
        th_percentage = 0.49

        count_of_straightup_or_down_normals = 0
        #print("---------------------------------------plane start------------------------------------")
        count_of_non_zero_points = 0
        for pt in plane:

            count_of_non_zero_points = count_of_non_zero_points + 1
            if(abs(pt[3]) < 0.01 and abs(pt[4]) < 0.01 and abs(pt[5]) > 0.899 and abs(pt[5]) <= 1.0):
                count_of_straightup_or_down_normals += 1

        #print("---------------------------------------plane end------------------------------------")
        #print("count_of_straightup_or_down_normals : ",count_of_straightup_or_down_normals)
        #print("len : ",count_of_non_zero_points)
        if(count_of_non_zero_points > 0.0):
            percentage = (count_of_straightup_or_down_normals /
                          float(count_of_non_zero_points))
        else:
            percentage = 0.0
        #print("percentage : ", percentage)
        if(percentage > th_percentage):
            return True
        else:
            return False

    def plan_path(self, gridded_cloud_list, row_index=0.019):

        #row_index = self.row_index_sealer
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

        #print("gridded_list_zero_removed : ",gridded_list_zero_removed)

        for plane in gridded_list_zero_removed:
            #print("plane i")
            horizontal_plane_list.append(self.get_horizontal_planes(plane))

        #print("horizontal_plane_list", horizontal_plane_list)

        horizontal_handled_gridded_list_inter = []

        row_approx = 0.01

        for ind, plane in enumerate(gridded_list_zero_removed):

            plane_without_zeros = []

            if(horizontal_plane_list[ind]):

                for row in plane:

                    for pt in row:
                        plane_without_zeros.append(pt)

                #print("plane_without_zeros : " , plane_without_zeros)
                desc_sorted = sorted(plane_without_zeros,
                                     key=lambda x: x[1], reverse=True)

                #print("plane_without_zeros : " , desc_sorted)

                horizontal_handled_gridded_list_inter.append(desc_sorted)

            else:

                horizontal_handled_gridded_list_inter.append(plane)

        horizontal_handled_gridded_list = []
        horizontal_plane = []
        new_horizontal_plane_list = []
        for ind, plane in enumerate(horizontal_handled_gridded_list_inter):
            #plane_without_zeros = []
            if(horizontal_plane_list[ind]):
                horizontal_plane.append(plane)
                maxy_plane = plane[0][1]

                miny_plane = plane[-1][1]
                iter = maxy_plane
                print("maxy_plane,miny_plane : ", maxy_plane, miny_plane)
                copy_plane = plane
                while(iter >= miny_plane):
                    row_indexBased = []

                    for pt in copy_plane:

                        # print(pt)
                        if(pt[1] > (iter - row_approx)):
                            #print(pt[1] , (iter - row_approx))
                            row_indexBased.append(pt)

                    print("before iter : ", iter)
                    iter = iter - row_index
                    print("after iter : ", iter)
                    #print("copy_plane : ",copy_plane)
                    #print("row_indexBased : ",row_indexBased)
                    print("len(row_indexBased) : ", len(row_indexBased))
                    copy_plane = [pt for pt in copy_plane if (
                        pt[1] < (iter + row_approx))]

                    if(len(row_indexBased) > 0):

                        # plane_without_zeros.append(row_indexBased)

                        print(len(copy_plane), len(row_indexBased))

                        horizontal_handled_gridded_list.append(
                            [row_indexBased])

                        new_horizontal_plane_list.append(True)

            else:
                horizontal_handled_gridded_list.append(plane)
                new_horizontal_plane_list.append(False)

        #print("horizontal_handled_gridded_list : ",horizontal_handled_gridded_list)
        # horizontal_only_gridded_list_1D = [pt for ind, plane in enumerate(
        #    horizontal_handled_gridded_list) if(horizontal_plane_list[ind]) for row in plane for pt in row]

        horizontal_plane_1D = [pt for row in horizontal_plane for pt in row]
        #print("horizontal_only_gridded_list_1D : ", horizontal_only_gridded_list_1D)

        horizontal_handled_gridded_list_1D = [
            pt for plane in horizontal_handled_gridded_list for row in plane for pt in row]

        #print("horizontal_handled_gridded_list_1D : ",horizontal_handled_gridded_list_1D)
        print("horizontal_plane_list", new_horizontal_plane_list)
        return new_horizontal_plane_list, horizontal_handled_gridded_list, horizontal_handled_gridded_list_1D, horizontal_plane_1D

    def sequencing(self,  horizontal_handled_gridded_list, horizontal_plane_list, index, minz):
        count = 1
        consider_count = count + 2
        # 5 for sealer,3 for base1, 4 for base2, 6 or 7 for clear 1 and clear 2
        not_consider_count = consider_count + index
        data_row_list = []
        #X_list = []
        #Y_list = []
        toggle = False
        points = []
        for ind, plane in enumerate(horizontal_handled_gridded_list):

            if(horizontal_plane_list[ind]):
                fitted_points = self.curve_fit(
                    plane, z_val=0.0, horizontal=True)
                # print("fitted_points",fitted_points)
                # print("Y_fit",Y_fit)
                if toggle:
                    fitted_points = fitted_points[::-1]
                    #Y_fit = Y_fit[::-1]
                    # print("after_rev_fitted_points",fitted_points)
                    # print("after_rev_Y_fit",Y_fit)

                toggle = not toggle
                data_row_list.append(fitted_points.tolist())

            else:
                if count == not_consider_count:
                    print("Count updated", count)
                    consider_count = count + 2
                    # 5 for sealer,3 for base1, 4 for base2, 6 or 7 for clear 1 and clear 2
                    not_consider_count = consider_count + index

                if count < consider_count:
                    print("count", count)
                    points.append(plane)
                    # print("points",points)
                    if len(points) > 1:
                        print("insde")
                        # print("points[0]+points[1]",points[0]+points[1])
                        z_val = minz + (count*0.01)-0.01
                        fitted_points = self.curve_fit(
                            points[0]+points[1], z_val)
                        # print("fitted_points",fitted_points)
                        # print("Y_fit",Y_fit)
                        if toggle:
                            fitted_points = fitted_points[::-1]
                            #Y_fit = Y_fit[::-1]
                            # print("after_rev_fitted_points",fitted_points)
                            # print("after_rev_Y_fit",Y_fit)

                        toggle = not toggle
                        data_row_list.append(fitted_points.tolist())
                        points = []

                count += 1
        # print("data_row_list",data_row_list)

        data_row_list_mod = [pt for row in data_row_list for pt in row]
         # modifiying sequencing from top to bottom
        data_row_list_mod.reverse()
        #print("data_row_list_mod",data_row_list_mod)


        #print("data_row_list_mod",data_row_list_mod)       
        data_row_path_list = Path()

        # modifiying sequencing from top to bottom
        for row in reversed(data_row_list):
            
            data_row_point_list = PathStamped()
            for pt in reversed(row):
                data_row_point_append = Plannedpath()
                #print("pt",pt)
                data_row_point_append.x = pt[0]
                data_row_point_append.y = pt[1]
                data_row_point_append.z = pt[2]
                data_row_point_append.ox = pt[3]
                data_row_point_append.oy = pt[4]
                data_row_point_append.oz = pt[5]
                data_row_point_append.is_index = False
                #print("data_row_path_append",data_row_point_append)
                data_row_point_list.path_msg.append(data_row_point_append)
            #print("data_row_point_list",data_row_point_list)

            data_row_path_list.path.append(data_row_point_list)

        return data_row_list_mod, data_row_path_list

    def start(self):

        rate = rospy.Rate(10)
        print("Waiting for coat_process to be true")
        while not rospy.is_shutdown():

            if rospy.get_param("coat_process") == True and self.received and not self.processed:
                ######### index always greater then 2 centimeter ################################
                # 5 for sealer,3 for base1, 4 for base2, 7 for clear 1 and clear 2
                #self.sealer_grid_width = rospy.get_param("sealer_grid_width")
                self.sealer_grid_height = rospy.get_param("sealer_grid_height")
                #self.sealer_grid_depth = rospy.get_param("sealer_grid_depth")
                #self.basecoat1_grid_width = rospy.get_param("basecoat1_grid_width")
                self.basecoat1_grid_height = rospy.get_param(
                    "basecoat1_grid_height")
                #self.basecoat1_grid_depth = rospy.get_param("basecoat1_grid_depth")
                #self.basecoat2_grid_width = rospy.get_param("basecoat2_grid_width")
                self.basecoat2_grid_height = rospy.get_param(
                    "basecoat2_grid_height")
                #self.basecoat2_grid_depth = rospy.get_param("basecoat2_grid_depth")
                #self.clearcoat1_grid_width = rospy.get_param("clearcoat1_grid_width")
                self.clearcoat1_grid_height = rospy.get_param(
                    "clearcoat1_grid_height")
                #self.clearcoat1_grid_depth = rospy.get_param("clearcoat1_grid_depth")
                #self.clearcoat2_grid_width = rospy.get_param("clearcoat2_grid_width")
                self.clearcoat2_grid_height = rospy.get_param(
                    "clearcoat2_grid_height")
                #self.clearcoat2_grid_depth = rospy.get_param("clearcoat2_grid_depth")

                ############################ Sealer ########################
                print("Sealer")
                #self.sealer_coat_data = self.start(self.sealer_grid_width,self.sealer_grid_height,self.sealer_grid_depth)
                self.common_max_depth_added_cloud_list = self.addDepthPlane(
                    self.point_cloud_list)
                common_plane_data_list, common_boolean_grid, common_gridded_cloud_list, common_synthetic_cloud_list, common_synthetic_cloud, minz = self.createGrid(
                    self.point_cloud_list)

                horizontal_plane_list, horizontal_handled_gridded_list, horizontal_handled_gridded_list_1D, horizontal_list = self.plan_path(
                    common_plane_data_list, self.sealer_row_index)
                #print("selaer len",len(sealer_plane_data_list))
                sealer_data_row_list, sealer_data_row_path_list = self.sequencing(
                    horizontal_handled_gridded_list, horizontal_plane_list, self.sealer_grid_height, minz)
                self.sealer_gridded_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_normal, common_gridded_cloud_list)
                self.sealer_data_seq_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_normal, sealer_data_row_list)

                #################################################################

                ############################ Basecoat1 ########################
                print("Basecoat1")
                horizontal_plane_list, horizontal_handled_gridded_list, horizontal_handled_gridded_list_1D, horizontal_list = self.plan_path(
                    common_plane_data_list, self.base1_row_index)
                basecoat1_data_row_list, basecoat1_data_row_path_list = self.sequencing(
                    horizontal_handled_gridded_list, horizontal_plane_list, self.basecoat1_grid_height, minz)
                self.basecoat1_gridded_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_normal, common_gridded_cloud_list)
                self.basecoat1_data_seq_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_normal, basecoat1_data_row_list)
                #################################################################

                ############################ Basecoat2 ########################
                print("Basecoat2")
                horizontal_plane_list, horizontal_handled_gridded_list, horizontal_handled_gridded_list_1D, horizontal_list = self.plan_path(
                    common_plane_data_list, self.base2_row_index)
                basecoat2_data_row_list, basecoat2_data_row_path_list = self.sequencing(
                    horizontal_handled_gridded_list, horizontal_plane_list, self.basecoat2_grid_height, minz)
                self.basecoat2_gridded_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_normal, common_gridded_cloud_list)
                self.basecoat2_data_seq_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_normal, basecoat2_data_row_list)
                #################################################################

                ############################ Clearcoat1 ########################
                print("Clearcoat1")
                horizontal_plane_list, horizontal_handled_gridded_list, horizontal_handled_gridded_list_1D, horizontal_list = self.plan_path(
                    common_plane_data_list, self.clear1_row_index)
                clearcoat1_data_row_list, clearcoat1_data_row_path_list = self.sequencing(
                    horizontal_handled_gridded_list, horizontal_plane_list, self.clearcoat1_grid_height, minz)
                self.clearcoat1_gridded_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_normal, common_gridded_cloud_list)
                self.clearcoat1_data_seq_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_normal, clearcoat1_data_row_list)
                #################################################################

                ############################ Clearcoat2 ########################
                print("Clearcoat2")
                horizontal_plane_list, horizontal_handled_gridded_list, horizontal_handled_gridded_list_1D, horizontal_list = self.plan_path(
                    common_plane_data_list, self.clear2_row_index)
                clearcoat2_data_row_list, clearcoat2_data_row_path_list = self.sequencing(
                    horizontal_handled_gridded_list, horizontal_plane_list, self.clearcoat2_grid_height, minz)
                self.clearcoat2_gridded_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_normal, common_gridded_cloud_list)
                self.clearcoat2_data_seq_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_normal, clearcoat2_data_row_list)
                #################################################################

                self.processed = True

            if(self.processed):
                # self.griddedCloud.publish(self.basecoat1_gridded_cloud)
                self.sealercoat_pc.publish(self.sealer_data_seq_cloud)
                self.basecoat1_pc.publish(self.basecoat1_data_seq_cloud)
                self.basecoat2_pc.publish(self.basecoat2_data_seq_cloud)
                self.clearcoat1_pc.publish(self.clearcoat1_data_seq_cloud)
                self.clearcoat2_pc.publish(self.clearcoat2_data_seq_cloud)

                self.sealercoat.publish(sealer_data_row_path_list)
                self.basecoat1.publish(basecoat1_data_row_path_list)
                self.basecoat2.publish(basecoat2_data_row_path_list)
                self.clearcoat1.publish(clearcoat1_data_row_path_list)
                self.clearcoat2.publish(clearcoat2_data_row_path_list)

            rate.sleep()


if __name__ == '__main__':
    SyntheticCloud()
    rospy.spin()
