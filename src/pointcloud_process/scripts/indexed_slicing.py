#!/usr/bin/env python

import open3d as o3d
import numpy as np
from ctypes import *  # convert float to uint32
import rospy
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import math
import tf
from tf import TransformListener
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs.msg import PointCloud2, PointField

import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline
from scipy.optimize import curve_fit
from scipy.spatial import ConvexHull, convex_hull_plot_2d, Delaunay
from itertools import compress



class IndexedSlicing():

    def __init__(self):
        rospy.init_node('indexed_slicing')
        rospy.Subscriber("zyx_sliced", PointCloud2, self._callback)

        self._cloud = PointCloud2()

        self.sealer_cloud_indexed = PointCloud2()
        self.base1_cloud_indexed = PointCloud2()
        self.base2_cloud_indexed = PointCloud2()
        self.clear1_cloud_indexed = PointCloud2()
        self.clear2_cloud_indexed = PointCloud2()

        self.cloud_list = []
        self.row_splitting_base1=[]
        self.row_splitting_sealer=[]
        self.row_splitting_base2=[]
        self.row_splitting_clear1=[]
        self.row_splitting_clear2=[]
        self.row_first = []

        self.maxy = -1.0
        self.miny = 10.0
        self.maxx = -1.0
        self.minx = 10.0
        self.minz = 10.0
        self.maxz = -1.0

        self.top_index_row_distance = 0.1524    #6 inches
        self.bottom_index_row_distance = 0.1524 #6 inches

        self.left_right_index_distance = 0.0508 #2 inches

        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                       PointField('y', 4, PointField.FLOAT32, 1),
                       PointField('z', 8, PointField.FLOAT32, 1)]

        self.header = Header()
        self.header.frame_id = "mir_link"
        self.header.stamp = rospy.Time.now()

        self.pub_sealer = rospy.Publisher(
            "/curve_indexed_sealer", PointCloud2, queue_size=1)
        self.pub_base1 = rospy.Publisher(
            "/curve_indexed_base1", PointCloud2, queue_size=1)
        self.pub_base2 = rospy.Publisher(
            "/curve_indexed_base2", PointCloud2, queue_size=1)
        self.pub_clear1 = rospy.Publisher(
            "/curve_indexed_clear1", PointCloud2, queue_size=1)
        self.pub_clear2 = rospy.Publisher(
            "/curve_indexed_clear2", PointCloud2, queue_size=1)

        self.read = False
        self.process_completed = False
        self.start_indexed_slicing()

    def _callback(self, cloud_msg):
        if(not self.read):
            self._cloud = cloud_msg
        self.read = True

    def start_indexed_slicing(self):
        rate = rospy.Rate(20)

        while (not rospy.is_shutdown()):

            if(self.read and not self.process_completed):
                print("start")
                self.caller()
            elif(self.process_completed):
                self.curve_indexed_publishers()
            else:
                pass

            rate.sleep()

    def caller(self):

        print("in caller")
        self.get_min_max_x_y_z()
        self.sealer_cloud_indexed = self.get_curve_indexed_cloud(0.0635)
        self.base1_cloud_indexed = self.get_curve_indexed_cloud(0.0381)
        self.base2_cloud_indexed = self.get_curve_indexed_cloud(0.0508)
        self.clear1_cloud_indexed = self.get_curve_indexed_cloud(0.0762)
        self.clear2_cloud_indexed = self.get_curve_indexed_cloud(0.0762)
        self.process_completed = True

    def curve_indexed_publishers(self):

        self.pub_sealer.publish(self.sealer_cloud_indexed)
        self.pub_base1.publish(self.base1_cloud_indexed)
        self.pub_base2.publish(self.base2_cloud_indexed)
        self.pub_clear1.publish(self.clear1_cloud_indexed)
        self.pub_clear2.publish(self.clear2_cloud_indexed)

    def get_curve_indexed_cloud(self, row_index):
        xslice = self.get_slices_in_x(row_index)
        print("length of full cloud", len(xslice))
        return pc2.create_cloud(self.header, self.fields, xslice)

    def get_slices_in_x(self, required_index_distance):
        print("in get slice in x")
        start_x = self.minx
        yz_plane_width = 0.01
        slicing_distance_x = 0.05
        x_new_list = []
        parameter_list = []
        #miny_dxcurve_list = []
        #maxy_dxcurve_list = []
        while(start_x <= self.maxx):
            conv_hull = [[start_x, self.miny, self.minz], [start_x, self.maxy, self.minz], [start_x, self.miny, self.maxz], [start_x, self.maxy, self.maxz], [start_x+yz_plane_width,
                                                                                                                                                              self.miny, self.minz], [start_x+yz_plane_width, self.maxy, self.minz], [start_x+yz_plane_width, self.miny, self.maxz], [start_x+yz_plane_width, self.maxy, self.maxz]]
            bool_list = self.in_hull(self.cloud_list, conv_hull)

            x_curr_list = list(compress(self.cloud_list, bool_list))

            output_points = []
            output_points_with_x = []
            if(len(x_curr_list) > 0):
                avgx = np.mean(np.array(x_curr_list), axis=0)[0]
                x_curr_list = [[avgx, val[1], val[2]] for val in x_curr_list]

                parameter, _, miny_dxcurve, maxy_dxcurve, minz_dxcurve, maxz_dxcurve= self.get_curve_at_x_slice(
                    x_curr_list)
                    
                if(len(parameter) != 0.0):
                    parameter_list.append(parameter)
                    #miny_dxcurve_list.append(miny_dxcurve)
                    #maxy_dxcurve_list.append(maxy_dxcurve_list)
                    output_points = self.get_points_at_index_row_distance(
                        parameter, miny_dxcurve, maxy_dxcurve, required_index_distance)

                if(len(output_points) != 0):
                    output_points_with_x = [
                        [avgx, val[0], val[1]] for val in output_points]
                    x_new_list = x_new_list + output_points_with_x

            start_x = start_x + yz_plane_width + slicing_distance_x

        return x_new_list

    def get_points_at_index_row_distance(self, parameter, miny_dxcurve, maxy_dxcurve, required_index_distance):
        x = np.linspace(miny_dxcurve, maxy_dxcurve, 10000)
        #self.top_index_row_distance
        points = []
        print(len(parameter))
        prevx, prevy = x[0], self.func(
            x[0], parameter[0], parameter[1], parameter[2], parameter[3], parameter[4], parameter[5])
        
        for pt in x:
            currx, curry = pt, self.func(
                pt, parameter[0], parameter[1], parameter[2], parameter[3], parameter[4], parameter[5])

            if(abs(self.get_2D_distance(prevx, prevy, currx, curry)-required_index_distance) < 0.001):
                points.append([currx, curry])
                prevx, prevy = currx, curry

        print("length of points", len(points))
        return points

    def get_min_max_x_y_z(self):

        print("in get min max x y z")

        self.cloud_list = []
        pointCloudData = list(pc2.read_points(
            self._cloud, field_names=['x', 'y', 'z'], skip_nans=True))
        print("-------------pointcloud read completed-------------------")
        for data in pointCloudData:
            x = data[0]
            y = data[1]
            z = data[2]
            if(self.maxy < y):
                self.maxy = y
            if(self.miny > y):
                self.miny = y
            if(self.maxx < x):
                self.maxx = x
            if(self.minx > x):
                self.minx = x
            if(self.maxz < z):
                self.maxz = z
            if(self.minz > z):
                self.minz = z

            self.cloud_list.append([x, y, z])

    def in_hull(self, p, hull):
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

    def func(self, x, k, a, b, c, d, const):
        return k*pow(x, 5) + a*pow(x, 4) + b*pow(x, 3) + c*pow(x, 2) + d * x + const

    def get_curve_at_x_slice(self, xslice):

        print("get_curve_at_x_slice")
        x_slice_array = np.asarray(xslice)
        x_deleted = np.delete(x_slice_array, 0, 1)

        print("x deleted length", len(x_deleted))
        x_deleted_z_sorted = sorted(x_deleted, key=lambda x: x[1])

        # print(x_deleted_z_sorted)

        y_points = [point[0] for point in x_deleted_z_sorted]
        z_points = [point[1] for point in x_deleted_z_sorted]

        print("y points length", len(y_points))
        print("z points length", len(z_points))

        if((len(y_points) == len(z_points)) and len(y_points) > 15):

            parameter, covariance_matrix = curve_fit(
                self.func, y_points, z_points)

            return parameter.tolist(), covariance_matrix.tolist(), min(y_points), max(y_points) , min(z_points) , max(z_points)
  
        
        return [], [], 0.0, 0.0, 0.0, 0.0

    def get_2D_distance(self, p1x, p1y, p2x, p2y):
        return pow((pow((p2x-p1x), 2)+pow((p2y-p1y), 2)), 0.5)


if __name__ == "__main__":
    try:
        IndexedSlicing()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(e)
