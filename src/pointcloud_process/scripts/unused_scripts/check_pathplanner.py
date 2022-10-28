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
from scipy.spatial import ConvexHull, convex_hull_plot_2d
from arm_ctrl_navigate.msg import *

from move_ur10e.msg import *
from move_ur10e.srv import *


class PathPlanner():

    def __init__(self):

        rospy.init_node('path_planner')

        rospy.Subscriber("orientation_points_sealercoat",
                         PointCloud2, self.sealer_callback)
        rospy.Subscriber("orientation_points_basecoat_1",
                         PointCloud2, self.base1_callback)
        rospy.Subscriber("orientation_points_basecoat_2",
                         PointCloud2, self.base2_callback)
        rospy.Subscriber("orientation_points_clearcoat_1",
                         PointCloud2, self.clear1_callback)
        rospy.Subscriber("orientation_points_clearcoat_2",
                         PointCloud2, self.clear2_callback)

        self.sealer_cloud = PointCloud2()
        self.base1_cloud = PointCloud2()
        self.base2_cloud = PointCloud2()
        self.clear1_cloud = PointCloud2()
        self.clear2_cloud = PointCloud2()

        self.sealer_cloud_viz = PointCloud2()
        self.base1_cloud_viz = PointCloud2()
        self.base2_cloud_viz = PointCloud2()
        self.clear1_cloud_viz = PointCloud2()
        self.clear2_cloud_viz = PointCloud2()

        self.pointcloud_with_quaternion_sealer = []
        self.pointcloud_with_quaternion_base1 = []
        self.pointcloud_with_quaternion_base2 = []
        self.pointcloud_with_quaternion_clear1 = []
        self.pointcloud_with_quaternion_clear2 = []

        self.path_points_sealer = Path()
        self.path_points_base1 = Path()
        self.path_points_base2 = Path()
        self.path_points_clear1 = Path()
        self.path_points_clear2 = Path()

        self.read_sealer = False
        self.read_base1 = False
        self.read_base2 = False
        self.read_clear1 = False
        self.read_clear2 = False

        self.process_completed = False

        self.read_as_list = False

        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                       PointField('y', 4, PointField.FLOAT32, 1),
                       PointField('z', 8, PointField.FLOAT32, 1)]
        self.header = Header()
        self.header.frame_id = "mir_link"
        self.header.stamp = rospy.Time.now()

        self.pub_sealer = rospy.Publisher(
            "/path_points_sealer", Path, queue_size=1)
        self.pub_base1 = rospy.Publisher(
            "/path_points_base1", Path, queue_size=1)
        self.pub_base2 = rospy.Publisher(
            "/path_points_base2", Path, queue_size=1)
        self.pub_clear1 = rospy.Publisher(
            "/path_points_clear1", Path, queue_size=1)
        self.pub_clear2 = rospy.Publisher(
            "/path_points_clear2", Path, queue_size=1)

        self.pub_sealer_visual = rospy.Publisher(
            "/visual_path_points_sealer", PointCloud2, queue_size=1)
        self.pub_base1_visual = rospy.Publisher(
            "/visual_path_points_base1", PointCloud2, queue_size=1)
        self.pub_base2_visual = rospy.Publisher(
            "/visual_path_points_base2", PointCloud2, queue_size=1)
        self.pub_clear1_visual = rospy.Publisher(
            "/visual_path_points_clear1", PointCloud2, queue_size=1)
        self.pub_clear2_visual = rospy.Publisher(
            "/visual_path_points_clear2", PointCloud2, queue_size=1)

        self.start_process()

    def sealer_callback(self, cloud_msg):
        self.sealer_cloud = cloud_msg
        self.read_sealer = True

    def base1_callback(self, cloud_msg):
        self.base1_cloud = cloud_msg
        self.read_base1 = True

    def base2_callback(self, cloud_msg):
        self.base2_cloud = cloud_msg
        self.read_base2 = True

    def clear1_callback(self, cloud_msg):
        self.clear1_cloud = cloud_msg
        self.read_clear1 = True

    def clear2_callback(self, cloud_msg):
        self.clear2_cloud = cloud_msg
        self.read_clear2 = True

    def planned_path_publishers(self):

        self.pub_sealer.publish(self.path_points_sealer)
        self.pub_base1.publish(self.path_points_base1)
        self.pub_base2.publish(self.path_points_base2)
        self.pub_clear1.publish(self.path_points_clear1)
        self.pub_clear2.publish(self.path_points_clear2)

        self.pub_sealer_visual.publish(self.sealer_cloud_viz)
        self.pub_base1_visual.publish(self.base1_cloud_viz)
        self.pub_base2_visual.publish(self.base2_cloud_viz)
        self.pub_clear1_visual.publish(self.clear1_cloud_viz)
        self.pub_clear2_visual.publish(self.clear2_cloud_viz)

    def start_process(self):
        rate = rospy.Rate(20)
        while (not rospy.is_shutdown()):
            if(self.read_sealer and self.read_base1 and self.read_base2 and self.read_clear1 and self.read_clear2 and not self.read_as_list):
                self.point_cloud_reader()
                print("all coat point read as list")
            elif(self.read_as_list and not self.process_completed):
                self.start_path_planning()
            elif(self.process_completed):
                self.planned_path_publishers()
            else:
                pass
            rate.sleep()

    def point_cloud_reader(self):

        self.pointcloud_with_quaternion_sealer = list(pc2.read_points(self.sealer_cloud, field_names=[
                                                      'x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z', 'curvature'], skip_nans=True))
        self.pointcloud_with_quaternion_base1 = list(pc2.read_points(self.base1_cloud, field_names=[
                                                     'x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z', 'curvature'], skip_nans=True))
        self.pointcloud_with_quaternion_base2 = list(pc2.read_points(self.base2_cloud, field_names=[
                                                     'x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z', 'curvature'], skip_nans=True))
        self.pointcloud_with_quaternion_clear1 = list(pc2.read_points(self.clear1_cloud, field_names=[
                                                      'x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z', 'curvature'], skip_nans=True))
        self.pointcloud_with_quaternion_clear2 = list(pc2.read_points(self.clear2_cloud, field_names=[
                                                      'x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z', 'curvature'], skip_nans=True))
        self.read_as_list = True

    def start_path_planning(self):
        print("computing sealer coat")
        final_path_sealer = self.convert_path_message_type(self.get_final_path(
            self.pointcloud_with_quaternion_sealer))
        #print("sealer coat", final_path_sealer)

        print("computing base1 coat")
        final_path_base1 = self.convert_path_message_type(self.get_final_path(
            self.pointcloud_with_quaternion_base1))
        #print("base1 coat", final_path_base1)

        print("computing base2 coat")
        final_path_base2 = self.convert_path_message_type(self.get_final_path(
            self.pointcloud_with_quaternion_base2))
        #print("base2 coat", final_path_base2)

        print("computing clear1 coat")
        final_path_clear1 = self.convert_path_message_type(self.get_final_path(
            self.pointcloud_with_quaternion_clear1))
        #print("clear1 coat", final_path_clear1)

        print("computing clear2 coat")
        final_path_clear2 = self.convert_path_message_type(self.get_final_path(
            self.pointcloud_with_quaternion_clear2))
        #print("clear2 coat", final_path_clear2)

        self.path_points_sealer = final_path_sealer
        self.path_points_base1 = final_path_base1
        self.path_points_base2 = final_path_base2
        self.path_points_clear1 = final_path_clear1
        self.path_points_clear2 = final_path_clear2

        self.sealer_cloud_viz = self.get_created_cloud(final_path_sealer)
        self.base1_cloud_viz = self.get_created_cloud(final_path_base1)
        self.base2_cloud_viz = self.get_created_cloud(final_path_base2)
        self.clear1_cloud_viz = self.get_created_cloud(final_path_clear1)
        self.clear2_cloud_viz = self.get_created_cloud(final_path_clear2)

        self.process_completed = True

    def get_created_cloud(self, final_path_):
        final_path_ = [[point.x, point.y, point.z]
                       for row in final_path_.path for point in row.path_msg]
        return pc2.create_cloud(self.header, self.fields, final_path_)

    def convert_path_message_type(self, final_list_path):
        path = Path()
        for row in final_list_path:
            row_new = PathStamped()
            for point in row:
                point_new = Plannedpath()
                point_new.x = point[0]
                point_new.y = point[1]
                point_new.z = point[2]
                point_new.ox = point[3]
                point_new.oy = point[4]
                point_new.oz = point[5]
                point_new.ow = point[6]
                row_new.path_msg.append(point_new)
            path.path.append(row_new)
        return path

    def get_final_path(self, input_points):

        sorted_in_z = sorted(
            input_points, key=lambda x: x[2], reverse=True)
        min_in_z = min(input_points,
                       key=lambda x: x[2])[2]
        max_in_z = max(input_points,
                       key=lambda x: x[2])[2]
        min_in_y = min(input_points,
                       key=lambda x: x[1])[1]
        max_in_y = max(input_points,
                       key=lambda x: x[1])[1]

        print("min max in y and z", min_in_y, min_in_z, max_in_y, max_in_z)
        print("splitting in z")
        row_splitted_path_z = self.split_in_z(sorted_in_z, min_in_z, max_in_z)
        print("splitting in y")
        row_splitted_path = self.split_in_y(row_splitted_path_z)
        print("sort each row in x")
        final_path = self.sort_each_row_in_x(row_splitted_path)
        count = 0
        for row in final_path:
            for pt in row:
                count = count + 1
        print("path_points_count : ", count)
        return final_path

    def split_in_z(self, sorted_in_z, min_in_z, max_in_z):

        row_splitted_path_z = []
        if(len(sorted_in_z) == 0):
            print("sorted_in_z is empty")
        else:
            currz = max_in_z
            step_in_z = 0.015

            while(currz >= min_in_z):
                row = []
                indices_to_delete = []
                for index, point in enumerate(sorted_in_z):
                    if (abs(point[2]-currz) < 0.0075):
                        row.append(point)
                        indices_to_delete.append(index)
                if(len(row) > 0):
                    row_splitted_path_z.append(row)
                    sorted_in_z_del = []
                    for ind, pt in enumerate(sorted_in_z):
                        if(ind not in indices_to_delete):
                            sorted_in_z_del.append(pt)
                    sorted_in_z = sorted_in_z_del
                    print(len(row))
                currz = currz - step_in_z

        print("length of rows after z splitting", len(row_splitted_path_z))
        return row_splitted_path_z

    def split_in_y(self, row_splitted_path_z):

        row_splitted_path = []
        if(len(row_splitted_path_z) == 0):
            print("row_splitted_path_z is empty")
        else:
            for row in row_splitted_path_z:
                sort_in_y = sorted(row, key=lambda x: x[1])
                first_point = sort_in_y[0]
                chunk_start_index = 0
                in_atleast_once = False
                for ind, point in enumerate(sort_in_y):
                    if(abs(first_point[1] - point[1]) > 0.025):
                        splitted_list = sort_in_y[chunk_start_index:ind]
                        chunk_start_index = ind
                        row_splitted_path.append(splitted_list)
                        in_atleast_once = True
                if(not in_atleast_once):
                    row_splitted_path.append(row)
        print("length of rows after y splitting", len(row_splitted_path))
        return row_splitted_path

    def sort_each_row_in_x(self, row_splitted_path):
        final_path = []
        toggle_order = False
        for eachrow in row_splitted_path:
            sorted_row = []
            if(toggle_order):
                sorted_row = sorted(eachrow, key=lambda x: x[0])
            else:
                sorted_row = sorted(eachrow, key=lambda x: x[0], reverse=True)
            final_path.append(sorted_row)

        return final_path


if __name__ == "__main__":
    PathPlanner()

    rospy.spin()
