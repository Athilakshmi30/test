#!/usr/bin/env python
from __future__ import print_function
from pickletools import float8
#from tkinter.tix import Tree
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
import scipy.optimize as optimize
import numpy as np
import copy

# 5 for sealer,3 for base1, 4 for base2, 7 for clear 1 and clear 2


#rospy.set_param("sealer_grid_width",0.019)#0.03175)#0.03175)#0.019
rospy.set_param("sealer_grid_height",4)#0.03175)#0.03175)#0.019
#rospy.set_param("sealer_grid_depth",0.019)#0.03175)#0.03175)#0.019
#rospy.set_param("basecoat1_grid_width",0.019)#0.019
rospy.set_param("basecoat1_grid_height",2)#0.019
#rospy.set_param("basecoat1_grid_depth",0.019)#0.019 
#rospy.set_param("basecoat2_grid_width",0.019)#0.0254)#0.0254)#0.019
rospy.set_param("basecoat2_grid_height",3)#0.0254)#0.0254)#0.019
#rospy.set_param("basecoat2_grid_depth",0.019)#0254)#0.0254)#0.019
#rospy.set_param("clearcoat1_grid_width",0.0381)#0.0381)#0.019
rospy.set_param("clearcoat1_grid_height",6)#0.0381)#0.019
#rospy.set_param("clearcoat1_grid_depth",0.0381)#0.0381)#0.019
#rospy.set_param("clearcoat2_grid_width",0.0381)#0.0381)#0.019
rospy.set_param("clearcoat2_grid_height",6)#0.0381)#0.019
#rospy.set_param("clearcoat2_grid_depth",0.0381)#0.0381)#0.019
rospy.set_param("coat_process",False)

class SyntheticCloud():

    def __init__(self):

        rospy.init_node('synthetic_cloud')

        #self.pub = rospy.Publisher(
        #    "/max_depth_added_cloud", PointCloud2, queue_size=1)

        #self.pub1 = rospy.Publisher(
        #    "/occlusion_removed_cloud", PointCloud2, queue_size=1)

        self.griddedCloud_pc = rospy.Publisher(
            "/gridded_cloud_pc", PointCloud2, queue_size=1)
        
        #self.pub3 = rospy.Publisher(
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

       

        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                       PointField('y', 4, PointField.FLOAT32, 1),
                       PointField('z', 8, PointField.FLOAT32, 1)
                       ]

        self.fields_normal = [PointField('x', 0, PointField.FLOAT32, 1),
                       PointField('y', 4, PointField.FLOAT32, 1),
                       PointField('z', 8, PointField.FLOAT32, 1),
                       PointField('normal_x', 12, PointField.FLOAT32, 1),
                       PointField('normal_y', 16, PointField.FLOAT32, 1),
                       PointField('normal_z', 20, PointField.FLOAT32, 1)
                       ]     

        self.fields_orientation = [PointField('x', 0, PointField.FLOAT32, 1),
                       PointField('y', 4, PointField.FLOAT32, 1),
                       PointField('z', 8, PointField.FLOAT32, 1),
                       PointField('normal_x', 12, PointField.FLOAT32, 1),
                       PointField('normal_y', 16, PointField.FLOAT32, 1),
                       PointField('normal_z', 20, PointField.FLOAT32, 1),
                       PointField('curvature', 24, PointField.FLOAT32, 1)
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
        self.synthetic_cloud = PointCloud2()
        
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

        self.depth_width = 0.254  # 10 inches

        self.left_right_index_offset = 0.0762 #3 inches #0.1524 - 6 inches

        self.top_bottom_index_row_offset = 0.0762 #3 inches #0.1524 - 6 inches

        

        self.received_sealer = False
        self.received_base1 = True
        self.received_base2 = True
        self.received_clear1 = True
        self.received_clear2 = True
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

    def normalEstimation(self,in_pointcloud): 

        downpcd = self.convertCloudFromRosToOpen3d(in_pointcloud)

        #downpcd = downpcd.voxel_down_sample(voxel_size= 0.011)
        downpcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.025, max_nn=100))
        o3d.visualization.draw_geometries_with_editing([downpcd])    
        return self.convertCloudFromOpen3dToRosWithNormals(downpcd)    
 
    def inHull(self, p, hull):
        #print("Inside hull function")
        """
        Test if points in `p` are in `hull`

        `p` should be a `NxK` coordinates of `N` points in `K` dimensions
        `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the 
        coordinates of `M` points in `K`dimensions for which Delaunay triangulation
        will be computed
        """
        
        

        if not isinstance(hull, Delaunay):
            hull = Delaunay(hull)

        #print("Excited hull function")
        return hull.find_simplex(p) >= 0


    def sealerCallBack(self, msg):

        if(not self.received_sealer):
            print("msg.fields",msg.fields)
            self.point_cloud_input = msg
            self.sealer_grid_height = rospy.get_param("sealer_grid_height")          

            print("Sealer")
            #self.process_input_cloud = self.normalEstimation(self.point_cloud_input)
            self.point_cloud_list = list(point_cloud2.read_points(
                    self.point_cloud_input, field_names=['x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z','curvature'], skip_nans=True))
                #self.sealer_coat_data = self.start(self.sealer_grid_width,self.sealer_grid_height,self.sealer_grid_depth)
            self.common_max_depth_added_cloud_list = self.addDepthPlane(
                    self.point_cloud_list)
            common_plane_data_list, common_boolean_grid, common_gridded_cloud_list, common_synthetic_cloud_list,common_synthetic_cloud,minz = self.createGrid(
                    self.point_cloud_list)
                #print("selaer len",len(sealer_plane_data_list))
            sealer_data_row_list, self.sealer_data_row_path_list = self.sequencing(common_plane_data_list,self.sealer_grid_height,minz)
            self.sealer_gridded_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_orientation, common_gridded_cloud_list)
            self.sealer_data_seq_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_orientation, sealer_data_row_list)
            print("Process complited")
            self.received_sealer = True
            #self.point_cloud_list = list(point_cloud2.read_points(
            #    msg, field_names=['x', 'y', 'z'], skip_nans=True))
            #self.received = True

    def base1CallBack(self, msg):
        

        if(not self.received_base1):
            #print(msg.fields)
            self.point_cloud_input = msg
            self.basecoat1_grid_height = rospy.get_param("basecoat1_grid_height")

            print("Base1")
            
            self.point_cloud_list = list(point_cloud2.read_points(
                    self.point_cloud_input, field_names=['x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z','curvature'], skip_nans=True))
                #self.sealer_coat_data = self.start(self.sealer_grid_width,self.sealer_grid_height,self.sealer_grid_depth)
            self.common_max_depth_added_cloud_list = self.addDepthPlane(
                    self.point_cloud_list)
            common_plane_data_list, common_boolean_grid, common_gridded_cloud_list, common_synthetic_cloud_list,common_synthetic_cloud,minz = self.createGrid(
                    self.point_cloud_list)
                #print("selaer len",len(sealer_plane_data_list))
            basecoat1_data_row_list, self.basecoat1_data_row_path_list = self.sequencing(common_plane_data_list,self.basecoat1_grid_height,minz)
            self.basecoat1_gridded_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_orientation, common_gridded_cloud_list)
            self.basecoat1_data_seq_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_orientation, basecoat1_data_row_list)
            self.received_base1 = True
            #self.point_cloud_list = list(point_cloud2.read_points(
            #    msg, field_names=['x', 'y', 'z'], skip_nans=True))
            #self.received = True
            # 

    def base2CallBack(self, msg):
        

        if(not self.received_base2):
            #print(msg.fields)
            self.point_cloud_input = msg
            self.basecoat2_grid_height = rospy.get_param("basecoat2_grid_height")

            print("Base2")
            
            self.point_cloud_list = list(point_cloud2.read_points(
                    self.point_cloud_input, field_names=['x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z','curvature'], skip_nans=True))
                #self.sealer_coat_data = self.start(self.sealer_grid_width,self.sealer_grid_height,self.sealer_grid_depth)
            self.common_max_depth_added_cloud_list = self.addDepthPlane(
                    self.point_cloud_list)
            common_plane_data_list, common_boolean_grid, common_gridded_cloud_list, common_synthetic_cloud_list,common_synthetic_cloud,minz = self.createGrid(
                    self.point_cloud_list)
                #print("selaer len",len(sealer_plane_data_list))
            basecoat2_data_row_list, self.basecoat2_data_row_path_list = self.sequencing(common_plane_data_list,self.basecoat2_grid_height,minz)
            self.basecoat2_gridded_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_orientation, common_gridded_cloud_list)
            self.basecoat2_data_seq_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_orientation, basecoat2_data_row_list)
            self.received_base2 = True
            #self.point_cloud_list = list(point_cloud2.read_points(
            #    msg, field_names=['x', 'y', 'z'], skip_nans=True))
            #self.received = True        

    def clear1CallBack(self, msg):
        

        if(not self.received_clear1):
            #print(msg.fields)
            self.point_cloud_input = msg
            self.clearcoat1_grid_height = rospy.get_param("clearcoat1_grid_height")
            print("Clear1")
            
            self.point_cloud_list = list(point_cloud2.read_points(
                    self.point_cloud_input, field_names=['x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z','curvature'], skip_nans=True))
                #self.sealer_coat_data = self.start(self.sealer_grid_width,self.sealer_grid_height,self.sealer_grid_depth)
            self.common_max_depth_added_cloud_list = self.addDepthPlane(
                    self.point_cloud_list)
            common_plane_data_list, common_boolean_grid, common_gridded_cloud_list, common_synthetic_cloud_list,common_synthetic_cloud,minz = self.createGrid(
                    self.point_cloud_list)
                #print("selaer len",len(sealer_plane_data_list))
            clearcoat1_data_row_list, self.clearcoat1_data_row_path_list = self.sequencing(common_plane_data_list,self.clearcoat1_grid_height,minz)
            self.clearcoat1_gridded_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_orientation, common_gridded_cloud_list)
            self.clearcoat1_data_seq_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_orientation, clearcoat1_data_row_list) 
            self.received_clear1 = True
            #self.point_cloud_list = list(point_cloud2.read_points(
            #    msg, field_names=['x', 'y', 'z'], skip_nans=True))
            #self.received = True

    def clear2CallBack(self, msg):

        if(not self.received_clear2):
            #print(msg.fields)
            self.point_cloud_input = msg
            self.clearcoat2_grid_height = rospy.get_param("clearcoat2_grid_height")

            print("Clear2")
          
            self.point_cloud_list = list(point_cloud2.read_points(
                    self.point_cloud_input, field_names=['x', 'y', 'z', 'normal_x', 'normal_y', 'normal_z','curvature'], skip_nans=True))
                #self.sealer_coat_data = self.start(self.sealer_grid_width,self.sealer_grid_height,self.sealer_grid_depth)
            self.common_max_depth_added_cloud_list = self.addDepthPlane(
                    self.point_cloud_list)
            common_plane_data_list, common_boolean_grid, common_gridded_cloud_list, common_synthetic_cloud_list,common_synthetic_cloud,minz = self.createGrid(
                    self.point_cloud_list)
                #print("selaer len",len(sealer_plane_data_list))
            clearcoat2_data_row_list, self.clearcoat2_data_row_path_list = self.sequencing(common_plane_data_list,self.clearcoat2_grid_height,minz)
            self.clearcoat2_gridded_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_orientation, common_gridded_cloud_list)
            self.clearcoat2_data_seq_cloud = point_cloud2.create_cloud(
                    self.header, self.fields_orientation, clearcoat2_data_row_list) 
            self.received_clear2 = True
            #self.point_cloud_list = list(point_cloud2.read_points(
            #    msg, field_names=['x', 'y', 'z'], skip_nans=True))
            #self.received = True
                    
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
        self.max_height = maxz #+ self.top_bottom_index_row_offset
        self.min_height = minz #- self.top_bottom_index_row_offset
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
        cropped_cloud = [[pt[0],pt[1],pt[2]] for pt in cloud if (
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
            print("Z_range : ",z_range)
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
                        #print("reached out from function")
                        ending = time.time() 
    
                        total = total + ending - start
                        current_list = list(compress(cropped_cloud, bool_list))
        
                        if(len(current_list) > 0):
                            pt_n = []
                            for pt in cloud:
                                if(abs(pt[0] - current_list[0][0])<=0.001 and abs(pt[1] - current_list[0][1])<=0.001 and abs(pt[2] - current_list[0][2])<=0.001):
                                    pt_n = pt
                                    break  
                            width_row.append(pt_n)
                            width_boolean_row.append(True)
                            width_synthetic_row.append(self.centroid(conv_hull))
                            #bool_inverted_list = [not state for state in bool_list]
                            #cropped_cloud = list(
                            #    compress(cropped_cloud, bool_inverted_list))
                        else:
                            width_boolean_row.append(False)
                            width_row.append([0,0,0,0,0,0,0])
                            width_synthetic_row.append([0,0,0,0,0,0,0])
                            #width_row.append([0,0,0])
                            #width_synthetic_row.append([0,0,0]) 
                        #print("one loop complete")                       
                        
                        
                        current_x = current_x + self.grid_width
    
                    depth_row.append(width_row)
                    depth_boolean_row.append(width_boolean_row)
                    depth_synthetic_row.append(width_synthetic_row)
                    current_y = current_y + self.grid_depth

                gridded_cloud.append(depth_row)
                boolean_grid.append(depth_boolean_row)
                synthetic_cloud.append(depth_synthetic_row)
                
            current_z = current_z + self.grid_height
            
        gridded_cloud_linear = [pt for row in gridded_cloud for col in row for pt in col]
        synthetic_cloud_linear = [pt for row in synthetic_cloud for col in row for pt in col]  
        
        #print("End time : ",time.time())

        #print("Total createGrid: ",total)  



        return gridded_cloud, boolean_grid, gridded_cloud_linear, synthetic_cloud_linear,synthetic_cloud, minz

    def points_for_path(self,boolean_grid):
        
        start = time.time()
        #print("start : ",start)
        len_of_planes = len(boolean_grid)
        len_of_depthrows_in_each_plane = len(boolean_grid[0])
        len_of_each_depthrow = len(boolean_grid[0][1])  

        curr_plane = len_of_planes - 1

        curr_depth_row =  0
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
                        row.append([curr_plane,curr_depth_row,curr_point])                  

                    curr_point += 1
                
                if(len(row)>0):    
                    plane.append(row)
                                       
                curr_depth_row += 1
                
            if(len(plane)>0):
                path_list.append(plane)            
            curr_plane -= 1

        ending = time.time()
        #print("Ending : ",ending)
        #print("Total points for path : ",ending-start)
        
        return path_list , boolean_grid                      
            
    def path_plan(self,path_list , boolean_grid):#, gridded_cloud_list, synthetic_cloud_list):
        #print("path_list[15]",path_list[15])
        toggle = True
        #print("len_path_list",path_list)
        path_rearanged_list = []  
        for plane in path_list:
            if toggle:
                plane.reverse()
                path_rearanged_list.append(plane)
            else:
                path_rearanged_list.append(plane)
            toggle =  not toggle

        path_rearanged_row_list = []
        #print("len_path_rearanged_list",path_rearanged_list)
        #print("path_rearanged_list",path_rearanged_list)
        for plane in path_rearanged_list:
            new_row = []

            for row in plane:
                #print("row",row)
                if toggle:
                    row.reverse()
                    path_rearanged_list.append(row)
                else:
                    path_rearanged_list.append(row)
                toggle =  not toggle
                new_row.append(row)
            path_rearanged_row_list.append(new_row)                   
        #print("path_rearanged_row_list[15]",path_rearanged_row_list[15])
        return path_rearanged_row_list
    

    
    def two_d_curve_fit(self,data_list,z_val,index_y_mod):

       

        X_data=[]
        Y_data=[]
        Z_data=[]
        check_z = []
        new_fitted_points = []
        #print("z_val",z_val)
        #print("curve_fit called")
        for rows in data_list:
            #print("points",points)
            for pt in rows:
                check_z.append(pt[2])                
                if pt[0]==0:
                    continue
                X_data.append(pt[0])
                Y_data.append(pt[1])
                Z_data.append(z_val)
        Z_data_arr = np.array(Z_data)

        #print("Z_data_arr",Z_data_arr.mean())
        #print("len(X_data)",len(X_data))
        if len(X_data)==0:
            check_z_arr = np.array(check_z)
            print("No X,Y point data for Z = ",check_z_arr.mean())
            new_fitted_points = np.array(new_fitted_points)
            return new_fitted_points
        #6deg
        a, b, c, d, e, f,g = np.polyfit(X_data, Y_data, 6)
        fit_equation = lambda x: a * x ** 6 + b * x **5+ c*x**4+d*x**3+e*x**2+f*x+g        

        avg_z = Z_data_arr.mean()
        fitted_points = []
        #print("data_list",len(data_list))
        #print("X_data",X_data)
        #prev_x = -100       
        for row in data_list:
 
            for indx, pt in enumerate(row):
                if pt[0]==0:
                    continue
                #print("pt : ",pt)

                for ind,xcoord in enumerate(X_data):

                    if(abs(xcoord - pt[0]) < 0.001 and abs(Y_data[ind] - pt[1]) < 0.001 and abs(Z_data[ind] - z_val) < 0.001 ):#and indx%3 == 0):
                        fitted_points.append([xcoord,fit_equation(xcoord),avg_z,pt[3],pt[4],pt[5],pt[6]])
                        #print("point",xcoord,Y_data[ind],Z_data[ind])
                        #print("prev_x",prev_x)
                        #print("xcoord",xcoord)
                        #if prev_x==-100:
                            #print("First condition")
                            #fitted_points.append([xcoord,fit_equation(xcoord),avg_z,pt[3],pt[4],pt[5]])
                            #prev_x = xcoord
                        #if abs(prev_x-xcoord)>0.01:
                            #print("Second condition")
                            #fitted_points.append([xcoord,fit_equation(xcoord),avg_z,pt[3],pt[4],pt[5]])
                            #prev_x = xcoord

        #print("before sorted fitted_points",fitted_points)
        fitted_points = sorted(fitted_points, key = lambda x : x[0])            
        #fitted_points = np.array(fitted_points)
        #print("after sorted fitted_points",fitted_points)

        #np.empty((0, 6), float)
        prev_x = -100
        for ind,i in enumerate(fitted_points):
            #print("itr")
            #print("ind",ind)
            #print("i",i)
            #print("new_fitted_points",new_fitted_points)
            if prev_x==-100:
                #print("First condition")
                prev_x= i[0]
                new_fitted_points.append(i)

            elif abs(prev_x - i[0])>0.02:#0.02
                #print("Second condition")
                prev_x= i[0]
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
        change_threshold = 0.01#0.02
        equal_threshold = 0.005#0.01        
        
        #print("======================================================================original new_fitted_points =================================================================")
        #print("original fitted_points",new_fitted_points)

        for ind, point in enumerate(new_fitted_points):
             #point.split(",")
             #out = map(int, point)
             #print("out",point.split(","))     
             first_point = new_fitted_points[ind]
             #print("first_point",first_point)
             #print("first_point vz" ,first_point[5])
        
        
             if len(new_fitted_points)-ind>1:
                 second_point = new_fitted_points[ind+1]
                 diff_f_s_xz = abs(float(first_point[5])-float(second_point[5]))
                 #print("second_point",second_point[5])
        
             if len(new_fitted_points)-ind>2:
                 third_point = new_fitted_points[ind+2]
                 diff_f_t_xz = abs(float(first_point[5])-float(third_point[5]))
                 diff_s_t_xz = abs(float(second_point[5])-float(third_point[5]))
                 #print("third_point",third_point[5])
        
             if len(new_fitted_points)-ind>3:
                 fourth_point = new_fitted_points[ind+3]
                 diff_f_fo_xz = abs(float(first_point[5])-float(fourth_point[5]))
                 diff_s_fo_xz = abs(float(second_point[5])-float(fourth_point[5]))
                 #print("fourth_point",fourth_point[5])
        
        
             if len(new_fitted_points)-ind>4:
                 fifth_point = new_fitted_points[ind+4]
                 diff_f_fi_xz = abs(float(first_point[5])-float(fifth_point[5]))
                 diff_s_fi_xz = abs(float(second_point[5])-float(fifth_point[5]))
                 #print("fifth_point",fifth_point[5])
             #exit()
        

        
             #print("diff_f_s_xz",diff_f_s_xz)
             if(ind==len(new_fitted_points)-1):
                 prev_point = new_fitted_points[ind-1]
                 first_point[3]= prev_point[3]
                 first_point[4]= prev_point[4]
                 first_point[5]= prev_point[5] 

             elif diff_f_s_xz>change_threshold:
                            #print("diff_s_t_xz",diff_s_t_xz)
                            #print("diff_s_fo_xz",diff_s_fo_xz)
                            #print("diff_s_fi_xz",diff_s_fi_xz)

                            
                            if (diff_s_t_xz<equal_threshold) and (diff_s_fo_xz<equal_threshold) and (diff_s_fi_xz<equal_threshold ) :    # modify current point / first point by giving second point value
                                 if not prev_point_exist:
                                     #print("Modify current point")
                                     first_point[3] = second_point[3]
                                     first_point[4] = second_point[4]
                                     first_point[5] = second_point[5]
                            elif (diff_s_t_xz<equal_threshold) and (diff_s_fo_xz<equal_threshold) and (diff_s_fi_xz>equal_threshold):  # modify second, third and fourth point
                                 #print("Modify 2 3 4 point")
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
        
                            elif (diff_s_t_xz<equal_threshold) and (diff_s_fo_xz>equal_threshold) and (diff_s_fi_xz>equal_threshold or diff_s_fi_xz==-500):  # modify second, third 
                                 #print("Modify 2 3 point")
                                 # Modify second point
                                 second_point[3] = first_point[3]
                                 second_point[4] = first_point[4]
                                 second_point[5] = first_point[5]
        
                                 # Modify third point
                                 third_point[3] = first_point[3]
                                 third_point[4] = first_point[4]
                                 third_point[5] = first_point[5]
        
        
                            elif (diff_s_t_xz>equal_threshold or diff_s_t_xz==-500):  # modify second 
                                 #print("Modify 2 point")
                                 # Modify second point
                                 second_point[3] = first_point[3]
                                 second_point[4] = first_point[4]
                                 second_point[5] = first_point[5]
        
        
             first_point=[]
             second_point=[]
             third_point=[]
             fourth_point=[]
             fifth_point=[]
             diff_f_s_xz = -500
             diff_f_t_xz = -500
             diff_f_fo_xz = -500
             diff_f_fi_xz = -500
             diff_s_t_xz = -500
             diff_s_fo_xz = -500
             diff_s_fi_xz = -500
             prev_point_exist = True
        

                #if diff_f_s_xz>
                #exit()
                

                #new_fitted_points = np.delete(new_fitted_points, ind)

        prev_x = -100
        drop_new_fitted_points = []
        for ind,i in enumerate(new_fitted_points):
            #print("itr")
            #print("ind",ind)
            #print("i",i)
            #print("new_fitted_points",new_fitted_points)
            if prev_x==-100:
                #print("First condition")
                prev_x= i[0]
                drop_new_fitted_points.append(i)

            elif abs(prev_x - i[0])>0.04:#0.02
                #print("Second condition")
                prev_x= i[0]
                drop_new_fitted_points.append(i)
            else:
                pass

        prev_x_old = -100
        prev_drop_new_fitted_points = []
        for ind,i in enumerate(prev_new_fitted_points):
            #print("itr")
            #print("ind",ind)
            #print("i",i)
            #print("new_fitted_points",new_fitted_points)
            if prev_x_old==-100:
                #print("First condition")
                prev_x_old= i[0]
                prev_drop_new_fitted_points.append(i)

            elif abs(prev_x_old - i[0])>0.04:#0.02
                #print("Second condition")
                prev_x_old= i[0]
                prev_drop_new_fitted_points.append(i)
            else:
                pass
        #print("======================================================================updated new_fitted_points =================================================================")
        #print("updated fitted_points",new_fitted_points)

        new_fitted_points = np.array(new_fitted_points)
        prev_new_fitted_points = np.array(prev_new_fitted_points)
        drop_new_fitted_points = np.array(drop_new_fitted_points)
        prev_drop_new_fitted_points = np.array(prev_drop_new_fitted_points)
        #print("fitted_points",fitted_points)
        return drop_new_fitted_points


    def three_d_curve_fit(self,data_row_point_list):
        for pt in data_row_point_list:
            print("pt in data row",pt)

        #A = np.array()
        #Xn_data=[]
        #Yn_data=[]
        #Zn_data=[]
        a_list = []
        pt_list = []
        #print("zn_val",Zn_val)
        print("3d_curve_fit called")
        for pt in data_row_point_list:
            pt_list.append([pt[0],pt[1],pt[2]])
            a_list.append([pt[3],pt[4],pt[5]])
            #data_x_y_z = np.append(pt[3],2)
            #A = np.array([(19,20,24), (10,40,28), (10,50,31)])




        def func(data, a, b):
            return data[:,0]*data[:,1]*a + b
        a_arr = np.array(a_list)
        guess = (1,1)
        params, pcov = optimize.curve_fit(func, a_arr[:,:2], a_arr[:,2], guess)
        z_data = func(a_arr[:,:2], params[0], params[1])
        #print("before a_arr", a_arr)
        #print("params",params)
        count_n = 0
        for i in a_arr:
            i[2] = z_data[count_n]

        newArray = np.append(pt_list, a_arr, axis = 1)
        #print("after a_arr", a_arr)

        return newArray

# [ 0.04919355  6.67741935]

        #6deg
        #a, b, c, d, e, f,g = np.polyfit(Xn_data, Yn_data, 6)
        #fit_equation = lambda x: a * x ** 6 + b * x **5+ c*x**4+d*x**3+e*x**2+f*x+g        





    def sequencing(self, data, index, minz):
        count = 1
        consider_count = count + 2
        not_consider_count = consider_count + index # 5 for sealer,3 for base1, 4 for base2, 6 or 7 for clear 1 and clear 2
        data_row_list = []

        toggle = False
        points = []
        reaced_last_point = False
        prev_to_last_point = []
        dy_dz_change_rows = [[],[]]
        for ind,i in enumerate(data):

            if len(data)-1==count:
                prev_to_last_point.append(i)

            if len(data)==count:
                print("last point reached")
                reaced_last_point = True

            if count<=not_consider_count:
                if reaced_last_point:
                    dy_change_row_avg = []
                    dz_change_row_avg = []
                    for ind,point in enumerate(dy_dz_change_rows):
                        dy_change=[]
                        dz_change=[]
                        for dy_dz in point:
                            dy_change.append(dy_dz[1])
                            dz_change.append(dy_dz[2])
                        print("dy_change",dy_change)
                        print("dz_change",dz_change)
                        dy_change_arr = np.array(dy_change)
                        dz_change_arr = np.array(dz_change)                            
                        dy_change_row_avg.append(dy_change_arr.mean())
                        dz_change_row_avg.append(dz_change_arr.mean())
                        
                    diff_dy_change = float(dy_change_row_avg[1]-dy_change_row_avg[0])
                    diff_dz_change = float(dz_change_row_avg[1]-dz_change_row_avg[0])

                    print("diff_dy_change",diff_dy_change)
                    print("diff_dz_change",diff_dz_change)
                    #min_length = len(min(dy_change_row, key=len))
                    #mod_dy_change_row=[]
                    #mod_dy_change_row[0]=dy_change_row[0][:len(min(dy_change_row, key=len))]
                    #mod_dy_change_row[1]=dy_change_row[1][:len(min(dy_change_row, key=len))]

                    #mod_dz_change_row[0]=dz_change_row[0][:len(min(dz_change_row, key=len))]
                    #mod_dz_change_row[1]=dz_change_row[1][:len(min(dz_change_row, key=len))]
                    
                    #dy_row_1 = np.array(mod_dy_change_row[0])
                    #dy_row_2 = np.array(mod_dy_change_row[1])
                    #subtracted_dy_row_array = np.subtract(dy_row_2, dy_row_1)
                    #subtracted_dy_row = list(subtracted_dy_row_array)
                        
                    #dz_row_1 = np.array(dz_change_row[0])
                    #dz_row_2 = np.array(dz_change_row[1])
                    #subtracted_dz_row_array = np.subtract(dz_row_2, dz_row_1)
                    #subtracted_dz_row = list(subtracted_dz_row_array)
                    

                    print("last point reached -- not consider count")
                    points.append(i)
                    print("index",index)
                    print("count",count)
                    print("consider_count",consider_count)
                    print("not_consider_count",not_consider_count)
                    
                    index_z_mod = (index+1)*0.01*(diff_dz_change/(diff_dy_change+diff_dz_change))
                    #z_val = minz + ((consider_count+index_z_mod)*0.01)

                    index_y_mod = (index+1)*0.01*(diff_dy_change/(diff_dy_change+diff_dz_change))

                    print("Before -----------------------   dy_dz_change_rows[1]",dy_dz_change_rows[1])
                    print("index_y_mod",index_y_mod)
                    print("index_z_mod",index_z_mod)
                    for dy_dz in dy_dz_change_rows[1]:
                        dy_dz[1]=dy_dz[1]+ index_y_mod
                        dy_dz[2]=dy_dz[2]+ index_z_mod
        
                    print("After -----------------------   dy_dz_change_rows[1]",dy_dz_change_rows[1])
                    fitted_points = np.array(dy_dz_change_rows[1])
                    
                    #fitted_points = self.two_d_curve_fit(prev_to_last_point[0]+points[0],z_val,index_y_mod)
                    #if len(fitted_points)==0:
                    #    points=[]
                    #    count += 1
                    #    continue
                    if toggle:
                        fitted_points = fitted_points[::-1]

                    toggle =  not toggle
                    data_row_list.append(fitted_points.tolist())
                    print("added last row")
                    points=[]


                elif count==not_consider_count:
                    consider_count = count + 2
                    not_consider_count = consider_count + index# 5 for sealer,3 for base1, 4 for base2, 6 or 7 for clear 1 and clear 2                

            if count<consider_count:
                points.append(i)
                if len(points)>1:# or reaced_last_point:
                    z_val = minz + (count*0.01)-0.01
                    #if reaced_last_point and len(points)<2:
                    #    print("last point reached -- consider count")
                    #    fitted_points = self.two_d_curve_fit(prev_to_last_point[0]+points[0],z_val)
                    if len(points)>1:
                        fitted_points = self.two_d_curve_fit(points[0]+points[1],z_val,0)

                    if len(fitted_points)==0:
                        points=[]
                        count += 1
                        continue
                    if toggle:
                        fitted_points = fitted_points[::-1]

                    toggle =  not toggle
                    data_row_list.append(fitted_points.tolist())
                    dy_dz_change_rows[0]=dy_dz_change_rows[1]
                    dy_dz_change_rows[1]=fitted_points.tolist()
                    points=[]


            count += 1    
 
        #print("dy_dz_change_rows[0]",dy_dz_change_rows[0])
        #print("dy_dz_change_rows[1]",dy_dz_change_rows[1])
        
        #print("dy_change_row_avg",dy_change_row_avg)
        #print("dz_change_row_avg",dz_change_row_avg)
             
        data_row_list_mod = [pt for row in data_row_list for pt in row]
        # modifiying sequencing from top to bottom
        data_row_list_mod.reverse()
        #print("data_row_list_mod",data_row_list_mod)

        data_row_path_list = Path()
        
        for row in reversed(data_row_list):
            
            data_row_point_list = PathStamped()
            #print("row",row)

            for pt in reversed(row):
                data_row_point_append = Plannedpath()
                #print("pt",pt)
                data_row_point_append.x = pt[0]
                data_row_point_append.y = pt[1]
                data_row_point_append.z = pt[2]
                data_row_point_append.ox = pt[3]
                data_row_point_append.oy = pt[4]
                data_row_point_append.oz = pt[5]
                data_row_point_append.ow = 0.0#pt[6]
                data_row_point_append.is_index = False
                #print("data_row_path_append",data_row_point_append)
                data_row_point_list.path_msg.append(data_row_point_append)

            data_row_path_list.path.append(data_row_point_list)

        return data_row_list_mod,data_row_path_list       
        '''toggle = True
        path_rearanged_list = []  
        for plane in data_row_list:
            if toggle:
                plane.reverse()
                path_rearanged_list.append(plane)
            else:
                path_rearanged_list.append(plane)
            toggle =  not toggle'''


    def start(self):


        rate = rospy.Rate(10)
        print("Waiting for coat_process to be true")
        while not rospy.is_shutdown():

            #if rospy.get_param("coat_process")==True and self.received_sealer and self.received_base1 and self.received_base2 and self.received_clear1 and self.received_clear2 and not self.processed:
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

            if(self.received_sealer and self.received_base1 and self.received_base2 and self.received_clear1 and self.received_clear2):
                #print("publishing ... ")
                self.griddedCloud_pc.publish(self.sealer_gridded_cloud)
                self.sealercoat_pc.publish(self.sealer_data_seq_cloud)
                #self.basecoat1_pc.publish(self.basecoat1_data_seq_cloud)
                #self.basecoat2_pc.publish(self.basecoat2_data_seq_cloud)
                #self.clearcoat1_pc.publish(self.clearcoat1_data_seq_cloud)
                #self.clearcoat2_pc.publish(self.clearcoat2_data_seq_cloud)


                self.sealercoat.publish(self.sealer_data_row_path_list)
                #self.basecoat1.publish(self.basecoat1_data_row_path_list) 
                #self.basecoat2.publish(self.basecoat2_data_row_path_list)
                #self.clearcoat1.publish(self.clearcoat1_data_row_path_list)
                #self.clearcoat2.publish(self.clearcoat2_data_row_path_list)


            rate.sleep()


if __name__ == '__main__':
    SyntheticCloud()
    rospy.spin()
