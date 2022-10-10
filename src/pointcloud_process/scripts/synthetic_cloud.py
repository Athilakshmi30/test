#!/usr/bin/env python
from __future__ import print_function
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

class SyntheticCloud():

    def __init__(self):

        rospy.init_node('synthetic_cloud')

        self.pub = rospy.Publisher(
            "/max_depth_added_cloud", PointCloud2, queue_size=1)

        self.pub1 = rospy.Publisher(
            "/occlusion_removed_cloud", PointCloud2, queue_size=1)

        self.pub2 = rospy.Publisher(
            "/gridded_cloud", PointCloud2, queue_size=1)
        
        self.pub3 = rospy.Publisher(
            "/synthetic_cloud", PointCloud2, queue_size=1)


        self.pub4 = rospy.Publisher(
            "/data_seq_cloud", PointCloud2, queue_size=1)
        
        #self.trajectory_path = rospy.Publisher("/test_trajectory_path", Path, queue_size=1)

        rospy.Subscriber("/filtered_data_transformed",
                         PointCloud2, self.transformedDataCallBack)

        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                       PointField('y', 4, PointField.FLOAT32, 1),
                       PointField('z', 8, PointField.FLOAT32, 1)
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

        self.left_right_index_offset = 0.0762 #3 inches #0.1524 - 6 inches

        self.top_bottom_index_row_offset = 0.0762 #3 inches #0.1524 - 6 inches

        self.grid_width = 0.019#0.01
        self.grid_height = 0.019#0.01
        self.grid_depth = 0.019#0.01 

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

            self.point_cloud_list = list(point_cloud2.read_points(
                msg, field_names=['x', 'y', 'z'], skip_nans=True))
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
        cropped_cloud = [pt for pt in cloud if (
            (pt[2] < (z_range + 0.006)) and (pt[2] > (z_range - 0.006)))]
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

        print("start time : ",time.time())
        print("length of input cloud : ", len(cloud))
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
                        ending = time.time() 
    
                        total = total + ending - start
                        current_list = list(compress(cropped_cloud, bool_list))
        
                        if(len(current_list) > 0):
                            width_row.append(current_list[0])
                            width_boolean_row.append(True)
                            width_synthetic_row.append(self.centroid(conv_hull))
                            #bool_inverted_list = [not state for state in bool_list]
                            #cropped_cloud = list(
                            #    compress(cropped_cloud, bool_inverted_list))
                        else:
                            width_boolean_row.append(False)
                            width_row.append([0,0,0])
                            width_synthetic_row.append([0,0,0])
                        
                        
                        
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
        
        print("End time : ",time.time())

        print("Total createGrid: ",total)  

        #print(len(boolean_grid) , len(gridded_cloud))
        #print(len(boolean_grid[0]) , len(gridded_cloud[0]))
        #print(len(boolean_grid[0][1]) , len(gridded_cloud[0][1]))


        return gridded_cloud, boolean_grid, gridded_cloud_linear, synthetic_cloud_linear,synthetic_cloud

    def points_for_path(self,boolean_grid):
        
        start = time.time()
        print("start : ",start)
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
        print("Ending : ",ending)
        print("Total points for path : ",ending-start)
        
        return path_list , boolean_grid                      
            
    def path_plan(self,path_list , boolean_grid):#, gridded_cloud_list, synthetic_cloud_list):
        print("path_list[15]",path_list[15])
        toggle = True
        print("len_path_list",path_list)
        path_rearanged_list = []  
        for plane in path_list:
            if toggle:
                plane.reverse()
                path_rearanged_list.append(plane)
            else:
                path_rearanged_list.append(plane)
            toggle =  not toggle

        path_rearanged_row_list = []
        print("len_path_rearanged_list",path_rearanged_list)
        print("path_rearanged_list",path_rearanged_list)
        for plane in path_rearanged_list:
            new_row = []

            for row in plane:
                print("row",row)
                if toggle:
                    row.reverse()
                    path_rearanged_list.append(row)
                else:
                    path_rearanged_list.append(row)
                toggle =  not toggle
                new_row.append(row)
            path_rearanged_row_list.append(new_row)                   
        print("path_rearanged_row_list[15]",path_rearanged_row_list[15])
        return path_rearanged_row_list
    

    
    def curve_fit(self,data_list):



        X_data=[]
        Y_data=[]
        Z_data=[]
        for points in data_list:
            #print("points",points)
            for pt in points:
                if pt[0]==0:
                    continue
                X_data.append(pt[0])
                Y_data.append(pt[1])
                Z_data.append(pt[2])
        Z_data_arr = np.array(Z_data)

        #X_data = list(range(0, 5))
        #Y_data = list(range(0, 5))
        #random.shuffle(X_data)
        #random.shuffle(Y_data)
        #print("Z_data",Z_data)

        '''
        #2deg
        a, b, c, d = np.polyfit(X_data, Y_data, 3)
        fit_equation = lambda x: a * x ** 3 + b * x **2+ c*x+d
        '''
        '''
        #3deg
        a, b, c, d = np.polyfit(X_data, Y_data, 3)
        fit_equation = lambda x: a * x ** 3 + b * x **2+ c*x+d
        '''
        '''
        #4deg
        a, b, c, d,e = np.polyfit(X_data, Y_data, 4)
        fit_equation = lambda x: a * x ** 4 + b * x **3+ c*x**2+d*x+e
        '''
        '''
        #5deg
        a, b, c, d, e, f = np.polyfit(X_data, Y_data, 5)
        fit_equation = lambda x: a * x ** 5 + b * x **4+ c*x**3+d*x**2+e*x+f'''
        '''
        #8deg
        a, b, c, d, e, f,g,h,i = np.polyfit(X_data, Y_data, 8)
        fit_equation = lambda x: a * x ** 8 + b * x **7+ c*x**6+d*x**5+e*x**4+f*x**3+g*x**2+h*x+i ''' 


        #6deg
        a, b, c, d, e, f,g = np.polyfit(X_data, Y_data, 6)
        fit_equation = lambda x: a * x ** 6 + b * x **5+ c*x**4+d*x**3+e*x**2+f*x+g        
        '''
        #9 deg
        a, b, c, d, e, f,g,h,i,j = np.polyfit(X_data, Y_data, 9)
        fit_equation = lambda x: a * x ** 9 + b * x **8+ c*x**7+d*x**6+e*x**5+f*x**4+g*x**3+h*x**2+i*x+j
        '''


        #def generate_x_y(X, Y, f):
        #    X_fit = np.linspace(min(X), max(X))
        #    Y_fit = f(X_fit)


            #fig, ax1 = plt.subplots()
            #print("min(X)",min(X))
            #print("max(X)",max(X))

            #print("len(x_fit)",len(X_fit))
            #print("len(Y_fit)",len(Y_fit))

            #print("x_fit",X_fit)
            #print("Y_fit",Y_fit)

            #ax1.plot(X_fit, Y_fit, color='r', alpha=1, label='Polynomial fit')
            #ax1.scatter(X, Y, s=4, color='b', label='Data points')
            #ax1.set_title('Polynomial fit example')
            #ax1.legend()
            #plt.show()
            
        #generate_x_y(X_data, Y_data, fit_equation)
        X_fit = np.linspace(min(X_data), max(X_data))
        #X_fit = np.array(X_data)
        Y_fit = fit_equation(X_fit)
        return X_fit,Y_fit,Z_data_arr



    def sequencing(self, data):
        count = 0
        data_row_list = []
        X_list = []
        Y_list = []
        toggle = False
        for i in data:
            count += 1
            if count%2 == 1:
                X_fit,Y_fit,Z_data_arr = self.curve_fit(i)
                print("X_fit",X_fit)
                print("Y_fit",Y_fit)              
                if toggle:
                    X_fit = X_fit[::-1]
                    Y_fit = Y_fit[::-1]
                    print("after_rev_X_fit",X_fit)
                    print("after_rev_Y_fit",Y_fit)                

                toggle =  not toggle
                x_count = 0
                while x_count<len(X_fit):
                    avg_z = Z_data_arr.mean()
                    data_row_list.append([X_fit[x_count],Y_fit[x_count], avg_z])
                    X_list.append(X_fit[x_count])
                    Y_list.append(Y_fit[x_count])
                    x_count+=1        
        print("data_row_list",data_row_list)



        X_list_arr = np.array(X_list)
        y_list_arr = np.array(Y_list)
        fig, ax1 = plt.subplots()

        ax1.plot(X_list_arr, y_list_arr, color='r', alpha=1, label='Polynomial fit')
        ax1.scatter(X_list_arr, y_list_arr, s=4, color='b', label='Data points')
        ax1.set_title('Polynomial fit example')
        ax1.legend()
        plt.show() 
        return data_row_list       
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
        while not rospy.is_shutdown():

            if(self.received and not self.processed):
                self.max_depth_added_cloud_list = self.addDepthPlane(
                    self.point_cloud_list)
                self.max_depth_added_cloud = point_cloud2.create_cloud(
                    self.header, self.fields, self.max_depth_added_cloud_list)
                self.occlusion_removed_cloud, hidden_points_removed_list = self.removeHiddenPoints(
                    self.max_depth_added_cloud)
                plane_data_list, boolean_grid, gridded_cloud_list, synthetic_cloud_list,synthetic_cloud = self.createGrid(
                    self.point_cloud_list)
                #plane_data_list, boolean_grid, gridded_cloud_list, synthetic_cloud_list,synthetic_cloud = self.createGrid(hidden_points_removed_list)
                print("------------------------------------------------------------synthetic_cloud-------------------------------------------------------------------")
                print("len()",len(plane_data_list))
                #print("synthetic_cloud",synthetic_cloud[0])
                #self.curve_fit(plane_data_list[100]+plane_data_list[101])
                data_row_list = self.sequencing(plane_data_list)
                    

                print("-------------------------------------------------------------------------------------------------------------------------------")
                #print("------------------------------------------------------------gridded_cloud_list-------------------------------------------------------------------")
                #print("gridded_cloud_list",gridded_cloud_list)
                #print("-------------------------------------------------------------------------------------------------------------------------------")

                self.gridded_cloud = point_cloud2.create_cloud(
                    self.header, self.fields, gridded_cloud_list)
                self.synthetic_cloud = point_cloud2.create_cloud(
                    self.header, self.fields, synthetic_cloud_list)

                self.data_seq_cloud = point_cloud2.create_cloud(
                    self.header, self.fields, data_row_list)


                #path_list , boolean_grid  = self.points_for_path(boolean_grid)  
                #path_reordered_list_all =self.path_plan(path_list , boolean_grid)
                #_ = self.plan_path(path_reordered_list_all)

                
                self.processed = True

            if(self.processed):
                self.pub.publish(self.max_depth_added_cloud)
                self.pub1.publish(self.occlusion_removed_cloud)
                self.pub2.publish(self.gridded_cloud)
                self.pub3.publish(self.synthetic_cloud)
                self.pub4.publish(self.data_seq_cloud)
                
            rate.sleep()


if __name__ == '__main__':

    SyntheticCloud()
    rospy.spin()
