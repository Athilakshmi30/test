#!/usr/bin/env python
from __future__ import print_function

import rospy
import numpy as np
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2 
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from scipy.spatial import Delaunay
from itertools import compress

def in_hull(p, hull):
        """
        Test if points in `p` are in `hull`

        `p` should be a `NxK` coordinates of `N` points in `K` dimensions
        `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the 
        coordinates of `M` points in `K`dimensions for which Delaunay triangulation
        will be computed
        """
        
        if not isinstance(hull,Delaunay):
            hull = Delaunay(hull)
        
        return hull.find_simplex(p)>=0
    
def reconstructedCallBack(msg):
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgb', 12, PointField.FLOAT32, 1)]
    fieldsn = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1)]          

    header = Header()
    header.frame_id = "mir_link"
    header.stamp = rospy.Time.now()

    cloud=msg
    points_list = []
    colourless_point_list = []
   
    pc2 = list(point_cloud2.read_points(cloud,field_names = ['x','y','z','rgb'], skip_nans=True))
    pc2_copy = pc2
    print("-------------pointcloud read completed-------------------")
    maxy = -1.0
    miny = 10.0
    maxx = -1.0
    minx = 10.0
    minz = 10.0
    maxz = -1.0 
    valclr=0
    for data in pc2:
        x = data[0]
        y = data[1]
        z = data[2] 
        valclr = data[3]
        #y = y -  
        if(maxy<y):
            maxy = y
        if(miny>y):
            miny = y 
        if(maxx<x):
            maxx = x
        if(minx>x):
            minx = x
        if(maxz<z):
            maxz = z
        if(minz>z):
            minz = z 

        
        #if (z < 1.35 and z > 0.62 and x > -0.075):
        #    points_list.append([x , y , z , data[3]])
        #    colourless_point_list.append([x, y , z ])  # --------------- black panel
        #if (z < 1.35 and z > 0.75 and x > -0.2 and x < 0.2):
        #    points_list.append([x , y , z , data[3]])
        #    colourless_point_list.append([x, y , z ])  # --------------- black panel

        #points_list.append([x+0.198 , y+0.762   , z -0.4236,data[3]])  
        #colourless_point_list.append([x+0.198, y+0.762 , z -0.4236]) #0.6 for black panel 36 inches

        points_list.append([x-0.034, y   , z ,data[3]])  
        colourless_point_list.append([x-0.034, y , z]) #0.6 for black panel 36 inches


    maxx = maxx-0.034
    minx = minx-0.034
    maxz = maxz 
    minz = minz     
    """y = miny
    z = maxz 
    x = minx
    ystep = 0.025
    xstep = 0.025
    while(y<maxy-0.1):
      x = minx
      while(x<maxx):
          points_list.append([x , y - 0.2, z , valclr])
          colourless_point_list.append([x, y-0.2 , z ]) 
          x = x + xstep
      y = y + ystep
    
    z = minz + 0.30
    y = miny
    zstep = 0.025
    while(z<maxz):
     x = minx
     while(x<maxx):
         points_list.append([x , y - 0.2, z , valclr])
         colourless_point_list.append([x, y - 0.2, z ]) 
         x = x + xstep
     z = z + zstep"""
    
    start_z = minz
    xy_plane_width = 0.005
    slicing_distance_z = 0.01
    z_new_list = []
    while(start_z <= maxz): 
        conv_hull = [[minx,miny,start_z],[minx,maxy,start_z],[maxx,miny,start_z],[maxx,maxy,start_z],[minx,miny,start_z+xy_plane_width],[minx,maxy,start_z+xy_plane_width],[maxx,miny,start_z+xy_plane_width],[maxx,maxy,start_z+xy_plane_width]]
        bool_list = in_hull(colourless_point_list,conv_hull)
        z_curr_list = list(compress(colourless_point_list,bool_list))
        if(len(z_curr_list) > 0):
            avgz = np.mean(np.array(z_curr_list),axis=0)[2]
            z_curr_list = [[val[0],val[1],avgz] for val in z_curr_list]
            z_new_list = z_new_list + z_curr_list
        start_z = start_z + xy_plane_width + slicing_distance_z


    """start_y = miny
    xz_plane_width = 0.005
    slicing_distance_y = 0.01
    y_new_list = []
    while(start_y <= maxy): 
        conv_hull = [[minx,start_y,minz],[minx,start_y,maxz],[maxx,start_y,minz],[maxx,start_y,maxz],[minx,start_y+xz_plane_width,minz],[minx,start_y+xz_plane_width,maxz],[maxx,start_y+xz_plane_width,minz],[maxx,start_y+xz_plane_width,maxz]]
        bool_list = in_hull(z_new_list,conv_hull)
        y_curr_list = list(compress(z_new_list,bool_list))
        if(len(y_curr_list) > 0):
            avgy = np.mean(np.array(y_curr_list),axis=0)[1]
            y_curr_list = [[val[0],avgy,val[2]] for val in y_curr_list]
            y_new_list = y_new_list + y_curr_list
        start_y = start_y + xz_plane_width + slicing_distance_y


    start_x = minx
    yz_plane_width = 0.005
    slicing_distance_x = 0.01
    x_new_list = []
    while(start_x <= maxx): 
        conv_hull = [[start_x,miny,minz],[start_x,maxy,minz],[start_x,miny,maxz],[start_x,maxy,maxz],[start_x+yz_plane_width,miny,minz],[start_x+yz_plane_width,maxy,minz],[start_x+yz_plane_width,miny,maxz],[start_x+yz_plane_width,maxy,maxz]]
        bool_list = in_hull(y_new_list,conv_hull)
        x_curr_list = list(compress(y_new_list,bool_list))
        if(len(x_curr_list) > 0):
            avgx = np.mean(np.array(x_curr_list),axis=0)[0]
            x_curr_list = [[avgx,val[1],val[2]] for val in x_curr_list]
            x_new_list = x_new_list + x_curr_list
        start_x = start_x + yz_plane_width + slicing_distance_x"""


    print("-----------------------filtered------------------------------")
    pc2_roi = point_cloud2.create_cloud(header, fields, points_list)
    print("--------------filtered cloud created-----------------")
    pc2_roi_nocolour = point_cloud2.create_cloud(header, fieldsn, z_new_list)
    
    pub.publish(pc2_roi)
    pub1.publish(pc2_roi_nocolour)

if __name__ == '__main__':
    rospy.init_node('range_filter_after_tf')
    pub = rospy.Publisher('points2', PointCloud2, queue_size=100)
    rate = rospy.Rate(10)
        
    pub = rospy.Publisher("/filtered_and_transformed",PointCloud2,queue_size=1)
    pub1 = rospy.Publisher("/zyx_sliced",PointCloud2,queue_size=1)
    
    
    rospy.Subscriber("/filtered_data_transformed",PointCloud2,reconstructedCallBack)
    rospy.spin()
    
