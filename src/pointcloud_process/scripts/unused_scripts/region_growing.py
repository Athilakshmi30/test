#!/usr/bin/env python
import math
import numpy as np
from sklearn.neighbors import KDTree
import rospy
import datause
from operator import itemgetter
from sensor_msgs.msg import PointCloud2, PointField 
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Header


class RegionGrowing():

    def __init__(self):
        self.flag = 0
        self.points = []
        #self.axes_shifted = []
        self.axes = []
        
       
        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                ]
        rospy.Subscriber("filtered_data", PointCloud2, self.region_callback)    
        self.stop = True
        self.msg_count = 0
        
    def region_callback(self,msg):
        
        
        if(self.stop):
            rospy.set_param("axalta/ccscore/dashboard/CURRENT_PROCESS","Region Growing in progress....")
            print 'got something!!'
            
            pointlist,g_in_y = self.region_grow(msg)
            self.points = pointlist
            self.axes = g_in_y
            
            self.flag = 1
   
            self.stop = False

            #rospy.set_param("axalta/ccscore/dashboard/LIDAR_OFF_TRIGGER",True)
            rospy.set_param("axalta/ccscore/dashboard/COMPLETION_PERCENTAGE",70) 

    """def percentile_based_outlier(self,data, threshold=99):

        diff = (100 - threshold) / 2.0
        minval, maxval = np.percentile(data, [diff, 100 - diff])
        print(minval,maxval)
        return minval,maxval

    def percentile_based_outlier_max(self,data, threshold=82):
        minval, maxval = self.percentile_based_outlier(data, threshold) 
        print(minval,maxval)   
        return (data > maxval)"""

    def region_grow_in_y(self,cloudoperate,avg_max_y,avg_min_y):
        #reg_grow = cloudoperate
        act_miny = rospy.get_param('axalta/ccscore/paintjobprocess/MINY')
        act_maxy = rospy.get_param('axalta/ccscore/paintjobprocess/MAXY') 
        reg_grow = [col for row in cloudoperate for col in row]
        reg_grow_min = []
        reg_grow_max = []  
        for cnt,row in enumerate(cloudoperate):
            for cntval,col in enumerate(row):
               if(col[1]<=avg_min_y+0.08): 
                   reg_grow_min.append(col)
               elif(col[1]>=avg_max_y-0.08):
                   reg_grow_max.append(col) 
        for pt in reg_grow_min:
            reg_grow.append([pt[0],pt[1]-(abs(act_miny-avg_min_y))-0.02,pt[2]] )
        for pt in reg_grow_max:
            reg_grow.append([pt[0],pt[1]+(abs(act_maxy-avg_max_y))+0.02,pt[2]] )
         
        return reg_grow

    def region_grow_in_z(self,reg_grow,minz,maxz):
        act_minz = rospy.get_param('axalta/ccscore/paintjobprocess/MINZ')
        act_maxz = rospy.get_param('axalta/ccscore/paintjobprocess/MAXZ') 
        full_region = reg_grow
        full_min_z = []
        full_max_z = []
        for point in reg_grow:
              if(point[2]<=minz+0.12): 
                  full_min_z.append(point)
              elif(point[2]>maxz-0.12):
                  full_max_z.append(point) 
        print("selection area z done")  
        for pt in full_min_z:
            full_region.append([pt[0],pt[1],pt[2]-0.1] )
        for pt in full_max_z:
            full_region.append([pt[0],pt[1],pt[2]+0.1] )
        return full_region

    def distance_between_two_pts(self,pt1,pt2):
        return math.sqrt(pow((pt2[0]-pt1[0]),2)+pow((pt2[1]-pt1[1]),2)+pow((pt2[2]-pt1[2]),2)) 
 
    


    def point_gen(self,input_seq):
        maxy = -1.0
        miny = 10.0
        maxx = -1.0
        minx = 10.0
        index = 0 
        points = []
        axes = []
        axes_shifted = []
        
       
        l = list(input_seq)
        input_length = len(l)
        print("len before region growing: ",input_length)
        Z_thresh = 0.002
        
        cloudoperate = []
        m = sorted(l, key=itemgetter(2))
        #print("sorted: ",m)
        minz = m[0][2]
        maxz = m[len(m)-1][2]
        avgx = sum([m[i][0] for i in range(len(m))])/len(m)
        print("avg of x ",avgx)
        print("min max z:", minz,maxz)
        newrow = []
        range_for_row = []
        set_point = m[0][2]   
        for it in range(input_length-1):
                 
                if(abs(set_point-m[it][2]) <= Z_thresh):
                    #print(m[it][2],m[it+1][2])      
                    x = m[it][0]
                    y = m[it][1]
                    if(maxy<y):
                        maxy = y
                    if(miny>y):
                        miny = y
                    if(maxx<x):
                        maxx = x
                    if(minx>x):
                        minx = x     
                    #if(x>avgx):
                    #    newrow.append([avgx,m[it][1],m[it][2]])
                    #else:
                    newrow.append([m[it][0],m[it][1],m[it][2]])  
                else:
                    set_point = m[it][2]
                    if(len(newrow)>40): #minimum 100 points should be there for a valid row
                        print("row " , index , len(newrow))
                        range_for_row.append([minx,maxx,miny,maxy])
                        maxy = -1.0
                        miny = 10.0
                        maxx = -1.0
                        minx = 10.0
                        cloudoperate.append(newrow) 
                        #print(cloudoperate)
                        newrow = []
                        index = index + 1  
                    
        #cld = cloudoperate            
        print("len of rangerow",len(range_for_row)) 
        print("len of cloudoperate",len(cloudoperate))   
        #print(range_for_row)
        #avg_max_y = sum([range_for_row[i][3] for i in range(len(range_for_row))])/len(range_for_row)
        avg_max_y = max([range_for_row[i][3] for i in range(len(range_for_row))])#maximum y
        #avg_min_y = sum([range_for_row[i][2] for i in range(len(range_for_row))])/len(range_for_row)
        avg_min_y = min([range_for_row[i][2] for i in range(len(range_for_row))])#minimum y
        print("miny,maxy",miny,maxy)
        reg_grow = self.region_grow_in_y(cloudoperate,avg_max_y,avg_min_y)
        print("reg_grow_len",len(reg_grow))
        print(reg_grow)
        act_miny = rospy.get_param('axalta/ccscore/paintjobprocess/MINY')
        act_maxy = rospy.get_param('axalta/ccscore/paintjobprocess/MAXY')
        act_minz = rospy.get_param('axalta/ccscore/paintjobprocess/MINZ')
        act_maxz = rospy.get_param('axalta/ccscore/paintjobprocess/MAXZ')
  
        full_region = self.region_grow_in_z(reg_grow,minz,maxz)
        #grown_in_y = [col for row in reg_grow for col in row]
        grown_in_y = reg_grow
        #points = [col for row in cloudoperate for col in row]
        points = [col for col in full_region if(col[1]<act_maxy and col[1]>act_miny and col[2] < act_maxz and col[2] > act_minz)]
        """del_y = 0.03
        y_wise = act_miny
        xminarr = []
        xmaxarr = []
        while(y_wise < act_maxy):
            minx = 10.0
            maxx = -1.0
            sumx
            for pt in points:
                if(pt[1]<(y_wise+del_y)):
                    
                    if(pt[0]>maxx):
                        maxx = pt[0]
                    elif(pt[0]<minx):
                        minx = pt[0]
                    print(pt[1],maxx,minx)  
            xminarr.append(minx)
            xmaxarr.append(maxx)
            y_wise = y_wise+del_y

        print("xminarr--",xminarr)
        print("xmaxarr--",xmaxarr) """
        print("after region growing :",len(points))
        #return points,cld[0]
        return points,grown_in_y

    
    def region_grow(self,cloud):
        
        generator_trav = pc2.read_points(cloud, field_names = ['x','y','z'], skip_nans = True)
        
        points,grown_in_y = self.point_gen(generator_trav)
        
        return points,grown_in_y

if __name__ == '__main__':
    try:
            rospy.init_node('region_growing', anonymous=True)
                    
            obj = RegionGrowing()
            header = Header()
            header.frame_id = "os_sensor"
            rate = rospy.Rate(10)
            pub = rospy.Publisher("region_grown", PointCloud2, queue_size=2)
            pub2 = rospy.Publisher("grown_only_in_y", PointCloud2, queue_size=2)
                     
            while not rospy.is_shutdown():
                if(rospy.has_param("axalta/ccscore/dashboard/RESTART_REGGROWNODE_TRIGGER") and rospy.get_param("axalta/ccscore/dashboard/RESTART_REGGROWNODE_TRIGGER")):
                    obj.flag=0 
                    obj.points = []
                    obj.axes = []
                    obj.stop = True
                    obj.msg_count = 0
                    rospy.set_param("axalta/ccscore/dashboard/RESTART_SURFACENODE_TRIGGER",True)  
                    rospy.set_param("axalta/ccscore/dashboard/RESTART_REGGROWNODE_TRIGGER",False)  
                if(obj.flag):
                    pcld2 = pc2.create_cloud(header, obj.fields, obj.points)
                    pcld2.header.stamp = rospy.Time.now()
                    pub.publish(pcld2)
                    
                    npcld3 = pc2.create_cloud(header, obj.fields, obj.axes)
                    npcld3.header.stamp = rospy.Time.now()
                    pub2.publish(npcld3)

                rate.sleep()
            rospy.spin()

    except rospy.ROSInterruptException:
        pass


