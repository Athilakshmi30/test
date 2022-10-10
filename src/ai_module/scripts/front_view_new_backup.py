# from matplotlib import testing
import numpy as np
import open3d as o3d
# import matplotlib.pyplot as plt
import cv2
import copy
import rospy
import os

def create_blank(width, height, rgb_color=(0, 0, 0)):
    """Create new image(numpy array) filled with certain color in RGB"""
    # Create black blank image
    image = np.zeros((height, width, 3), np.uint8)

    # Since OpenCV uses BGR, convert the color first
    color = tuple(reversed(rgb_color))
    # Fill image with color
    image[:] = color

    return image


class pcdConversion():
    def __init__(self, filePath):
        self.pcd = o3d.io.read_point_cloud(filePath)
        self.crop = None
        self.image = None
        
        #bounding box detection
        self.actual_bb = self.pcd.get_axis_aligned_bounding_box()
        self.actual_bb.color = (1, 0, 0)

        #Take a backup to show the rejected points for reference
        self.ori = copy.deepcopy(self.pcd)
        self.ori.paint_uniform_color([1, 0.706, 0])
    
    def cropPoints(self,crop_xy=True):
        self.bounding_polygon = self.actual_bb.get_box_points()
        print(np.asarray(self.bounding_polygon))
        if crop_xy:    
            self.bounding_polygon = np.array([[-1,-1,0],
                    [2,-1,0],
                    [-1,2,0],
                    [-1,-1,2],
                    [2,2,2],
                    [-1,2,2],
                    [2,-1,2],
                    [2,2,0]])

        # Remove points in Z axis which is away from 2m
        vol = o3d.visualization.SelectionPolygonVolume()
        vol.orthogonal_axis = "Z"
        vol.axis_max = 2.0
        vol.axis_min = -5.0
        vol.bounding_polygon = o3d.utility.Vector3dVector(self.bounding_polygon)

        self.crop = vol.crop_point_cloud(self.pcd)
        o3d.visualization.draw_geometries([self.crop])

    def computeOffshift(self):

        if self.crop == None:
            self.crop = copy.deepcopy(self.pcd)

        # Get bounding box corners and find out min and max for X & Y
        corners = np.asarray(self.actual_bb.get_box_points())
        x = corners[:,0]
        y= corners[:,1]
        xmin, xmax = min(x), max(x)
        ymin, ymax = min(y), max(y)
        print("XMin , XMax" , xmin,xmax,ymin,ymax)
        
#         o3d.visualization.draw_geometries([self.crop])
        
        # Find width and height in m
        width = xmax - xmin
        height = ymax - ymin
        print("(w:h in meter)",width,height)

        # Shift all X point to make from origin
        x1 = np.asarray(self.pcd.points)[:,0]
        print("x1[0] , xmin",x1[0] , xmin)
        np.asarray(self.pcd.points)[:,0] = x1 - xmin #-0.2

        # Shift all Y point to make from origin
        y1 = np.asarray(self.pcd.points)[:,1]
        np.asarray(self.pcd.points)[:,1] = y1 - ymin #-0.2

        # Shift x and y min from origin to max
        xmin, xmax = xmin-xmin, xmax-xmin
        ymin, ymax = ymin-ymin, ymax-ymin

        # Compute image size
        width, height = int(xmax*1000//8), int(ymax*1000//8)
        print("(w:h in pixel)",width,height)

        # Prepare blank image and depth image
        imgcol = (255, 255, 255) # Fill White as a background
        self.image = create_blank(width, height, rgb_color=imgcol)
        depth = np.zeros((height, width), np.float32)
        depth = depth+2000

        p = np.asarray(self.pcd.points)
        c = np.asarray(self.pcd.colors)
        # print("points : ",len(p))
        # print("colors : ",len(c))

        # Read X & Y in m and convert to pixel
        # Copy the color information 
        # read the depth info and discard any thing data placed behind
        for i in range(len(p)):
            x = int(p[i,0] * 1000 // 8)
            y = int(p[i,1] * 1000 // 8)
            z = p[i,2]
          
            
            if depth[y-1,x-1] > z:
                self.image[y-1,x-1] = (c[i] * 255.0)
                depth[y-1,x-1] = z

        depth[depth>100] = 0
        # plt.imshow(depth,'gray')
        # plt.imshow(self.image)

        self.image = cv2.cvtColor(self.image,  cv2.COLOR_BGR2RGB)
    
    def displayImage(self):
        cv2.imshow("3D to 2D",self.image)
        while True:
            if cv2.waitKey(20) & 0xFF == 27:
                break
        cv2.destroyAllWindows()
    
    def saveImage(self, fileOP):
        cv2.imwrite(fileOP, self.image)

    def display3Dcloudpoint(self,full=True,crop=True,bb=True):
        points = []
        if full:
            points.append(self.ori)
        if crop:
            points.append(self.crop)
        if bb : 
            points.append(self.actual_bb)
        o3d.visualization.draw_geometries(points)

    def displayRawPCD(self):
        o3d.visualization.draw_geometries([self.pcd , self.actual_bb])

    def savePCD(self, fileOP):
        o3d.io.write_point_cloud(fileOP,self.pcd,True)

# def Snapshot(req):
#     print("Executing as Main")
#     # filePath =r"C:\Users\1984065\Downloads\Recfusion_outputs\Recfusion_outputs\depth_200\pcd_blue.ply"
#     if os.path.isfile(r"C:\Users\axalta\Desktop\shared\pointcloud\pointcloud_manualsegmented.ply"):
#         input_cloud = r'C:\Users\axalta\Desktop\shared\pointcloud\pointcloud_manualsegmented.ply'
#         pcd = pcdConversion(input_cloud)
#         pcd.cropPoints(False)
# #       pcd.displayRawPCD()
#         pcd.computeOffshift()
# #       pcd.displayImage()
#         pcd.saveImage("/home/axalta_ws/img/manual_cropped_front_view_image.png")

#         #pcd.display3Dcloudpoint(True,True,False)

# def Start_server():
#     front_view_service = rospy.Service('ai_modeule/get_front_view_server', GetFrontView, Snapshot)

def Snapshot(input_cloud):
    print("Executing as Main")
    # filePath =r"C:\Users\1984065\Downloads\Recfusion_outputs\Recfusion_outputs\depth_200\pcd_blue.ply"
    pcd = pcdConversion(input_cloud)
    pcd.cropPoints(False)
#     pcd.displayRawPCD()
    pcd.computeOffshift()
#     pcd.displayImage()
    pcd.saveImage("Output/Image.png")
    #pcd.display3Dcloudpoint(True,True,False)
# if __name__ == '__main__':
#     Start_server()

# if __name__ == '__main__':
#     print("Executing as Main")
#     filePath =r"C:\Users\1984065\Downloads\Recfusion_outputs\Recfusion_outputs\depth_200\pcd_blue.ply"
#     pcd = pcdConversion(filePath)
#     pcd.cropPoints(False)
# #     pcd.displayRawPCD()
#     pcd.computeOffshift()
# #     pcd.displayImage()
#     pcd.saveImage(r"C:\Users\1984065\Downloads\Recfusion_outputs\Recfusion_outputs\depth_200\pcd_blue.png")
#     #pcd.display3Dcloudpoint(True,True,False)