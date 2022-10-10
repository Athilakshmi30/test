#!/usr/bin/env python
from __future__ import print_function
import cv2
import requests
import json
import base64
import math
import numpy as np
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import roslibpy
import yaml


class Map():
    def __init__(self, ):

        # loading yaml

        with open("/home/axalta_ws/src/mir/scripts/config.yaml", "r") as stream:
            self.params = yaml.safe_load(stream)
        # print("params :", params)
        mapname = self.params["map_name"]

        # connection MIR
        self.ip = self.params["ip"]
        # auth = 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='
        auth = self.params['auth']
        self.host = 'http://' + self.ip + '/api/v2.0.0/'

        self.headers = {}
        self.headers['Content-Type'] = 'application/json'
        self.headers['Authorization'] = auth

        # connect bridge
        #self.client = roslibpy.Ros(host=self.ip, port=9090)
        #self.client.run()

        #if self.client.is_connected:
            #print("mir client connecte")

        # global data : map image
        self.debug = True
        self.map_image_path = "./mapimage.png"
        self.laser_image_path = "./laserimage.png"
        self.map_resolution = 0
        self.map_image = self.getMapImage(mapname)
        self.robot_arrow_length = 20
        self.robot_circle_radius = 5

        self.x_list = []
        self.y_list = []
        self.ros_init()

    def ros_init(self):
        # cv bridge
        self.bridge = CvBridge()
        print("Initializing node")
        rospy.init_node("mir_map", anonymous=True)
        self.pub = rospy.Publisher("/map_image", Image, queue_size=10)
        print("calling init node")
        self.init_node()

    def init_node(self):
        rate = rospy.Rate(5)
        try:
            while not rospy.is_shutdown():

                #image = self.getcombinedimage2()
                image = self.markRobotPos()
                image=self.fillmask(image,220)

                # self.listener = roslibpy.Topic(
                #    self.client, "/mirwebapp/web_path", "mirMsgs/WebPath")
                # self.listener.subscribe(self.callback)

                image_message = self.bridge.cv2_to_imgmsg(
                    image, encoding="bgr8")
                self.pub.publish(image_message)
            # rospy.Subscriber(
            #    "/move_base_node/MIRPlannerROS/local_plan", Path, callback)

                rate.sleep()
        except KeyboardInterrupt:
            print("keyboard exception")
            cv2.destroyAllWindows()

            #self.client.terminate()
        except Exception as e:
            print("some exception in init_node(): ", e.message)
            #self.client.terminate()

    def callback(self, data):
        print("inside callback")
        print("x_list: \n", self.x_list)
        print("y_list: \n", self.y_list)

        if not data['y']:
            self.y_list = []
            self.x_list = []
        else:
            print("here")
            self.y_list = data['y']
            self.x_list = data['x']

    def drawPath(self, image):
        print("len of path points : ", len(self.y_list))
        if len(self.y_list) > 0:

            for i in range(len(self.y_list)):
                print("here2")
                y_in_mtr = int(self.y_list[i])
                x_in_mtr = int(self.x_list[i])
                x, y = self.getCurrentPosInPixels((x_in_mtr, y_in_mtr))
                print("x,y in draw path : ", x, y)
                image = cv2.circle(image, (int(x), int(y)),
                                   1, (255, 0, 0, 1), -1)
        # cv2.imshow("image", image)
        # cv2.waitKey(5)

        return image

    def print_map_details(self, data):
        for item in data:
            print("url : {}\nguid : {}\nname : {}".format(
                item['url'], item['guid'], item['name']))
            print("\n\n")

    def getMap(self):
        maps = requests.get(self.host + 'maps', headers=self.headers)
        print("getMap() status : {}".format(maps.status_code))
        mapsJ = json.loads(maps.text)
        return(mapsJ)

    def getMapGuid(self, name):
        mapGuid = ""
        mapsJ = self.getMap()
        for map in mapsJ:
            if map["name"] == name:
                mapGuid = map["guid"]
        return mapGuid

    def getMapImage(self, name):

        mapGuid = self.getMapGuid(name)
        print("map guid : {}".format(mapGuid))
        map_data = requests.get(
            self.host + "maps/{}".format(mapGuid), headers=self.headers)
        print("get map data status : {}".format(map_data.status_code))
        map_dataJ = json.loads(map_data.text)
        # print("map data : {}".format(map_dataJ))
        map_base64 = map_dataJ["map"]
        self.map_resolution = map_dataJ["resolution"]
        print("map resolution : {}".format(self.map_resolution))
        imgdata = base64.b64decode(map_base64)
        # with open(map_image_path, 'wb') as f:
        #    f.write(imgdata)
        image = np.asarray(bytearray(imgdata), dtype=np.uint8)
        image = cv2.imdecode(image, cv2.IMREAD_UNCHANGED)
        print("Image shape in getMapImage() : ", image.shape)

        if self.debug:
            cv2.imwrite(self.map_image_path, image)

        return image

    def Status(self):
        print('Status')
        get_status = requests.get(self.host + 'status', headers=self.headers)
        get_statusJ = json.loads(get_status.text)
        return get_statusJ

    def getCurrentPos(self):
        status = self.Status()
        x = str(status['position']['x'])
        y = str(status['position']['y'])
        o = str(status['position']['orientation'])
        return(x, y, o)

    def convertToPixels(self, x, y):
        im_width = self.map_image.shape[1]
        im_height = self.map_image.shape[0]
        im_height_meters = im_height * self.map_resolution
        im_width_meters = im_width * self.map_resolution
        x_new = im_width_meters - x
        y_new = im_height_meters - y
        x_new = max(0, x_new)
        y_new = max(0, y_new)
        x_new = x_new / self.map_resolution
        y_new = y_new / self.map_resolution
        return x, y_new

    def getCurrentPosInPixels(self, pos=None):
        if pos == None:
            pos = self.getCurrentPos()

        im_width = self.map_image.shape[1]
        im_height = self.map_image.shape[0]
        # im_width_meters = im_width * self.map_resolution
        im_height_meters = im_height * self.map_resolution
        print("width : {}".format(im_width))

        print("pos : {}".format(pos))
        x_new = float(pos[0])
        y_new = float(pos[1])
        # x_new=im_width_meters-x_new
        y_new = im_height_meters - y_new
        x_new = max(0, x_new)
        y_new = max(0, y_new)
        x_new = x_new / self.map_resolution
        y_new = y_new / self.map_resolution
        print("x new : {}\ny new : {}".format(x_new, y_new))
        return x_new, y_new

    def getx2y2(self):
        _, _, theta = self.getCurrentPos()
        theta = float(theta)
        # x = d*cos(theta) where theta is in radians
        x_ = math.cos(theta * (math.pi / 180)) * self.robot_arrow_length
        y_ = math.sin(theta * (math.pi / 180)) * self.robot_arrow_length
        print("x2' : {}, y2' : {}".format(
            x_ / self.robot_arrow_length, y_ / self.robot_arrow_length))
        return x_, y_

    def markRobotPos(self):

        x_new, y_new = self.getCurrentPosInPixels()
        mpimage = self.map_image.copy()

        x2, y2 = self.getx2y2()
        x2 = x_new + x2
        y2 = y_new - y2
        image = cv2.arrowedLine(mpimage, (int(x_new), int(y_new)), (int(x2), int(y2)),
                                (255, 255, 0, 1), 2)
        image = cv2.circle(image, (int(x_new), int(y_new)),
                           self.robot_circle_radius, (255, 0, 0, 1), -1)
        # image = self.drawPath(image)
        # print(image.shape)
        # cv2.imshow("image",image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        return image

    def fillmask(self, image, fill_value):
        transparent_mask = image[:, :, 3] == 0

        image[transparent_mask] = [fill_value] * 4
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

    def getlaserimage(self, i):
        try:
            t = round(time.time() * 1000)
            print("time (ms):{}".format(t))
            laserhost = "http://{}/robot-images/laser_map/laser_map_{}.png?t={}".format(
                self.ip, i, t)
            response = requests.get(laserhost)
            if response.status_code == 200:
                data = response.content
                image = np.asarray(bytearray(data), dtype=np.uint8)
                laserimage = cv2.imdecode(image, cv2.IMREAD_UNCHANGED)

                # in_memory_file=io.BytesIO(response.content)
                # pil_image = Image.open(in_memory_file)
                # laserimage = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGBA2BGRA)
                if self.debug:
                    cv2.imwrite(self.laser_image_path, laserimage)

                return laserimage
            else:
                print("eror in response : {}".format(response.status_code))

        except Exception as e:
            print("exception in getlaserimage() : {}", format(e.message()))

    def getcombinedimage2(self):
        try:

            print("\n\n")
            # print(" laserimage type : {}   mapimage type : {}".format(
            #    laserimage.shape, mapimage.shape))

            #print("Reached here\n")
            laserimage = self.getlaserimage(0)
            #print("Reached here2\n")
            mapimage = self.markRobotPos()
            #print("Reached here3\n")
           # mapimage = self.drawPath(mapimage)

            print(mapimage.shape)
            mp_rows, mp_cols, _ = mapimage.shape
            print("Reached here4\n")
            print(laserimage.shape)
            ls_rows, ls_cols, _ = laserimage.shape
            print("Reached here5\n")
            out1_row_start = int(ls_rows / 2)
            out1_col_start = int(ls_cols / 2)
            xpos, ypos = self.getCurrentPosInPixels()

            out1 = np.full([2000, 2000, 4], 250, dtype=np.uint8)
            out2 = np.full([2000, 2000, 4], 0, dtype=np.uint8)
            print("out1 row start, mp_rows ",
                  out1_row_start, " ", mp_rows, "\n")

            out1[out1_row_start:out1_row_start + mp_rows,
                 out1_col_start:out1_col_start + mp_cols, :] = mapimage
            print("Reached here6\n")

            out2_row_start = int(out1_row_start + ypos - ls_rows / 2)
            out2_col_start = int(out1_col_start + xpos - ls_cols / 2)
            # out2[out2_row_start:out2_row_start + ls_rows,
            #     out2_col_start:out2_col_start + ls_cols, :] = laserimage
            out2[out2_row_start:out2_row_start + ls_rows,
                 out2_col_start:out2_col_start + ls_cols, :] = laserimage

            # out_image=cv2.addWeighted(out1,.8,out2,1,0)
            # out_image = cv2.add(out1, out2)
            out_image = cv2.add(out1, out2)
            out_image = self.fillmask(out_image, 220)

            # cv2.destroyAllWindows()

            return out_image

        except KeyboardInterrupt:
            print("keyboard exception")
            cv2.destroyAllWindows()

            self.client.terminate()
        except Exception as e:
            print("some exception in getCombinedImage2() : ", e.message)
            self.client.terminate()


'''
    def getcombinedimage(self):

        try:
            laserimage = self.getlaserimage(0)
            mapimage = self.map_image.copy()

            mp_rows, mp_cols, _ = mapimage.shape
            ls_rows, ls_cols, _ = laserimage.shape
            out1_row_start = int(ls_rows / 2)
            out1_col_start = int(ls_cols / 2)
            xpos, ypos = self.getCurrentPosInPixels()
            out2_row_start = int(out1_row_start + ypos - ls_rows / 2)
            out2_col_start = int(out1_col_start + xpos - ls_cols / 2)
            out1 = np.full([1000, 1000, 4], 250, dtype=np.uint8)
            out2 = np.full([1000, 1000, 4], 0, dtype=np.uint8)
            count = 0

            while_loop_start_time = time.time()

            print("\n\n\n\n")
            for i in range(20):
                for_loop_start_time = time.time()
                print("\n\n")
                print(" laserimage type : {}   mapimage type : {}".format(
                    laserimage.shape, mapimage.shape))
                if count > 1000:
                    count = 0

                laserimage = self.getlaserimage(i)
                mapimage = self.markRobotPos()

                out1[out1_row_start:out1_row_start + mp_rows,
                     out1_col_start:out1_col_start + mp_cols, :] = mapimage

                if count % 1 == 0:
                    print("count = {}".format(count))
                    xpos, ypos = self.getCurrentPosInPixels()
                out2_row_start = int(out1_row_start + ypos - ls_rows / 2)
                out2_col_start = int(out1_col_start + xpos - ls_cols / 2)
                # out2[out2_row_start:out2_row_start + ls_rows,
                #     out2_col_start:out2_col_start + ls_cols, :] = laserimage
                out2[out2_row_start:out2_row_start + ls_rows,
                     out2_col_start:out2_col_start + ls_cols, :] = laserimage

                # out_image=cv2.addWeighted(out1,.8,out2,1,0)
                # out_image = cv2.add(out1, out2)
                out_image = cv2.add(out1, out2)
                out_image = self.fillmask(out_image, 220)
                # out_image = self.fillmask(out1, 220)
                # print(type(image))
                # cv2.imwrite("./output.png", out_image)
                # cv2.imshow("image", out_image)
                # cv2.waitKey(5)
                # out_image = cv2.cvtColor(out_image, cv2.COLOR_BGRA2BGR)
                image_message = self.bridge.cv2_to_imgmsg(
                    out_image, encoding="bgr8")
                self.pub.publish(image_message)
                count = count + 1
                # cv2.destroyAllWindows()
                time.sleep(.1)
                # return out_image
                for_loop_end_time = time.time()
                print("for loop execution time = {}".format(
                    for_loop_end_time - for_loop_start_time))

            while_loop_end_time = time.time()
            print("while loop execution time = {}".format(
                while_loop_end_time - while_loop_start_time))
        except KeyboardInterrupt:
            print("keyboard exception")
            cv2.destroyAllWindows()
        except Exception as e:
            print("some exception in getCombinedImage() : ", e.message)
            cv2.destroyAllWindows()

        cv2.destroyAllWindows()
'''
'''
    def showMap(self):
        try:
            while(True):
                # img = markRobotPos(mapname)
                img = self.getcombinedimage2()
                cv2.imshow("image", img)
                cv2.waitKey(5)
        except KeyboardInterrupt:
            print("keyboard exception")
            cv2.destroyAllWindows()

            self.client.terminate()
        except Exception as e:
            print("some exception in shwowMap() : ", e.message)
            self.client.terminate()
'''

if __name__ == "__main__":
    map = Map()
    # map.showMap()
