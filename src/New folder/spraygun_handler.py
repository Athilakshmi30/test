#!/usr/bin/env python
from __future__ import print_function
from subprocess import Popen, PIPE
import os
from dashboard.srv import *
from dashboard.msg import *
import rospy
#from report.msg import Report
from mir.srv import *
import time
import yaml


class SpraygunHandler():

    def __init__(self):
        pass

    def update_yaml(self, data):

        with open("/home/axalta_ws/src/dashboard/config/dashboard_persistent_settings.yaml", 'r+') as file:
            doc = yaml.safe_load(file)
            if(data.keys()[0] in doc.keys()):
                del doc[data.keys()[0]]
            file.seek(0)
            documents = yaml.dump(doc, file, default_flow_style=False)
            file.truncate()
        with open("/home/axalta_ws/src/dashboard/config/dashboard_persistent_settings.yaml", 'a') as file:
            documents = yaml.dump(data, file, default_flow_style=False)

    def get_coat_settings(self, index):
        libe = ['Sealer Coat', 'Base Coat 1',
                'Base Coat 2', 'Clear Coat 1', 'Clear Coat 2']
        coat_num = libe[index]
      #added left,right,top,bottom offset
        set_list_ = ['axalta/ccscore/dashboard/SPRAYGUN_NAME_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_NO_OF_LAYERS_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_AIR_PRESSURE_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_COAT_NUMBER_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_INDEX_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_DELAY_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_LEFT_OFFSET_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_RIGHT_OFFSET_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_TOP_OFFSET_' + coat_num,
                     'axalta/ccscore/dashboard/SPRAYGUN_BOTTOM_OFFSET_' + coat_num,
                     ]
        print(" im printing set_list_")
        print(set_list_)
        return set_list_

    def read_settings_params(self, index):
        get_settings = []
        set_list_ = self.get_coat_settings(index)
        for k in set_list_:
            get_settings.append(rospy.get_param(k))
        return get_settings

    def handle_spray_gun(self, req):
        print('-----------------------spraygun_settings_server------------------------------------------------')
        print('request:', req)
        #print('response:', 'True')
        print('time:', time.time())
     #added left,right,top,bottom offset
        settings = SpraygunSettingsResponse()
        for i in range(5):
            sett = GunSettings()
            get_settings = self.read_settings_params(i)
            sett.name = get_settings[0]
            sett.no_of_layers = get_settings[1]
            sett.air_pressure = get_settings[2]
            sett.coat_number = get_settings[3]
            sett.traverse_speed = get_settings[4]
            sett.index = get_settings[5]
            sett.gun_distance1 = get_settings[6]
            sett.delay = get_settings[7]
            sett.left_offset=get_settings[8]
            sett.right_offset=get_settings[9]
            sett.top_offset=get_settings[10]
            sett.bottom_offset=get_settings[11]
            settings.spraygun_settings.append(sett)

        #print('response:', settings)
        print('time:', time.time())
        return settings

    def load_spraygun_params(self, set_dict_):
        try:
            for k, v in set_dict_.items():
                self.update_yaml({k: v})

            for k, v in set_dict_.items():
                rospy.set_param(k, v)
            return True

        except Exception as e:
            print("Exception occured :", e)
            return False

    def handle_spray_gun_edit(self, req):
        print('-----------------------spraygun_settings_edit_server------------------------------------------------')
        print('request:', req)

        print('time:', time.time())
        if(req.coat_number > 0):
            coat_num = str(req.name) + str(req.coat_number)
        else:
            coat_num = str(req.name)
        #added left,right,top,bottom offset
        set_dict_ = {'axalta/ccscore/dashboard/SPRAYGUN_NAME_' + coat_num: req.name,
                     'axalta/ccscore/dashboard/SPRAYGUN_NO_OF_LAYERS_' + coat_num: req.no_of_layers,
                     'axalta/ccscore/dashboard/SPRAYGUN_AIR_PRESSURE_' + coat_num: req.air_pressure,
                     'axalta/ccscore/dashboard/SPRAYGUN_COAT_NUMBER_' + coat_num: req.coat_number,
                     'axalta/ccscore/dashboard/SPRAYGUN_TRAVERSE_SPEED_' + coat_num: req.traverse_speed,
                     'axalta/ccscore/dashboard/SPRAYGUN_INDEX_' + coat_num: req.index,
                     'axalta/ccscore/dashboard/SPRAYGUN_DISTANCE1_' + coat_num: req.gun_distance1,
                     'axalta/ccscore/dashboard/SPRAYGUN_DELAY_' + coat_num: req.delay,
                     'axalta/ccscore/dashboard/SPRAYGUN_LEFT_OFFSET_' + coat_num: req.left_offset,
                     'axalta/ccscore/dashboard/SPRAYGUN_RIGHT_OFFSET_' + coat_num: req.right_offset,
                     'axalta/ccscore/dashboard/SPRAYGUN_TOP_OFFSET_' + coat_num: req.top_offset,
                     'axalta/ccscore/dashboard/SPRAYGUN_BOTTOM_OFFSET_' + coat_num: req.bottom_offset,
                     }

        param_status = self.load_spraygun_params(set_dict_)
        print('response:', SpraygunSettingsEditResponse(param_status))
        return SpraygunSettingsEditResponse(param_status)
        
if __name__ == '__main__':
    rospy.init_node('Spraygun_Handler')
    SpraygunHandler()
    rospy.spin()
