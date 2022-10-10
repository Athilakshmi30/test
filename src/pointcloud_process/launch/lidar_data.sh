#!/bin/bash

source /home/axalta/axalta_ws/devel/setup.bash

roslaunch ouster_ros ouster.launch sensor_hostname:=os-992104000607.local udp_dest:=192.168.12.24 lidar_mode:=2048x10 image:=true &
