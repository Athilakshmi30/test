#!/bin/sh

#sudo dnsmasq -C /dev/null -kd -F 10.5.5.50,10.5.5.100 -i enp1s0 --bind-dynamic &


nc -w 1 os-992104000607.local 7501 < /home/axalta/axalta_ws/src/exit_job/launch/lidar_normal_commands.txt


