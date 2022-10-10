#!/usr/bin/env python

import os
import sys
import array
import struct
import rospy
import signal
import time
from numpy.distutils.system_info import boost_python_info
from fcntl import ioctl
from std_msgs.msg import Bool, String

isJoystickConnected = False


def find_joystick():
    foundJS = False
    global isJoystickConnected
    for fn in os.listdir('/dev/input'):
        # print(fn)
        if fn.startswith('js'):
            foundJS = True
            break
        else:
            continue
    isJoystickConnected = foundJS

    if isJoystickConnected:
        print('joystick connected')

        return True
    else:
        print('no joystick found')
        return False


def open_joy_stream():
    fn = '/dev/input/js1'
    #fn = '/devices/platform/soc/3f980000.usb/usb1/1-1/1-1.2/1-1.2:1.0/input/input11'
    try:
        jsdev = open(fn, 'rb')
    except:
        print("trying js0")
        fn = '/dev/input/js0'
    try:
        jsdev = open(fn, 'rb')
    except:
        pass
    return jsdev


def buffer_test():

    joystick_data = 'nil'

    rospy.init_node('Joystick', anonymous=False)
    pub = rospy.Publisher('/joystick', String, queue_size=1)

    buf = array.array('B', [0] * 64)
    print(len(buf))

    jsdev = open_joy_stream()
    print(type(jsdev))
    #ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf)
    #js_name = buf.tostring().rstrip(b'\x00').decode('utf-8')
    #print('Device name: %s' % js_name)

    buf = array.array('B', [0])
    ioctl(jsdev, 0x80016a11, buf)
    num_axis = buf[0]
    print('No of axes: %s' % num_axis)

    buf = array.array('B', [0])
    ioctl(jsdev, 0x80016a12, buf)  # JSIOCGAXES
    num_button = buf[0]
    print('No of button: %s' % num_button)

    axis_states = {}
    button_states = {}

    axis_names = {
        0x00: 'x',
        0x01: 'y',
        0x02: 'z',
        0x03: 'rx',
        0x04: 'ry',
        0x05: 'rz',
        0x06: 'throttle',
        0x07: 'rudder',
        0x08: 'wheel',
        0x09: 'gas',
        0x0a: 'brake',
        0x10: 'hat0x',
        0x11: 'hat0y',
        0x12: 'hat1x',
        0x13: 'hat1y',
        0x14: 'hat2x',
        0x15: 'hat2y',
        0x16: 'hat3x',
        0x17: 'hat3y',
        0x18: 'pressure',
        0x19: 'distance',
        0x1a: 'tilt_x',
        0x1b: 'tilt_y',
        0x1c: 'tool_width',
        0x20: 'volume',
        0x28: 'misc',
    }

    button_names = {
        0x120: 'trigger',
        0x121: 'thumb',
        0x122: 'thumb2',
        0x123: 'top',
        0x124: 'top2',
        0x125: 'pinkie',
        0x126: 'base',
        0x127: 'base2',
        0x128: 'base3',
        0x129: 'base4',
        0x12a: 'base5',
        0x12b: 'base6',
        0x12f: 'dead',
        0x130: 'a',
        0x131: 'b',
        0x132: 'c',
        0x133: 'x',
        0x134: 'y',
        0x135: 'z',
        0x136: 'tl',
        0x137: 'tr',
        0x138: 'tl2',
        0x139: 'tr2',
        0x13a: 'select',
        0x13b: 'start',
        0x13c: 'mode',
        0x13d: 'thumbl',
        0x13e: 'thumbr',

        0x220: 'dpad_up',
        0x221: 'dpad_down',
        0x222: 'dpad_left',
        0x223: 'dpad_right',

        # XBox 360 controller uses these codes.
        0x2c0: 'dpad_left',
        0x2c1: 'dpad_right',
        0x2c2: 'dpad_up',
        0x2c3: 'dpad_down',
    }
    rate = rospy.Rate(1000)
    not_connected = True
    while not rospy.is_shutdown():
        joy_connection = find_joystick()
        print(joy_connection , not_connected)
        if(joy_connection):
            
            if not_connected:
                time.sleep(5)
                jsdev = open_joy_stream()
                not_connected = False
            try:
                evbuf = jsdev.read(8)
                timeval, value, jtype, number = struct.unpack('IhBB', evbuf)
                print(timeval, value, jtype, number)
                if evbuf and number != 2:
                    if jtype & 0x02 and (number == 0 or number == 1):
                        print(timeval, value, jtype, number)
                        if value > 1000 and (number == 1):
                            joystick_data = 'down'
                        elif value < -2 and (number == 1):
                            joystick_data = 'up'
                        elif value > 1000 and (number == 0):
                            joystick_data = 'right'
                        elif value < 0 and (number == 0):
                            joystick_data = 'left'
                        else:
                            joystick_data = 'nil'
                        pub.publish(joystick_data)
            except Exception as e:
                print(e)
                continue

        else:
            not_connected = True

        rate.sleep()


if __name__ == '__main__':
    print('Main Function')
    find_joystick()
    try:
        buffer_test()
    except(KeyboardInterrupt, SystemExit):
        pass

