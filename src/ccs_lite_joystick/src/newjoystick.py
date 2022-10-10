#!/usr/bin/env python

import os, sys, array, struct
import rospy
import signal
from ccs_lite_msgs.msg import Joystick
from numpy.distutils.system_info import boost_python_info
from fcntl import ioctl
from std_msgs.msg import Bool,String

isJoystickConnected = None


def find_joystick():
    global isJoystickConnected
    for fn in os.listdir('/dev/input'):
        print(fn)
        if fn.startswith('js0'):
            isJoystickConnected = bool(1)
            break
        else:
            continue

    if isJoystickConnected:
        print('joystick connected')
    else:
        print('no joystick found')


def buffer_test():

    joystick_data = Joystick()
    joystick_data.leftAxis.x = 0.0
    joystick_data.leftAxis.y = 0.0
    joystick_data.leftAxis.z = 0.0

    rospy.init_node('Joystick', anonymous=False)
    pub = rospy.Publisher('/joystick', String, queue_size=10)

    buf = array.array('B', [0] * 64)
    print(len(buf))
    fn = '/dev/input/js0'
    #fn = '/devices/platform/soc/3f980000.usb/usb1/1-1/1-1.2/1-1.2:1.0/input/input11'
    jsdev = open(fn, 'rb')
    print(jsdev)
    ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf)
    js_name = buf.tostring().rstrip(b'\x00').decode('utf-8')
    print('Device name: %s' % js_name)

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

    axis_map = []
    button_map = []

    buf = array.array('B', [0] * 0x40)
    ioctl(jsdev, 0x80406a32, buf)
    for axis in buf[:num_axis]:
        axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
        axis_map.append(axis_name)
        axis_states[axis_name] = 0.0
    print('%d axes found: %s' % (num_axis, ', '.join(axis_map)))

    buf = array.array('H', [0] * 200)
    ioctl(jsdev, 0x80406a34, buf)
    for btn in buf[:num_button]:
        btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
        button_map.append(btn_name)
        button_states[btn_name] = 0
    print('%d buttons found: %s' % (num_button, ', '.join(button_map)))

    while not rospy.is_shutdown():
        evbuf = jsdev.read(8)
        if evbuf:
            time, value, jtype, number = struct.unpack('IhBB', evbuf)

            #print("jtype %s: " % jtype)
            #print("number %s: " % number)
            #if jtype & 0x80:
            #    print("\n (initial)")

            """if jtype & 0x01:
                button = button_map[number]
                if button:
                    button_states[button] = value
                    if value:
                        print("%s pressed" % button)
                    else:
                        print("%s released" % button)
		(2733250, -32767, 2, 7)up
		(2739630, 0, 2, 7)
		(2741270, 32767, 2, 7)down
		(2743770, 0, 2, 7)
		(2746000, 32767, 2, 6)left
		(2747630, 0, 2, 6)
		(2748310, -32767, 2, 6)right
		(2748840, 0, 2, 6)

		(2790190, -32767, 2, 1)up
		(2790340, -2, 2, 1)nil
		(2792230, 32767, 2, 1)down
		(2792350, -2, 2, 1)nil
		(2793160, -32767, 2, 0)left
		(2793300, 0, 2, 0)nil
		(2794250, 32767, 2, 0)right 
		(2794370, 0, 2, 0)nil

	"""

            if jtype & 0x02:
                # fvalue = value / 32767.0
                print(time, value, jtype, number)
                if value>1000 and (number == 7 or number == 1 ):
                    joystick_data = 'down'
                elif value<-2 and (number == 7 or number == 1 ):
                    joystick_data = 'up'
                elif value>1000 and (number == 6 or number == 0 ):
                    joystick_data = 'right'
                elif value<0 and (number == 6 or number == 0 ):
                    joystick_data = 'left'
                else:
                    joystick_data = 'nil'   

                pub.publish(joystick_data)


if __name__ == '__main__':
    print('Main Function')
    find_joystick()
    try:
        buffer_test()
    except(KeyboardInterrupt, SystemExit):
        sys.exit(0)

