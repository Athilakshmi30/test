#!/usr/bin/env python

import json
import websocket
"""
Class to send and receive ROS messages to a ROS master that has a rosbridge websocket.
Author: Dmitry Devitt
Rework: Sammy Pfeiffer (https://gist.github.com/awesomebytes/67ef305a2d9072ac64b786af292f5907)
"""


class WebsocketROSClient:
    def service(websocket_ip, name,arg1,arg2,port=9090):
       
        print("Connecting to websocket: {}:{}".format(websocket_ip, port))
        ws = websocket.create_connection('ws://' + websocket_ip + ':' + str(port))
        msg = {
            'op': 'call_service',
            'service': name,
            'args': {"username":arg1,"password":arg2}
        }
        json_msg = json.dumps(msg)
        ws.send(json_msg)
