#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import json
import numpy as np
from math import pi as PI
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
import threading
import time

from std_msgs.msg import UInt8MultiArray, String
from sensor_msgs.msg import JointState
from xpkg_arm_msgs.msg import XmsgArmJointParam, XmsgArmJointParamList

from .ros_interface import DataInterface

class Test:
    def __init__(self):
        self.data_interface = DataInterface(node_name = "test_node")
        self.data_interface.create_subscriber(XmsgArmJointParamList, "joints_cmd", self.__joints_cmd_callback)
        self.data_interface.create_timer(0.01, self.__callback)
        self.call_cnt = 0

    def __joints_cmd_callback(self, msg: XmsgArmJointParamList):
        self.data_interface.logi("joints_cmd_callback")

        length = len(msg.joints)

        modes = np.array([joint.mode for joint in msg.joints])
        positions = np.array([joint.position for joint in msg.joints])
        velocities = np.array([joint.velocity for joint in msg.joints])
        efforts = np.array([joint.effort for joint in msg.joints])
        extra_params = [joint.extra_param for joint in msg.joints]

        self.data_interface.logi(f"length: {length}")
        self.data_interface.logi(f"joints: {msg.joints}")
        self.data_interface.logi(f"modes: {modes}")
        self.data_interface.logi(f"positions: {positions}")
        self.data_interface.logi(f"velocities: {velocities}")
        self.data_interface.logi(f"efforts: {efforts}")
        self.data_interface.logi(f"extra_params: {extra_params}")

    def __callback(self):
        self.data_interface.logi("mother fuck")
        self.call_cnt += 1
        if self.call_cnt  == 10:
            self.data_interface.cancel_timer()

def main():
    test = Test()
    try:
        while test.data_interface.ok():
            
            test.data_interface.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        test.data_interface.shutdown()