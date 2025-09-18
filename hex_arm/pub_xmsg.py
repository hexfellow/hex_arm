#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import sys
import tty
import termios
from .ros_interface import DataInterface
from xpkg_arm_msgs.msg import XmsgArmJointParam, XmsgArmJointParamList

class XmsgInterface:
    def __init__(self, node_name: str):
        self.data_interface = DataInterface(node_name)
        self.__joints_cmd_pub = self.data_interface.create_publisher(XmsgArmJointParamList, "/joints_cmd")

    def pub_joints_cmd(self):
        msg = XmsgArmJointParamList(
            joints=[
                XmsgArmJointParam(mode="position_mode", position=0.0, velocity=0.0, effort=0.0, extra_param="{\"braking_state\": true}"),
                XmsgArmJointParam(mode="position_mode", position=3.0, velocity=0.0, effort=0.0, extra_param=""),
                XmsgArmJointParam(mode="position_mode", position=3.0, velocity=0.0, effort=0.0, extra_param=""),
                XmsgArmJointParam(mode="position_mode", position=3.0, velocity=0.0, effort=0.0, extra_param=""),
                XmsgArmJointParam(mode="position_mode", position=3.0, velocity=0.0, effort=0.0, extra_param=""),
                XmsgArmJointParam(mode="position_mode", position=3.0, velocity=0.0, effort=0.0, extra_param=""),
            ]
        )

        if not hasattr(msg, 'joints'):
            self.data_interface.loge("Created message without 'joints' attribute")
            return
        if len(msg.joints) == 0:
            self.data_interface.loge("Created message with empty joints list")
            return
            
        self.data_interface.logi(f"Publishing message with {len(msg.joints)} joints")
        self.__joints_cmd_pub.publish(msg)

def main():
    xmsg_interface = XmsgInterface("xmsg_pub")
    try:
        while xmsg_interface.data_interface.ok():
            xmsg_interface.pub_joints_cmd()
            xmsg_interface.data_interface.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        xmsg_interface.data_interface.shutdown()

if __name__ == "__main__":
    main()