#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from math import pi as PI
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional

import os
import sys
script_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(script_path)

from ros_interface import DataInterface
from hex_device import Hands, ArmArcher
from ros_interface import GripperConfig, ArmConfig

@dataclass
class JointParam:
    joint_name: str
    joint_limit: List[float] # [min_pos, max_pos, min_vel, max_vel, min_acc, max_acc]

class HexArmApi:
    def __init__(self):
        self.data_interface = DataInterface(node_name = "xnode_arm")

        # init arm
        arm_config = ArmConfig()
        self.arm = ArmArcher(
            self.data_interface.arm_series,
            arm_config.arm_motor_map[self.data_interface.arm_series],
            control_hz = self.data_interface.ros_rate,
            send_message_callback = self.data_interface._pub_ws_down,
            )

        ## reload joints config
        if self.data_interface.joint_config_path is None:
            self.data_interface.logi("Joint config path is not set, using default config.")
        else:
            self.joints = self.data_interface.get_config_from_json(self.data_interface.joint_config_path)

        ## get init pose
        if self.data_interface.init_pose_path is not None:
            self.init_pose = self.data_interface.get_list_from_json(self.data_interface.init_pose_path)

        # init hands
        if self.data_interface.gripper_type is None:
            gripper_config = GripperConfig()
            self.hands = Hands(
                self.data_interface.gripper_type, 
                gripper_config.gripper_motor_map[self.data_interface.gripper_type], 
                control_hz = self.data_interface.ros_rate,
                send_message_callback = self.data_interface._pub_ws_down,
                )
        
def main():
    api = HexArmApi()
    
    try:
        while api.data_interface.ok():
            pass
    except KeyboardInterrupt:
        api.data_interface.logi("Received Ctrl-C.")
    finally:
        api.data_interface.shutdown()

if __name__ == '__main__':
    main()