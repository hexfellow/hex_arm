#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import os

from .interface_base import GripperConfig as GripperConfig
from .interface_base import ArmConfig as ArmConfig

ROS_VERSION = os.environ.get('ROS_VERSION')
if ROS_VERSION == '1':
    from .ros1_interface import DataInterface as DataInterface
elif ROS_VERSION == '2':
    from .ros2_interface import DataInterface as DataInterface
else:
    raise ValueError("ROS_VERSION is not set")


__all__ = [
    "DataInterface",
]