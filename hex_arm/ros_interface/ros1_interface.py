#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import rospkg
import json
from typing import List, Dict, Tuple, Optional

from .interface_base import InterfaceBase
from std_msgs.msg import UInt8MultiArray, String
from xpkg_arm_msgs.msg import XmsgArmJointParamList
from sensor_msgs.msg import JointState
from hex_device import public_api_up_pb2, public_api_down_pb2, public_api_types_pb2

class DataInterface(InterfaceBase):
    def __init__(self, name: str):
        super(DataInterface, self).__init__(name=name)

        ### ros node
        rospy.init_node(self._name, anonymous=True)
        # init rate
        self.ros_rate = rospy.get_param('~rate_ros', 300.0)
        self._rate = rospy.Rate(self.ros_rate)

        # load parameters
        self.joint_config_path = rospy.get_param('~joint_config_path', None)
        self.init_pose_path = rospy.get_param('~init_pose_path', None)
        self._is_init = True
        self.gripper_type = rospy.get_param('~gripper_type', None)
        self.arm_series = rospy.get_param('~arm_series', None)

        ### publisher
        self.__ws_down_pub = rospy.Publisher('ws_down', UInt8MultiArray, queue_size=10)
        self.__motor_status_pub = rospy.Publisher('/xtopic_arm/joint_states', JointState, queue_size=10)
        self.__json_feedback_pub = rospy.Publisher('/xtopic_arm/json_feedback', String, queue_size=10)

        ### subscriber
        self.__ws_up_sub = rospy.Subscriber(
            'ws_up',
            UInt8MultiArray,
            self._ws_up_callback,
        )
        self.__joints_cmd_sub = rospy.Subscriber(
            '/xtopic_arm/joints_cmd',
            XmsgArmJointParamList,
            self._joints_cmd_callback,
        )

        # gripper subscriber and publisher
        if self.gripper_type is not None and self.gripper_type != 0:
            self.__gripper_status_pub = rospy.Publisher('/xtopic_arm/gripper_states', JointState, queue_size=10)
            self.__gripper_cmd_sub = rospy.Subscriber(
                '/xtopic_arm/gripper_cmd',
                XmsgArmJointParamList,
                self._gripper_cmd_callback,
            )

    def create_timer(self, interval_sec: float, callback):
        self.timer = rospy.Timer(rospy.Duration(interval_sec), callback)
    
    def cancel_timer(self):
        if self.timer is not None:
            self.timer.shutdown()
            self.timer = None

    def set_parameter(self, name: str, value):
        rospy.set_param(name, value)

    def get_parameter(self, name: str):
        return rospy.get_param(name)
    
    def ok(self):
        return not rospy.is_shutdown()

    def shutdown(self):
        rospy.signal_shutdown("Normal shutdown")

    def sleep(self):
        try:
            self._rate.sleep()
        except Exception:
            pass

    def logd(self, msg, *args, **kwargs):
        try:
            rospy.logdebug(msg, *args, **kwargs)
        except Exception:
            pass

    def logi(self, msg, *args, **kwargs):
        try:
            rospy.loginfo(msg, *args, **kwargs)
        except Exception:
            pass

    def logw(self, msg, *args, **kwargs):
        try:
            rospy.logwarn(msg, *args, **kwargs)
        except Exception:
            pass

    def loge(self, msg, *args, **kwargs):
        try:
            rospy.logerr(msg, *args, **kwargs)
        except Exception:
            pass

    def logf(self, msg, *args, **kwargs):
        try:
            rospy.logfatal(msg, *args, **kwargs)
        except Exception:
            pass

    def get_pkg_share_path(self, package_name: str) -> str:
        try:    
            rospack = rospkg.RosPack()
            return rospack.get_path(package_name)
        except rospkg.ResourceNotFound:
            self.loge(f"Package '{package_name}' not found.")
            return ""
        except Exception as e:
            self.loge(f"An error occurred while getting the path for package '{package_name}': {e}")
            return ""

    async def _pub_ws_down(self, data: public_api_down_pb2.APIDown):
        '''
        data: Protobuf data of APIDown message
        '''
        try:
            msg = UInt8MultiArray()
            msg.data = data.SerializeToString()
            self.__ws_down_pub.publish(msg)
        except Exception as e:
            # Silently ignore publish errors during shutdown
            pass

    def pub_motor_status(self, pos, vel, eff):
        try:
            msg = JointState()
            length = max(len(pos), len(vel), len(eff))
            msg.name = [f"joint{i}" for i in range(length)]
            msg.position = pos
            msg.velocity = vel
            msg.effort = eff
            self.__motor_status_pub.publish(msg)
        except Exception:
            # Silently ignore publish errors during shutdown
            pass

    def _ws_up_callback(self, msg: UInt8MultiArray):
        api_up = public_api_up_pb2.APIUp()
        api_up.ParseFromString(bytes(msg.data))
        robot_type = api_up.robot_type

        if self.arm is not None:
            if robot_type == self.arm.robot_type:
                self.arm._update(api_up)

        if self.hands is not None:
            if hasattr(api_up, 'hand_status') and api_up.HasField('hand_status'):
                self.hands._update_optional_data('hand_status', api_up.hand_status)

    def _joints_cmd_callback(self, msg: XmsgArmJointParamList):
        if self._is_init is False:
            self._process_motor_command(msg, self.arm, 'arm')

    def _gripper_cmd_callback(self, msg: XmsgArmJointParamList):
        self._process_motor_command(msg, self.hands, 'gripper')