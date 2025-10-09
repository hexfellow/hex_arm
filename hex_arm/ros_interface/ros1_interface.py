#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import rospkg
from typing import List, Dict, Tuple, Optional

from .interface_base import InterfaceBase
from std_msgs.msg import UInt8MultiArray, JointState
from xpkg_arm_msgs.msg import XmsgArmJointParamList, String
from hex_device.generated import public_api_up_pb2, public_api_down_pb2, public_api_types_pb2
from .interface_base import JointParam

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
            self.__ws_up_callback,
        )
        self.__joints_cmd_sub = rospy.Subscriber(
            '/xtopic_arm/joints_cmd',
            XmsgArmJointParamList,
            self.__joints_cmd_callback,
        )

        # gripper subscriber and publisher
        if self._gripper_type is not None:
            self.__gripper_status_pub = rospy.Publisher('/xtopic_arm/gripper_states', JointState, queue_size=10)
            self.__gripper_cmd_sub = rospy.Subscriber(
                '/xtopic_arm/gripper_cmd',
                XmsgArmJointParamList,
                self.__gripper_cmd_callback,
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
        self.__rate.sleep()

    def logd(self, msg, *args, **kwargs):
        rospy.logdebug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        rospy.loginfo(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        rospy.logwarn(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        rospy.logerr(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        rospy.logfatal(msg, *args, **kwargs)

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

    def _pub_ws_down(self, data: public_api_down_pb2.APIDown):
        '''
        data: Protobuf data of APIDown message
        '''
        msg = UInt8MultiArray()
        msg.data = data.SerializeToString()
        self.__ws_down_pub.publish(msg)

    def __pub_motor_status(self, pos, vel, eff):
        msg = JointState()
        length = max(len(pos), len(vel), len(eff))
        msg.name = [f"joint{i}" for i in range(length)]
        msg.position = pos
        msg.velocity = vel
        msg.effort = eff
        self.__motor_status_pub.publish(msg)

    def __ws_up_callback(self, msg: UInt8MultiArray):
        api_up = public_api_up_pb2.APIUp()
        api_up.ParseFromString(bytes(msg.data))
        # TODO: 解析并更新数据

    def __joints_cmd_callback(self, msg: XmsgArmJointParamList):
        if msg.joints is None or not self.pose_initialized:
            return
        # TODO：解析并更新指令

        if hasattr(msg, 'joints'):
            length = len(msg.joints)
            if length != motor_count:
                self.data_interface.logw(f"XmsgArmJointParamList message length {length} not match motor count {motor_count}.")
                return
        else:
            self.data_interface.logw(f"XmsgArmJointParamList message has no joints.")
            return
        modes = np.array([joint.mode for joint in msg.joints])
        positions = np.array([joint.position for joint in msg.joints])
        velocities = np.array([joint.velocity for joint in msg.joints])
        efforts = np.array([joint.effort for joint in msg.joints])
        extra_params = [joint.extra_param for joint in msg.joints]
        
        api_down = public_api_down_pb2.APIDown()
        bin = api_down.SerializeToString()
        json_feedback = {
            'joint_states': joint_states
        }
        self.__pub_ws_down(bin)
        self.__json_feedback_pub.publish(String(data=json.dumps(json_feedback)))

    def __gripper_cmd_callback(self, msg: XmsgArmJointParamList):
        if msg.joints is None or not self.pose_initialized:
            return
        # TODO：解析并更新指令