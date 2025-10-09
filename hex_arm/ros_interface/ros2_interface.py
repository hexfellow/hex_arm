#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import threading
import rclpy
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import UInt8MultiArray, JointState
from xpkg_arm_msgs.msg import XmsgArmJointParamList, String
from hex_device.generated import public_api_up_pb2, public_api_down_pb2, public_api_types_pb2
from .interface_base import InterfaceBase

class DataInterface(InterfaceBase):
    def __init__(self, name: str):
        super(DataInterface, self).__init__(name=name)

        ### ros node
        rclpy.init()
        self.__node = rclpy.node.Node(name)
        self.__logger = self.__node.get_logger()
        # init rate
        self.__node.declare_parameter('rate_ros', 300.0)
        self.ros_rate = self.__node.get_parameter('rate_ros').value
        self._rate = self.__node.create_rate(self.ros_rate)

        # load parameters
        self.__node.declare_parameter('joint_config_path', '')
        self.joint_config_path = self.__node.get_parameter('joint_config_path').value or None
        self.__node.declare_parameter('init_pose_path', '')
        self.init_pose_path = self.__node.get_parameter('init_pose_path').value or None
        self.__node.declare_parameter('gripper_type', '')
        self.gripper_type = self.__node.get_parameter('gripper_type').value or None
        self.__node.declare_parameter('arm_series', '')
        self.arm_series = self.__node.get_parameter('arm_series').value or None
        
        ### publisher
        self.__ws_down_pub = self.__node.create_publisher(UInt8MultiArray, 'ws_down', queue_size=10)
        self.__motor_status_pub = self.__node.create_publisher(JointState, '/xtopic_arm/joint_states', queue_size=10)
        self.__json_feedback_pub = self.__node.create_publisher(String, '/xtopic_arm/json_feedback', queue_size=10)

        ### subscriber
        self.__ws_up_sub = self.__node.create_subscription(
            UInt8MultiArray,
            'ws_up',
            self.__ws_up_callback,
            10,
        )
        self.__joints_cmd_sub = self.__node.create_subscription(
            XmsgArmJointParamList,
            '/xtopic_arm/joints_cmd',
            self.__joints_cmd_callback,
            10,
        )

        # gripper subscriber and publisher
        if self._gripper_type is not None:
            self.__gripper_status_pub = self.__node.create_publisher(JointState, '/xtopic_arm/gripper_states', queue_size=10)
            self.__gripper_cmd_sub = self.__node.create_subscription(
                XmsgArmJointParamList,
                '/xtopic_arm/gripper_cmd',
                self.__gripper_cmd_callback,
                10,
            )

        ### spin thread
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

    def create_timer(self, interval_sec: float, callback):
        self.timer = self.__node.create_timer(interval_sec, callback)
        return self.timer
    
    def cancel_timer(self):
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
            
    def set_parameter(self, name: str, value):
        self.__node.declare_parameter(name, value)

    def get_parameter(self, name: str):
        return self.__node.get_parameter(name).value
    
    def __spin(self):
        rclpy.spin(self.__node)

    def ok(self):
        return rclpy.ok()

    def shutdown(self):
        self.__spin_thread.join()
        self.__node.destroy_node()
        if self.ok():
            rclpy.shutdown()

    def sleep(self):
        self.__rate.sleep()

    def logd(self, msg, *args, **kwargs):
        self.__logger.debug(msg, *args, **kwargs)

    def logi(self, msg, *args, **kwargs):
        self.__logger.info(msg, *args, **kwargs)

    def logw(self, msg, *args, **kwargs):
        self.__logger.warning(msg, *args, **kwargs)

    def loge(self, msg, *args, **kwargs):
        self.__logger.error(msg, *args, **kwargs)

    def logf(self, msg, *args, **kwargs):
        self.__logger.fatal(msg, *args, **kwargs)  

    def get_pkg_share_path(self, package_name: str) -> str:
        try:  
            return get_package_share_directory(package_name)
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