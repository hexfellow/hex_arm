#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import threading
import rclpy
import rclpy.node
import json
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import UInt8MultiArray, String
from xpkg_arm_msgs.msg import XmsgArmJointParamList
from sensor_msgs.msg import JointState
from hex_device import public_api_up_pb2, public_api_down_pb2, public_api_types_pb2
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
        joint_config_path_value = self.__node.get_parameter('joint_config_path').value
        self.joint_config_path = joint_config_path_value if joint_config_path_value else None
        
        self.__node.declare_parameter('init_pose_path', '')
        init_pose_path_value = self.__node.get_parameter('init_pose_path').value
        self.init_pose_path = init_pose_path_value if init_pose_path_value else None
        
        self._is_init = True
        
        self.__node.declare_parameter('gripper_type', 0)
        self.gripper_type = self.__node.get_parameter('gripper_type').value
        
        self.__node.declare_parameter('arm_series', 0)
        self.arm_series = self.__node.get_parameter('arm_series').value
        
        ### publisher
        self.__ws_down_pub = self.__node.create_publisher(UInt8MultiArray, 'ws_down', 10)
        self.__motor_status_pub = self.__node.create_publisher(JointState, '/xtopic_arm/joint_states', 10)
        self.__json_feedback_pub = self.__node.create_publisher(String, '/xtopic_arm/json_feedback', 10)

        ### subscriber
        self.__ws_up_sub = self.__node.create_subscription(
            UInt8MultiArray,
            'ws_up',
            self._ws_up_callback,
            10,
        )
        self.__joints_cmd_sub = self.__node.create_subscription(
            XmsgArmJointParamList,
            '/xtopic_arm/joints_cmd',
            self._joints_cmd_callback,
            10,
        )

        # gripper subscriber and publisher
        if self.gripper_type is not None:
            self.__gripper_status_pub = self.__node.create_publisher(JointState, '/xtopic_arm/gripper_states', 10)
            self.__gripper_cmd_sub = self.__node.create_subscription(
                XmsgArmJointParamList,
                '/xtopic_arm/gripper_cmd',
                self._gripper_cmd_callback,
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
        try:
            rclpy.spin(self.__node)
        except Exception:
            # Ignore exceptions during shutdown (ExternalShutdownException)
            pass

    def ok(self):
        return rclpy.ok()

    def shutdown(self):
        try:
            # First shutdown rclpy to stop spin thread
            if rclpy.ok():
                rclpy.shutdown()
            # Wait for spin thread to finish
            if self.__spin_thread and self.__spin_thread.is_alive():
                self.__spin_thread.join(timeout=1.0)
            # Then destroy node
            try:
                self.__node.destroy_node()
            except Exception:
                pass
        except Exception:
            # Ignore any shutdown errors
            pass

    def sleep(self):
        try:
            self._rate.sleep()
        except Exception:
            pass

    def logd(self, msg, *args, **kwargs):
        try:
            self.__logger.debug(msg, *args, **kwargs)
        except Exception:
            pass

    def logi(self, msg, *args, **kwargs):
        try:
            self.__logger.info(msg, *args, **kwargs)
        except Exception:
            pass

    def logw(self, msg, *args, **kwargs):
        try:
            self.__logger.warning(msg, *args, **kwargs)
        except Exception:
            pass

    def loge(self, msg, *args, **kwargs):
        try:
            self.__logger.error(msg, *args, **kwargs)
        except Exception:
            pass

    def logf(self, msg, *args, **kwargs):
        try:
            self.__logger.fatal(msg, *args, **kwargs)
        except Exception:
            pass  

    def get_pkg_share_path(self, package_name: str) -> str:
        try:  
            return get_package_share_directory(package_name)
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