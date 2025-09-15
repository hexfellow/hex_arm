#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import threading

import rclpy
import rclpy.node
from ament_index_python.packages import get_package_share_directory

from .interface_base import InterfaceBase

class DataInterface(InterfaceBase):
    def _init_node(self):
        # ros node
        rclpy.init()
        self.__node = rclpy.node.Node(self._node_name)
        self.__logger = self.__node.get_logger()
        self.__rate = self.__node.create_rate(300.0)

        # spin thread
        self.__spin_thread = threading.Thread(target=self.__spin)
        self.__spin_thread.start()

    def create_publisher(self, msg_type, topic: str, queue_size: int = 10):
        return self.__node.create_publisher(msg_type, topic, queue_size)

    def create_subscriber(self, msg_type, topic: str, callback, queue_size: int = 10):
        self.__node.create_subscription(msg_type, topic, callback, queue_size)
    
    def set_parameter(self, name: str, value):
        self.__node.declare_parameter(name, value)

    def get_parameter(self, name: str):
        return self.__node.get_parameter(name).value
    
    def __spin(self):
        rclpy.spin(self.__node)

    def ok(self):
        return rclpy.ok()

    def shutdown(self):
        self.__node.destroy_node()
        rclpy.shutdown()
        self.__spin_thread.join()

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
