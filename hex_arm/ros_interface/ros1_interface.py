#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import rospkg

from .interface_base import InterfaceBase

class DataInterface(InterfaceBase):
    def _init_node(self):
        # ros node
        rospy.init_node(self._node_name, anonymous=True)
        self.__rate = rospy.Rate(300.0)
        self.timer = None

    def create_publisher(self, msg_type, topic: str, queue_size: int = 10):
        return rospy.Publisher(topic, msg_type, queue_size=queue_size)
    
    def create_subscriber(self, msg_type, topic: str, callback, queue_size: int = 10):
        rospy.Subscriber(topic, msg_type, callback, queue_size=queue_size)

    def crearte_timer(self, interval_sec: float, callback):
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
        rospy.signal_shutdown("Shutdown")

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