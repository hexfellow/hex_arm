#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import json
from abc import ABC, abstractmethod

class InterfaceBase(ABC):
    def __init__(self, node_name: str):
        self._node_name = node_name
        self._init_node()

    @abstractmethod
    def _init_node(self):
        raise NotImplementedError("InterfaceBase._init_node")

    def __del__(self):
        self.shutdown()

    @abstractmethod
    def create_publisher(self, msg_type, topic: str, queue_size: int = 10):
        raise NotImplementedError("InterfaceBase.create_publisher")
    
    @abstractmethod
    def create_subscriber(self, msg_type, topic: str, callback, queue_size: int = 10):
        raise NotImplementedError("InterfaceBase.create_subscriber")
    
    @abstractmethod
    def set_parameter(self, name: str, value):
        raise NotImplementedError("InterfaceBase.set_parameter")
    
    @abstractmethod
    def get_parameter(self, name: str):
        raise NotImplementedError("InterfaceBase.get_parameter")
    
    @abstractmethod
    def ok(self) -> bool:
        raise NotImplementedError("InterfaceBase.ok")

    @abstractmethod
    def shutdown(self):
        raise NotImplementedError("InterfaceBase.shutdown")

    @abstractmethod
    def sleep(self):
        raise NotImplementedError("InterfaceBase.sleep")

    # logging
    @abstractmethod
    def logd(self, msg, *args, **kwargs):
        raise NotImplementedError("logd")

    @abstractmethod
    def logi(self, msg, *args, **kwargs):
        raise NotImplementedError("logi")

    @abstractmethod
    def logw(self, msg, *args, **kwargs):
        raise NotImplementedError("logw")

    @abstractmethod
    def loge(self, msg, *args, **kwargs):
        raise NotImplementedError("loge")

    @abstractmethod
    def logf(self, msg, *args, **kwargs):
        raise NotImplementedError("logf")
    
    @abstractmethod
    def get_pkg_share_path(self, package_name: str) -> str:
        raise NotImplementedError("get_pkg_share_path")
    
    def load_from_json(self, json_path: str):
        try:
            with open(json_path, "r") as f:
                json_data = json.load(f)
                return json_data
        except FileNotFoundError:
            self.loge(f"Error: File not found: {json_path}.")
        except json.JSONDecodeError:
            self.loge(f"Error: Failed to decode JSON from {json_path}.")
        except Exception as e:
            self.loge(f"Error: An unexpected error occurred while loading {json_path}: {e}.")