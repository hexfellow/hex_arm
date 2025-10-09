#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import json
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Dict, Optional
from hex_device.generated import public_api_types_pb2


class GripperConfig:
    """Gripper configuration class for mapping gripper type to motor count."""
    def __init__(self):
        self.gripper_motor_map = {
            public_api_types_pb2.HandType.HtInvalid: 0, 
            public_api_types_pb2.HandType.HtGp100: 1}

class ArmConfig:
    """Gripper configuration class for mapping gripper type to motor count."""
    def __init__(self):
        self.arm_motor_map = {
            public_api_types_pb2.RobotType.RtArmArcherD6Y: 6, 
            public_api_types_pb2.RobotType.RtArmSaberD6X: 6,
            public_api_types_pb2.RobotType.RtArmSaberD7X: 7,
            public_api_types_pb2.RobotType.RtArmSaber750d3Lr3DmDriver: 6,
            public_api_types_pb2.RobotType.RtArmSaber750d4Lr3DmDriver: 7,
            public_api_types_pb2.RobotType.RtArmSaber750h3Lr3DmDriver: 6,
            public_api_types_pb2.RobotType.RtArmSaber750h4Lr3DmDriver: 7,
            public_api_types_pb2.RobotType.RtArmSaberD6X: 6,
            public_api_types_pb2.RobotType.RtArmSaberD7X: 7,
            public_api_types_pb2.RobotType.RtArmArcherD6Y: 6,
        }

class InterfaceBase(ABC):
    def __init__(self, name: str):
        self._name = name
        print(f"#### InterfaceBase init: {self._name} ####")
    
    def create_timer(self, interval_sec: float, callback):
        raise NotImplementedError("InterfaceBase.create_timer")
    
    def cancel_timer(self):
        raise NotImplementedError("InterfaceBase.cancel_timer")
    
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

    @abstractmethod
    def __pub_ws_down(self, data):
        raise NotImplementedError("__pub_ws_down")

    @abstractmethod
    def __pub_motor_status(self, pos, vel, eff):
        raise NotImplementedError("__pub_motor_status")

    @abstractmethod
    def __ws_up_callback(self, msg):
        raise NotImplementedError("__ws_up_callback")

    @abstractmethod
    def __joints_cmd_callback(self, msg):
        raise NotImplementedError("__joints_cmd_callback")

    @abstractmethod
    def __gripper_cmd_callback(self, msg):
        raise NotImplementedError("__gripper_cmd_callback")
    
    def get_config_from_json(self, json_path: str) -> Optional[Dict]:
        try:
            with open(json_path, "r") as f:
                json_data = json.load(f)
                if json_data is not None:
                    if "joints" in json_data and isinstance(json_data["joints"], list):
                        self.logi(f"Load joint parameters from {json_path}.")
                        self.logi(f"Joint parameters: {json_data['joints']}.")
                        return json_data
                    else:
                        self.loge(f"Error: Have not found joints in {json_path}.")
                else:
                    self.loge(f"Error: JSON data is None in {json_path}.")
        except FileNotFoundError:
            self.loge(f"Error: File not found: {json_path}.")
        except json.JSONDecodeError:
            self.loge(f"Error: Failed to decode JSON from {json_path}.")
        except Exception as e:
            self.loge(f"Error: An unexpected error occurred while loading {json_path}: {e}.")
        
        return None

    def get_list_from_json(self, json_path: str) -> Optional[List[float]]:
        try:
            with open(json_path, "r") as f:
                json_data = json.load(f)
                if json_data is not None:
                    if isinstance(json_data, list):
                        self.logi(f"Load initial pose from {json_path}.")
                        self.logi(f"Initial pose: {json_data}.")
                        return json_data
                    else:
                        self.loge(f"Error: Expected a list/array in {json_path}, got {type(json_data).__name__}.")
        except FileNotFoundError:
            self.loge(f"Error: File not found: {json_path}.")
        except json.JSONDecodeError:
            self.loge(f"Error: Failed to decode JSON from {json_path}.")
        except Exception as e:
            self.loge(f"Error: An unexpected error occurred while loading {json_path}: {e}.")
        return None

    def parse_extra_param(self, extra_param_str):
        try:
            if extra_param_str == "":
                return {}
            extra_param_dict = json.loads(extra_param_str)
            return extra_param_dict
        except json.JSONDecodeError:
            self.loge(f"Error: {extra_param_str} is not a valid value.")
            return {}