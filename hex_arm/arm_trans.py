#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import json
import numpy as np
from math import pi as PI
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
import threading
import time

from std_msgs.msg import UInt8MultiArray, String
from sensor_msgs.msg import JointState
from xpkg_arm_msgs.msg import XmsgArmJointParam, XmsgArmJointParamList

import os
import sys
script_path = os.path.abspath(os.path.dirname(__file__))
sys.path.append(script_path)

from ros_interface import DataInterface
import public_api_down_pb2
import public_api_types_pb2
import public_api_up_pb2

@dataclass
class JointParam:
    joint_name: str
    joint_limit: List[float] # [min_pos, max_pos, min_vel, max_vel, min_acc, max_acc]

class ArmDataInterface:
    def __init__(self):
        self.data_interface = DataInterface(node_name = "arm_trans")
        self.__lock = threading.Lock()

        self.__last_positions = None
        self.__last_velocities = None
        self.joints_json_path = self.data_interface.get_pkg_share_path("hex_arm") + "/config/joints.json"
        self.pose_init_json_path = self.data_interface.get_pkg_share_path("hex_arm") + "/config/init_pose.json"
        self.__motor_count: Optional[int] = None
        self.__api_initialized: bool = False
        self.__calibrated: bool = False
        self.pose_initialized: bool = False
        self.__motor_temperatures = None
        self.__pulse_per_rotation_list = None
        self.__current_positions = None
        self.__current_velocities = None
        self.__parking_stop_detail = public_api_types_pb2.ParkingStopDetail()
        self.__last_warning_time = time.perf_counter()

        self.joints: List[JointParam] = []
        joints_data = self.data_interface.load_from_json(self.joints_json_path)
        if joints_data is not None:
            if isinstance(joints_data["joints"], List):
                for joint_data in joints_data["joints"]:
                    self.joints.append(JointParam(joint_name=joint_data["joint_name"], joint_limit=joint_data["joint_limit"]))
                self.data_interface.logi(f"Load joint parameters from {self.joints_json_path}.")
                self.data_interface.logi(f"Joint parameters: {self.joints}.")
            else:
                self.data_interface.loge(f"Error: Please check your joints parameters in {self.joints_json_path}.")

        self.pose_init_params: List[float] = []
        self.step_limits: List[float] = []
        pose_init_data = self.data_interface.load_from_json(self.pose_init_json_path)
        if pose_init_data is not None:
            if isinstance(pose_init_data["init_pose"], List) and isinstance(pose_init_data["step_limits"], List):
                self.pose_init_params = pose_init_data["init_pose"]
                self.step_limits = pose_init_data["step_limits"]
                self.data_interface.logi(f"Load initial pose parameters from {self.pose_init_json_path}.")
                self.data_interface.logi(f"Initial pose parameters: {self.pose_init_params}.")
                self.data_interface.logi(f"Step limits: {self.step_limits}.")
            else:
                self.data_interface.loge(f"Error: Please check your initial pose parameters in {self.pose_init_json_path}.")

        self.__ws_down_pub = self.data_interface.create_publisher(UInt8MultiArray, "ws_down")
        self.__motor_status_pub = self.data_interface.create_publisher(JointState, "joint_states")
        self.__json_feedback_pub = self.data_interface.create_publisher(String, "json_feedback")
        self.data_interface.create_subscriber(UInt8MultiArray, "ws_up", self.__ws_up_callback)
        self.data_interface.create_subscriber(XmsgArmJointParamList, "joints_cmd", self.__joints_cmd_callback)
        self.data_interface.logi("ArmDataInterface initialized.")

    def __pub_ws_down(self, data: List[int]):
        msg = UInt8MultiArray()
        msg.data = data
        self.__ws_down_pub.publish(msg)

    def __pub_motor_status(self, pos: List[float], vel: List[float], eff: List[float]):
        length = len(pos)
        msg = JointState()
        msg.name = [f"joint{i}" for i in range(length)]
        msg.position = pos
        msg.velocity = vel
        msg.effort = eff
        self.__motor_status_pub.publish(msg)

    def __ws_up_callback(self, msg: UInt8MultiArray):
        api_up = public_api_up_pb2.APIUp()
        api_up.ParseFromString(bytes(msg.data))
        with self.__lock:
            self.__api_initialized = api_up.arm_status.api_control_initialized
            self.__calibrated = api_up.arm_status.calibrated
            self.__motor_count = len(api_up.arm_status.motor_status)
            self.__motor_temperatures = [motor.motor_temperature for motor in api_up.arm_status.motor_status]
            if api_up.arm_status.HasField('parking_stop_detail'):
                self.__parking_stop_detail = api_up.arm_status.parking_stop_detail
            else:
                self.__parking_stop_detail = public_api_types_pb2.ParkingStopDetail()
            # parse data
            pp, vv, tt, self.__pulse_per_rotation_list = self.parse_motor_data(api_up)
            self.__current_positions = pp
            self.__current_velocities = vv
        # publish data
        self.__pub_motor_status(pp, vv, tt)

    def __joints_cmd_callback(self, msg: XmsgArmJointParamList):
        if msg.joints is None or not self.pose_initialized:
            return
        with self.__lock:
            api_initialized = self.__api_initialized
            calibrated = self.__calibrated
            motor_count = self.__motor_count
            pulse_per_rotation_list = self.__pulse_per_rotation_list
            motor_temperatures = self.__motor_temperatures
            parking_stop_detail = self.__parking_stop_detail

        if parking_stop_detail.category == public_api_types_pb2.ParkingStopCategory.PscAPICommunicationTimeout:
            msg = public_api_down_pb2.APIDown()
            msg.arm_command.clear_parking_stop = True
            bin = msg.SerializeToString()
            self.__pub_ws_down(bin)

        if not api_initialized:
            api_init = public_api_down_pb2.APIDown()
            api_init.arm_command.api_control_initialize = True
            bin = api_init.SerializeToString()
            self.__pub_ws_down(bin)
        elif not calibrated:
            api_calibrate = public_api_down_pb2.APIDown()
            api_calibrate.arm_command.calibrate = True
            bin = api_calibrate.SerializeToString()
            self.__pub_ws_down(bin)
        else:
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
            
            positions = self.validate_joint_positions(positions)
            velocities = self.validate_joint_velocities(velocities)

            api_down = public_api_down_pb2.APIDown()
            joint_states: List[Dict] = []
            for i in range(length):
                extra_param = self.parse_extra_param(extra_params[i])
                if extra_param.get('braking_state', False):
                    joint_cmd = api_down.arm_command.motor_targets.targets.add()
                    joint_cmd.brake = True
                    joint_state = {
                        'control_mode': modes[i],
                        'braking_state': True,
                        'motor_temperature': motor_temperatures[i] if motor_temperatures is not None and i < len(motor_temperatures) else None
                    }
                else:
                    if modes[i] == "position_mode":
                        if pulse_per_rotation_list is not None and len(pulse_per_rotation_list) == length:
                            joint_cmd = api_down.arm_command.motor_targets.targets.add()
                            joint_cmd.position = int((positions[i] / (2 * PI)) * pulse_per_rotation_list[i] + 65535.0 / 2.0)
                        joint_state = {
                            'control_mode': "position_mode",
                            'braking_state': False,
                            'motor_temperature': motor_temperatures[i] if motor_temperatures is not None and i < len(motor_temperatures) else None
                        }
                    elif modes[i] == "velocity_mode":
                        joint_cmd = api_down.arm_command.motor_targets.targets.add()
                        joint_cmd.speed = velocities[i]
                        joint_state = {
                            'control_mode': "velocity_mode",
                            'braking_state': False,
                            'motor_temperature': motor_temperatures[i] if motor_temperatures is not None and i < len(motor_temperatures) else None
                        }
                    elif modes[i] == "torque_mode":
                        joint_cmd = api_down.arm_command.motor_targets.targets.add()
                        joint_cmd.torque = efforts[i]
                        joint_state = {
                            'control_mode': "torque_mode",
                            'braking_state': False,
                            'motor_temperature': motor_temperatures[i] if motor_temperatures is not None and i < len(motor_temperatures) else None
                        }
                    elif modes[i] == "mit_mode":
                        joint_cmd = api_down.arm_command.motor_targets.targets.add()
                        joint_cmd.mit_target.position = positions[i]
                        joint_cmd.mit_target.speed = velocities[i]
                        joint_cmd.mit_target.torque = efforts[i]
                        joint_cmd.mit_target.kp = extra_param.get('mit_kp', 0.0)
                        joint_cmd.mit_target.kd = extra_param.get('mit_kd', 0.0)
                        joint_state = {
                            'control_mode': {
                                "mit_mode": [
                                    extra_param.get('mit_kp', 0.0),
                                    extra_param.get('mit_kd', 0.0)
                                ]
                            },
                            'braking_state': False,
                            'motor_temperature': motor_temperatures[i] if motor_temperatures is not None and i < len(motor_temperatures) else None
                        }
                    else:
                        self.data_interface.logw(f"Unknown mode: {modes[i]}. Set speed to 0.0.")
                        joint_cmd = api_down.arm_command.motor_targets.targets.add()
                        joint_cmd.speed = 0.0
                        joint_state = {
                            'control_mode': "unknown",
                            'braking_state': False,
                            'motor_temperature': motor_temperatures[i] if motor_temperatures is not None and i < len(motor_temperatures) else None
                        }
                joint_states.append(joint_state)
            bin = api_down.SerializeToString()
            json_feedback = {
                'joint_states': joint_states
            }
            self.__pub_ws_down(bin)
            self.__json_feedback_pub.publish(String(data=json.dumps(json_feedback)))

    def parse_motor_data(self, api_up: public_api_up_pb2.APIUp) -> Tuple[List, List, List, List]:
        pp = []
        vv = []
        tt = []
        pulse_per_rotation_list = []
        if api_up.arm_status.IsInitialized():
            for motor_status in api_up.arm_status.motor_status:
                torque = motor_status.torque
                speed = motor_status.speed
                position = (motor_status.position - 65535.0 / 2.0 ) / motor_status.pulse_per_rotation * 2.0 * PI
                pulse_per_rotation_list.append(motor_status.pulse_per_rotation)
                pp.append(position)
                vv.append(speed)
                tt.append(torque)
        return pp, vv, tt , pulse_per_rotation_list
    
    def validate_joint_positions(self, positions: np.ndarray, dt: float = 0.01) -> np.ndarray:
        validated_positions = np.zeros_like(positions)
        with self.__lock:
            last_positions = np.array(self.__last_positions) if self.__last_positions is not None else np.array(self.__current_positions)

        for i, (position, joint) in enumerate(zip(positions, self.joints)):
            min_pos, max_pos = joint.joint_limit[0], joint.joint_limit[1]
            min_vel, max_vel = joint.joint_limit[2], joint.joint_limit[3]

            validated_position = np.clip(position, min_pos, max_pos)

            if last_positions is not None and i < len(last_positions):
                last_position = last_positions[i]

                current_velocity = (position - last_position) / dt

                if current_velocity < min_vel or current_velocity > max_vel:
                    if current_velocity > max_vel:
                        max_displacement = max_vel * dt
                        validated_position = last_position + max_displacement
                    elif current_velocity < min_vel:
                        min_displacement = min_vel * dt
                        validated_position = last_position + min_displacement

                    validated_position = np.clip(validated_position, min_pos, max_pos)

            validated_positions[i] = validated_position

        # Update recorded last position
        self.__last_positions = validated_positions.copy()

        return validated_positions
    
    def validate_joint_velocities(self, velocities: np.ndarray, dt: float = 0.01) -> np.ndarray:
        validated_velocities = np.zeros_like(velocities)
        with self.__lock:
            last_velocities = np.array(self.__last_velocities) if self.__last_velocities is not None else np.array(self.__current_velocities)

        for i, (velocity, joint) in enumerate(zip(velocities, self.joints)):
            min_vel, max_vel = joint.joint_limit[2], joint.joint_limit[3]
            min_acc, max_acc = joint.joint_limit[4], joint.joint_limit[5]

            validated_velocity = np.clip(velocity, min_vel, max_vel)

            if last_velocities is not None and i < len(last_velocities):
                last_velocity = last_velocities[i]

                current_acceleration = (validated_velocity - last_velocity) / dt

                if current_acceleration < min_acc or current_acceleration > max_acc:
                    if current_acceleration > max_acc:
                        max_velocity_change = max_acc * dt
                        validated_velocity = last_velocity + max_velocity_change
                        validated_velocity = min(validated_velocity, max_vel)
                    elif current_acceleration < min_acc:
                        min_velocity_change = min_acc * dt
                        validated_velocity = last_velocity + min_velocity_change
                        validated_velocity = max(validated_velocity, min_vel)

            validated_velocities[i] = validated_velocity

        # Update recorded last velocity
        self.__last_velocities = validated_velocities.copy()

        return validated_velocities
    
    def parse_extra_param(self, extra_param_str):
        try:
            if extra_param_str == "":
                return {}
            extra_param_dict = json.loads(extra_param_str)
            return extra_param_dict
        except json.JSONDecodeError:
            self.data_interface.loge(f"Error: {extra_param_str} is not a valid value.")
            return {}
        
    def check_parking_stop_detail(self):
        start_time = time.perf_counter()
        with self.__lock:
            parking_stop_detail = self.__parking_stop_detail
        if parking_stop_detail != public_api_types_pb2.ParkingStopDetail():
            if start_time - self.__last_warning_time > 1.0:
                self.data_interface.loge(f"emergency stop: {parking_stop_detail}")
                self.__last_warning_time = start_time
        
    def init_pose(self):
        self.data_interface.create_timer(0.01, self.__init_pose_callback)
    def __init_pose_callback(self):
        if self.pose_initialized:
            self.data_interface.cancel_timer()
            return
        
        with self.__lock:
            motor_count = self.__motor_count
            current_positions = self.__current_positions
            pulse_per_rotation_list = self.__pulse_per_rotation_list
            api_initialized = self.__api_initialized
            calibrated = self.__calibrated
            parking_stop_detail = self.__parking_stop_detail

        if motor_count is not None and current_positions is not None and pulse_per_rotation_list is not None:
            self.check_parking_stop_detail()
            if parking_stop_detail.category == public_api_types_pb2.ParkingStopCategory.PscAPICommunicationTimeout:
                msg = public_api_down_pb2.APIDown()
                msg.arm_command.clear_parking_stop = True
                bin = msg.SerializeToString()
                self.__pub_ws_down(bin)
            if not api_initialized:
                api_init = public_api_down_pb2.APIDown()
                api_init.arm_command.api_control_initialize = True
                bin = api_init.SerializeToString()
                self.__pub_ws_down(bin)
            elif not calibrated:
                api_calibrate = public_api_down_pb2.APIDown()
                api_calibrate.arm_command.calibrate = True
                bin = api_calibrate.SerializeToString()
                self.__pub_ws_down(bin)
            else:
                api_down = public_api_down_pb2.APIDown()
                for i in range(motor_count):
                    error = self.pose_init_params[i] - current_positions[i]
                    error = np.clip(error, -self.step_limits[i], self.step_limits[i])
                    target_position = current_positions[i] + error
                    api_down.arm_command.motor_targets.targets.add().position = int((target_position / (2 * PI)) * pulse_per_rotation_list[i] + 65535.0 / 2.0)
                bin = api_down.SerializeToString()
                self.__pub_ws_down(list(bin))
            if all(abs(self.pose_init_params[i] - current_positions[i]) < 0.01 for i in range(motor_count)):
                self.pose_initialized = True
                self.data_interface.logi("Initial pose reached.")
                self.data_interface.cancel_timer()
        # self.debug()

    def debug(self):
        # pass
        with self.__lock:
            self.data_interface.logi(f"Current positions: {self.__current_positions}")
            # self.data_interface.logi(f"Last positions: {self.__last_positions}")
            # self.data_interface.logi(f"Last velocities: {self.__last_velocities}")
            # self.data_interface.logi(f"Motor temperatures: {self.__motor_temperatures}")
            # self.data_interface.logi(f"Pulse per rotation list: {self.__pulse_per_rotation_list}")
            # self.data_interface.logi(f"API initialized: {self.__api_initialized}")
            # self.data_interface.logi(f"Pose initialized: {self.pose_initialized}")
            # self.data_interface.logi(f"Calibrated: {self.__calibrated}")
            # self.data_interface.logi(f"motor_count: {self.__motor_count}")

def main():
    arm = ArmDataInterface()
    try:
        arm.init_pose()
        while arm.data_interface.ok():
            arm.check_parking_stop_detail()
            # arm.debug()
            arm.data_interface.sleep()
    except KeyboardInterrupt:
        arm.data_interface.logi("Received Ctrl-C.")
    finally:
        arm.data_interface.shutdown()

if __name__ == '__main__':
    main()