#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import json
import numpy as np
from math import pi as PI
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional

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
        self.__ws_down_pub = self.data_interface.create_publisher(UInt8MultiArray, "ws_down")
        self.__motor_status_pub = self.data_interface.create_publisher(JointState, "joint_states")
        self.__json_feedback_pub = self.data_interface.create_publisher(String, "json_feedback")
        self.data_interface.create_subscriber(UInt8MultiArray, "ws_up", self.__ws_up_callback)
        self.data_interface.create_subscriber(XmsgArmJointParamList, "joints_cmd", self.__joints_cmd_callback)

        self.__last_positions = None
        self.__last_velocities = None
        self.joints_json_path = self.data_interface.get_pkg_share_path("hex_arm") + "/config/joints.json"
        self.pose_init_json_path = self.data_interface.get_pkg_share_path("hex_arm") + "/config/init_pose.json"
        self.__motor_count: Optional[int] = None
        self.__api_initialized: bool = False
        self.pose_initialized: bool = False
        self.__motor_temperatures = None
        self.__pulse_per_rotation_list = None
        self.__current_positions = None

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
        self.__api_initialized = api_up.arm_status.api_control_initialized
        self.__motor_count = len(api_up.arm_status.motor_states)
        self.__motor_temperatures = [motor.motor_temperature for motor in api_up.arm_status.motor_states]
        # parse data
        pp, vv, tt, self.__pulse_per_rotation_list= self.prase_motor_data(api_up)
        self.__current_positions = pp
        # publish data
        self.__pub_motor_status(pp, vv, tt)

    def __joints_cmd_callback(self, msg: XmsgArmJointParamList):
        if msg.joints is not None and self.pose_initialized:
            if not self.__api_initialized:
                api_init = public_api_down_pb2.APIDown()
                api_init.arm_command.api_control_initialize = True
                bin = api_init.SerializeToString()
                self.__pub_ws_down(bin)

            length = len(msg.joints)
            if length != self.__motor_count:
                self.data_interface.logw(f"XmsgArmJointParamList message length {length} not match motor count {self.__motor_count}.")
                return
            
            modes = []
            positions = []
            velocities = []
            efforts = []
            extra_params = []
            for joint in msg.joints:
                modes.append(joint.mode)
                positions.append(joint.position)
                velocities.append(joint.velocity)
                efforts.append(joint.effort)
                extra_params.append(joint.extra_param)
            positions = self.validate_joint_positions(list(positions))
            velocities = self.validate_joint_velocities(list(velocities))

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
                        'motor_temperature': self.__motor_temperatures[i] if self.__motor_temperatures is not None and i < len(self.__motor_temperatures) else None
                    }
                else:
                    if modes[i] == "position_mode":
                        if self.__pulse_per_rotation_list is not None and len(self.__pulse_per_rotation_list) == length:
                            joint_cmd = api_down.arm_command.motor_targets.targets.add()
                            joint_cmd.position = int((positions[i] / (2 * PI)) * self.__pulse_per_rotation_list[i] + 65535.0 / 2.0)
                    elif modes[i] == "velocity_mode":
                        joint_cmd = api_down.arm_command.motor_targets.targets.add()
                        joint_cmd.speed = velocities[i]
                    elif modes[i] == "torque_mode":
                        joint_cmd = api_down.arm_command.motor_targets.targets.add()
                        joint_cmd.torque = efforts[i]
                    elif modes[i] == "mit_mode":
                        joint_cmd = api_down.arm_command.motor_targets.targets.add()
                        joint_cmd.mit_target.position = positions[i]
                        joint_cmd.mit_target.speed = velocities[i]
                        joint_cmd.mit_target.torque = efforts[i]
                        joint_cmd.mit_target.kp = extra_param.get('mit_kp', 0.0)
                        joint_cmd.mit_target.kd = extra_param.get('mit_kd', 0.0)
                    else:
                        self.data_interface.logw(f"Unknown mode: {modes[i]}. Set speed to 0.0.")
                        joint_cmd = api_down.arm_command.motor_targets.targets.add()
                        joint_cmd.speed = 0.0
                    joint_state = {
                        'control_mode': modes[i],
                        'braking_state': False,
                        'motor_temperature': self.__motor_temperatures[i] if self.__motor_temperatures is not None and i < len(self.__motor_temperatures) else None
                    }
                joint_states.append(joint_state)
            bin = api_down.SerializeToString()
            json_feedback = {
                'joint_states': joint_states
            }
            self.__pub_ws_down(list(bin))
            self.__json_feedback_pub.publish(String(data=json.dumps(json_feedback)))

    def prase_motor_data(self, api_up: public_api_up_pb2.APIUp) -> Tuple[List, List, List, List]:
        pp = []
        vv = []
        tt = []
        pulse_per_rotation_list = []
        if api_up.arm_status.IsInitialized():
            for motor_status in api_up.arm_status.motor_status:
                torque = motor_status.torque
                speed = motor_status.speed
                position = (motor_status.position % motor_status.pulse_per_rotation) / motor_status.pulse_per_rotation * (2.0 * PI) - PI # radian
                pulse_per_rotation_list.append(motor_status.pulse_per_rotation)
                pp.append(position)
                vv.append(speed)
                tt.append(torque)
        return pp, vv, tt , pulse_per_rotation_list
    
    def validate_joint_positions(self, positions: List[float], dt: float = 0.01) -> List[float]:
        validated_positions = []
        last_positions = self.__last_positions

        for i, (position, joint) in enumerate(zip(positions, self.joints)):
            min_pos, max_pos = joint.joint_limit[0], joint.joint_limit[1]
            min_vel, max_vel = joint.joint_limit[2], joint.joint_limit[3]

            if position < min_pos:
                validated_position = min_pos
            elif position > max_pos:
                validated_position = max_pos
            else:
                validated_position = position

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

                    if validated_position < min_pos:
                        validated_position = min_pos
                    elif validated_position > max_pos:
                        validated_position = max_pos

            validated_positions.append(validated_position)

        # Update recorded last position
        self.__last_positions = validated_positions.copy()

        return validated_positions
    
    def validate_joint_velocities(self, velocities: List[float], dt: float = 0.01) -> List[float]:
        validated_velocities = []
        last_velocities = self.__last_velocities

        for i, (velocity, joint) in enumerate(zip(velocities, self.joints)):
            min_vel, max_vel = joint.joint_limit[2], joint.joint_limit[3]
            min_acc, max_acc = joint.joint_limit[4], joint.joint_limit[5]

            if velocity < min_vel:
                validated_velocity = min_vel
            elif velocity > max_vel:
                validated_velocity = max_vel
            else:
                validated_velocity = velocity

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

            validated_velocities.append(validated_velocity)

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
        
    def init_pose(self, init_pose: List[float], step_limits: List[float]):
        while self.data_interface.ok() and not self.pose_initialized:
            if self.__motor_count is not None and self.__current_positions is not None and self.__pulse_per_rotation_list is not None: 
                if not self.__api_initialized:
                    api_init = public_api_down_pb2.APIDown()
                    api_init.arm_command.api_control_initialize = True
                    bin = api_init.SerializeToString()
                    self.__pub_ws_down(bin)
                api_down = public_api_down_pb2.APIDown()
                for i in range(self.__motor_count):
                    error = init_pose[i] - self.__current_positions[i]
                    error = np.clip(error, -step_limits[i], step_limits[i])
                    target_position = self.__current_positions[i] + error
                    api_down.arm_command.motor_targets.targets.add().position = int((target_position / (2 * PI)) * self.__pulse_per_rotation_list[i] + 65535.0 / 2.0)
                bin = api_down.SerializeToString()
                self.__pub_ws_down(list(bin))
                if all(abs(init_pose[i] - self.__current_positions[i]) < 0.01 for i in range(self.__motor_count)):
                    self.__pose_initialized = True
                    self.data_interface.logi("Initial pose reached.")
                    break
            self.data_interface.sleep()
    
def main():
    arm = ArmDataInterface()
    try:
        arm.init_pose(arm.pose_init_params, arm.step_limits)
        while arm.data_interface.ok():
            arm.data_interface.sleep()
    except KeyboardInterrupt:
        arm.data_interface.logi("Received Ctrl-C.")
    finally:
        arm.data_interface.shutdown()
    exit(0)

if __name__ == '__main__':
    main()