#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import json
import numpy as np
from math import pi as PI
from dataclasses import dataclass

from std_msgs.msg import UInt8MultiArray
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
    joint_limit: list[float] # [min_pos, max_pos, min_vel, max_vel, min_acc, max_acc]

class ArmDataInterface:
    def __init__(self):
        self.data_interface = DataInterface(node_name = "arm_trans")
        self.__ws_down_pub = self.data_interface.create_publisher(UInt8MultiArray, "ws_down")
        self.__motor_status_pub = self.data_interface.create_publisher(JointState, "joint_states")
        self.data_interface.create_subscriber(UInt8MultiArray, "ws_up", self.__ws_up_callback)
        self.data_interface.create_subscriber(XmsgArmJointParamList, "joints_cmd", self.__joints_cmd_callback)

        self.__last_positions = []
        self.__last_velocities = []
        self.json_path = self.data_interface.get_pkg_share_path("hex_arm") + "/config/joints.json"
        self.__motor_count: int = 0
        self.__api_initialized: bool = False

        self.joints: list[JointParam] = []
        json_data = self.data_interface.load_from_json(self.json_path)
        if json_data is not None:
            if isinstance(json_data["joints"], list):
                for joint_data in json_data["joints"]:
                    self.joints.append(JointParam(joint_name=joint_data["joint_name"], joint_limit=joint_data["joint_limit"]))
                self.data_interface.logi(f"Load joint parameters from {self.json_path}.")
                self.data_interface.logi(f"Joint parameters: {self.joints}.")
            else:
                self.data_interface.loge(f"Error: Please check your joints parameters in {self.json_path}.")
        
        self.data_interface.logi("ArmDataInterface initialized.")

    def __pub_ws_down(self, data: list[int]):
        msg = UInt8MultiArray()
        msg.data = data
        self.__ws_down_pub.publish(msg)

    def __pub_motor_status(self, pos: list[float], vel: list[float], eff: list[float]):
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
        # parse data
        pp, vv, tt = self.prase_motor_data(api_up)
        # publish data
        self.__pub_motor_status(pp, vv, tt)

    def __joints_cmd_callback(self, msg: XmsgArmJointParamList):
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
        for joint in msg.joints:
            modes.append(joint.mode)
            positions.append(joint.position)
            velocities.append(joint.velocity)
            efforts.append(joint.effort)
        positions = self.validate_joint_positions(list(positions))
        velocities = self.validate_joint_velocities(list(velocities))

        api_down = public_api_down_pb2.APIDown()
        for i in range(length):
            if modes[i] == "position_mode":
                joint_cmd = api_down.arm_command.motor_targets.targets.add()
                joint_cmd.position = positions[i]
            elif modes[i] == "velocity_mode":
                joint_cmd = api_down.arm_command.motor_targets.targets.add()
                joint_cmd.speed = velocities[i]
            elif modes[i] == "torque_mode":
                joint_cmd = api_down.arm_command.motor_targets.targets.add()
                joint_cmd.torque = efforts[i]
            else:
                self.data_interface.logw(f"Unknown mode: {modes[i]}. Set speed to 0.0.")
                joint_cmd = api_down.arm_command.motor_targets.targets.add()
                joint_cmd.speed = 0.0
        bin = api_down.SerializeToString()
        self.__pub_ws_down(list(bin))

    def prase_motor_data(self, api_up: public_api_up_pb2.APIUp) -> tuple[list, list, list]:
        pp = []
        vv = []
        tt = []
        if api_up.arm_status.IsInitialized():
            for motor_status in api_up.arm_status.motor_status:
                torque = motor_status.torque
                speed = motor_status.speed
                position = (motor_status.position % motor_status.pulse_per_rotation) / motor_status.pulse_per_rotation * (2.0 * PI) - PI # radian
                pp.append(position)
                vv.append(speed)
                tt.append(torque)
        return pp, vv, tt
    
    def validate_joint_positions(self, positions: list[float], dt: float = 0.01) -> list[float]:
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
    
    def validate_joint_velocities(self, velocities: list[float], dt: float = 0.01) -> list[float]:
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
    
def main():
    arm = ArmDataInterface()
    try:
        while arm.data_interface.ok():
            arm.data_interface.sleep()
    except KeyboardInterrupt:
        arm.data_interface.logi("Received Ctrl-C.")
    finally:
        arm.data_interface.shutdown()
    exit(0)

if __name__ == '__main__':
    main()