#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

def generate_launch_description():
    # Note: ROS2 launch does not provide control over node shutdown order.
    # It is recommended to launch xpkg_bridge_node separately to ensure proper
    # command forwarding during the shutdown phase and prevent premature termination
    # of the bridge node that may cause shutdown commands to fail.
    enable_bridge = DeclareLaunchArgument(
        'enable_bridge',
        default_value='false',
        description='Whether to enable the xpkg_bridge node, you can set it to false if you want to launch the node separately.'
    )
    url = DeclareLaunchArgument(
        'url',
        default_value='ws://172.18.23.18:8439',
        description='The URL of the robot.'
    )
    read_only = DeclareLaunchArgument(
        'read_only',
        default_value='false',
        description='Whether to read only the chassis state.'
    )

    # hex_arm parameters
    joint_config = FindPackageShare('hex_arm').find(
        'hex_arm') + '/config/joints.json'
    joint_config_path = DeclareLaunchArgument(
        'joint_config_path',
        default_value=joint_config,
        description='The path to the joint config file.'
    )
    init_pose_file_path = FindPackageShare('hex_arm').find(
        'hex_arm') + '/config/init_pose.json'
    init_pose_path = DeclareLaunchArgument(
        'init_pose_path',
        default_value=init_pose_file_path,
        description='The path to the init pose file.'
    )
    gripper_type = DeclareLaunchArgument(
        'gripper_type',
        default_value='0',
        description='The type of the Gripper (integer).'
    )
    arm_series = DeclareLaunchArgument(
        'arm_series',
        default_value='14',
        description='The series of the Archer (integer).'
    )

    # Define the node
    xpkg_bridge_node = Node(
        package='xpkg_bridge',
        executable='xnode_bridge',
        name='xnode_bridge',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_bridge')),
        parameters=[{
            'url': LaunchConfiguration('url'),
            'read_only': LaunchConfiguration('read_only'),
        }],
        remappings=[
            # subscribe
            ('/ws_down', '/ws_down'),
            # publish
            ('/ws_up', '/ws_up')
        ]
    )

    hex_arm_node = Node(
        package='hex_arm',
        executable='arm_trans',
        name='hex_arm',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'joint_config_path': LaunchConfiguration('joint_config_path'),
            'init_pose_path': LaunchConfiguration('init_pose_path'),
            'gripper_type': LaunchConfiguration('gripper_type'),
            'arm_series': LaunchConfiguration('arm_series'),
        }],
        remappings=[
            ('/joint_states', '/joint_states'),
            ('/joints_cmd', '/joints_cmd'),
        ]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        # arguments
        enable_bridge,
        url,
        read_only,
        joint_config_path,
        init_pose_path,
        gripper_type,
        arm_series,
        # nodes
        xpkg_bridge_node,
        hex_arm_node
    ])