#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Declare the launch arguments
    url = DeclareLaunchArgument(
        'url',
        default_value='ws://0.0.0.0:8439',
        description='The URL of the robot.'
    )

    read_only = DeclareLaunchArgument(
        'read_only',
        default_value='false',
        description='Whether to read only the chassis state.'
    )

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
        default_value='16',
        description='The series of the Archer (integer).'
    )

    # Define the node
    xpkg_bridge_node = Node(
        package='xpkg_bridge',
        executable='xnode_bridge',
        name='xnode_bridge',
        output='screen',
        emulate_tty=True,
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