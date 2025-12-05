#!/usr/bin/env python3
import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_path = get_package_share_directory('ros2_iiwa') #package path
    urdf_file = os.path.join(pkg_path, 'urdf', 'iiwa.urdf') #urdf path

    # read the urdf and store the robot description
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    rviz_config_path = os.path.join(pkg_path, 'config', 'standing_iiwa.rviz') #rviz path    

    # Params: containing the robot description
    params = {'robot_description': robot_desc}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        #condition=IfCondition(LaunchConfiguration('jsp_gui'))

    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        #jsp_gui,
        robot_state_publisher,
        jsp,
        rviz2,
    ])
