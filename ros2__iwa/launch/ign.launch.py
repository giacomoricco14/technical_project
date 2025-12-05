#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_path = get_package_share_directory('ros2_iiwa')
    urdf_file = os.path.join(pkg_path, 'urdf', 'iiwa.urdf')
    world_path = os.path.join(pkg_path, 'worlds', 'empty.world')  # opzionale

    # Leggi urdf
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    

    empty_world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            #'gui': LaunchConfiguration('gui'),
            'pause': 'true',
            'gz_args': ['-r ', 'empty.sdf'],
        }.items(),
    )

    # Pubblica la descrizione robotica
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': True}],
    )

    # Spawner del robot
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'iiwa_robot',
            '-allow_renaming', 'true' , 
            '-static', 'true',
        ]
        
    )

    # Joint state publisher (utile se controlli manualmente le pose)
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    return LaunchDescription([
        empty_world_launch,
        robot_state_pub,
        jsp,
        spawn
    ])
