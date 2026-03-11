from launch import LaunchDescription 
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration,PythonExpression, Command, FindExecutable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'runtime_config_package',
            default_value='iiwa_description',
            description='Package with the controller\'s configuration in "config" folder. \
                         Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file',
            default_value='iiwa_controllers.yaml',
            description='YAML file with the controllers configuration.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='iiwa_description',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='iiwa.config.xacro',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_planning',
            default_value='false',
            description='Start robot with Moveit2 `move_group` planning \
                         config for Pilz and OMPL.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_servoing',
            default_value='false',
            description='Start robot with Moveit2 servoing.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'initial_positions_file',
            default_value='initial_positions.yaml',
            description='Configuration file of robot initial positions for simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'command_interface',
            default_value='position',
            description='Robot command interface [position|velocity|effort].',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'base_frame_file',
            default_value='base_frame.yaml',
            description='Configuration file of robot base frame wrt World.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        )
    )


    # Initialize Arguments
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    use_planning = LaunchConfiguration('use_planning')
    use_servoing = LaunchConfiguration('use_servoing')
    initial_positions_file = LaunchConfiguration('initial_positions_file')
    command_interface = LaunchConfiguration('command_interface')
    base_frame_file = LaunchConfiguration('base_frame_file')
    namespace = LaunchConfiguration('namespace')
    use_sim = LaunchConfiguration('use_sim')

     # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'config', description_file]
            ),
            ' ',
            'initial_positions_file:=',
            initial_positions_file,
            ' ',
            'command_interface:=',
            command_interface,
            ' ',
            'base_frame_file:=',
            base_frame_file,
            ' ',
            'description_package:=',
            description_package,
            ' ',
            'runtime_config_package:=',
            runtime_config_package,
            ' ',
            'controllers_file:=',
            controllers_file,
            ' ',
        ]
    )

    robot_description = {'robot_description': robot_description_content}


     # Running with Moveit2 planning
    iiwa_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('iiwa_bringup'),
            '/launch',
            '/iiwa_planning.launch.py'
        ]),
        launch_arguments={
            'description_package': description_package,
            'description_file': description_file,
            'base_frame_file': base_frame_file,
        }.items(),
        condition=IfCondition(use_planning),
    )
    # Running with Moveit2 servoing
    iiwa_servoing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('iiwa_bringup'),
            '/launch',
            '/iiwa_servoing.launch.py'
        ]),
        launch_arguments={
            'description_package': description_package,
            'description_file': description_file,
            'base_frame_file': base_frame_file,
        }.items(),
        condition=IfCondition(use_servoing),
    )
    # Robot controllers configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            'config',
            controllers_file,
        ]
    )
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers, {'use_sim_time': True}],
        output='both',
        namespace=namespace,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager',
                   [namespace, 'controller_manager']],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller', '--controller-manager', [namespace, 'controller_manager']],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    #Delay `joint_state_broadcaster` after control_node
    delay_joint_state_broadcaster_spawner_after_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[joint_state_broadcaster_spawner],
        ),
        condition=UnlessCondition(use_sim),
    )

    # Delay start of velocity_controller until external torque broadcaster finished
    delay_velocity_controller_spawner_after_joint_state_broadcaster_spawner= RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[velocity_controller_spawner],
        )
    )

    return LaunchDescription(
        declared_arguments + [
        control_node,
        iiwa_planning_launch,
        iiwa_servoing_launch,
        delay_joint_state_broadcaster_spawner_after_control_node,
        delay_velocity_controller_spawner_after_joint_state_broadcaster_spawner,
        ]
    )
