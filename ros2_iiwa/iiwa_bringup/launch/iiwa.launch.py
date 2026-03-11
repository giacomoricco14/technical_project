# Copyright 2022 ICube Laboratory, University of Strasbourg
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, OrSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('runtime_config_package', default_value='iiwa_description'))
    declared_arguments.append(DeclareLaunchArgument('controllers_file', default_value='iiwa_controllers.yaml'))
    declared_arguments.append(DeclareLaunchArgument('description_package', default_value='iiwa_description'))
    declared_arguments.append(DeclareLaunchArgument('description_file', default_value='iiwa.config.xacro'))
    declared_arguments.append(DeclareLaunchArgument('prefix', default_value='""'))
    declared_arguments.append(DeclareLaunchArgument('namespace', default_value='/'))
    declared_arguments.append(DeclareLaunchArgument('use_sim', default_value='true'))
    declared_arguments.append(DeclareLaunchArgument('use_fake_hardware', default_value='true'))
    declared_arguments.append(DeclareLaunchArgument('use_planning', default_value='false'))
    declared_arguments.append(DeclareLaunchArgument('use_servoing', default_value='false'))
    declared_arguments.append(DeclareLaunchArgument('robot_controller', default_value='iiwa_arm_controller'))
    declared_arguments.append(DeclareLaunchArgument('start_rviz', default_value='true'))
    declared_arguments.append(DeclareLaunchArgument('robot_ip', default_value='192.170.10.2'))
    declared_arguments.append(DeclareLaunchArgument('robot_port', default_value='30200'))
    declared_arguments.append(DeclareLaunchArgument('initial_positions_file', default_value='initial_positions.yaml'))
    declared_arguments.append(DeclareLaunchArgument('command_interface', default_value='position'))
    declared_arguments.append(DeclareLaunchArgument('base_frame_file', default_value='base_frame.yaml'))

    runtime_config_package = LaunchConfiguration('runtime_config_package')
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    prefix = LaunchConfiguration('prefix')
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_planning = LaunchConfiguration('use_planning')
    use_servoing = LaunchConfiguration('use_servoing')
    robot_controller = LaunchConfiguration('robot_controller')
    start_rviz = LaunchConfiguration('start_rviz')
    robot_ip = LaunchConfiguration('robot_ip')
    robot_port = LaunchConfiguration('robot_port')
    initial_positions_file = LaunchConfiguration('initial_positions_file')
    command_interface = LaunchConfiguration('command_interface')
    base_frame_file = LaunchConfiguration('base_frame_file')
    namespace = LaunchConfiguration('namespace')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([FindPackageShare(description_package), 'config', description_file]), ' ',
            'prefix:=', prefix, ' ', 'use_sim:=', use_sim, ' ', 'use_fake_hardware:=', use_fake_hardware, ' ',
            'robot_ip:=', robot_ip, ' ', 'robot_port:=', robot_port, ' ', 'initial_positions_file:=', initial_positions_file, ' ',
            'command_interface:=', command_interface, ' ', 'base_frame_file:=', base_frame_file, ' ',
            'description_package:=', description_package, ' ', 'runtime_config_package:=', runtime_config_package, ' ',
            'controllers_file:=', controllers_file, ' ', 'namespace:=', namespace,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    iiwa_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('iiwa_bringup'), '/launch', '/iiwa_planning.launch.py']),
        launch_arguments={'description_package': description_package, 'description_file': description_file, 'prefix': prefix, 'start_rviz': start_rviz, 'base_frame_file': base_frame_file, 'namespace': namespace, 'use_sim': use_sim}.items(),
        condition=IfCondition(use_planning),
    )

    iiwa_servoing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('iiwa_bringup'), '/launch', '/iiwa_servoing.launch.py']),
        launch_arguments={'description_package': description_package, 'description_file': description_file, 'prefix': prefix, 'base_frame_file': base_frame_file, 'namespace': namespace}.items(),
        condition=IfCondition(use_servoing),
    )

    robot_controllers = PathJoinSubstitution([FindPackageShare(runtime_config_package), 'config', controllers_file])
    rviz_config_file = PathJoinSubstitution([FindPackageShare(description_package), 'rviz', 'iiwa.rviz'])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
        namespace=namespace,
        condition=UnlessCondition(use_sim),
    )
    
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description],
    )
    # -----------------------

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description],
        condition=UnlessCondition(OrSubstitution(use_planning, use_sim)),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'iiwa', '-x', '1.0', '-y', '0.0', '-z', '0.0', '-allow_renaming', 'true'],
        condition=IfCondition(use_sim),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', [namespace, 'controller_manager']],
    )

    external_torque_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ets_state_broadcaster', '--controller-manager', [namespace, 'controller_manager']],
        condition=UnlessCondition(use_sim),
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[robot_controller, '--controller-manager', [namespace, 'controller_manager']],
    )

    delay_joint_state_broadcaster_spawner_after_spawn_entity = RegisterEventHandler(event_handler=OnProcessExit(target_action=spawn_entity, on_exit=[joint_state_broadcaster_spawner]), condition=IfCondition(use_sim))
    delay_joint_state_broadcaster_spawner_after_control_node = RegisterEventHandler(event_handler=OnProcessStart(target_action=control_node, on_start=[joint_state_broadcaster_spawner]), condition=UnlessCondition(use_sim))
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(event_handler=OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[rviz_node]), condition=IfCondition(start_rviz))
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(event_handler=OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[robot_controller_spawner]))

    bridge_camera = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=["/camera@sensor_msgs/msg/Image@gz.msgs.Image", "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo", "/iiwa/gripper_control@std_msgs/msg/Bool@gz.msgs.Boolean", "--ros-args", "-r", "/camera:=/videocamera"],
        output="screen"
    )

    # NODO AGGIUNTO PER LA PINZA DELL'IIWA (Detachable Joint)
    iiwa_gripper_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="iiwa_gripper_bridge",
        arguments=[
            "/iiwa/gripper_attach@std_msgs/msg/Empty]ignition.msgs.Empty",
            "/iiwa/gripper_detach@std_msgs/msg/Empty]ignition.msgs.Empty"
        ],
        output="screen"
    )

    nodes = [control_node, iiwa_planning_launch, iiwa_servoing_launch, spawn_entity, robot_state_pub_node, delay_joint_state_broadcaster_spawner_after_control_node, delay_joint_state_broadcaster_spawner_after_spawn_entity, delay_rviz_after_joint_state_broadcaster_spawner, external_torque_broadcaster_spawner, delay_robot_controller_spawner_after_joint_state_broadcaster_spawner, bridge_camera, iiwa_gripper_bridge]
    return LaunchDescription(declared_arguments + nodes)