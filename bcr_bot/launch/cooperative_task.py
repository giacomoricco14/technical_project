import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # ========================================================================
    # DEFINIZIONE DEI PERCORSI DEI PACCHETTI E VARIABILI
    # ========================================================================
    bcr_bot_dir = get_package_share_directory('bcr_bot')
    iiwa_bringup_dir = get_package_share_directory('iiwa_bringup')
    ros2_kdl_dir = get_package_share_directory('ros2_kdl_package')

    # --- NUOVO: Dichiariamo l'argomento da riga di comando ---
    use_obstacle_arg = DeclareLaunchArgument(
        'use_obstacle',
        default_value='True',
        description='Imposta su True per lanciare ostacolo_dinamico.py, False per disabilitarlo'
    )
    
    # --- NUOVO: Creiamo la variabile che leggerà il valore ---
    use_obstacle = LaunchConfiguration('use_obstacle')

    # ========================================================================
    # STEP 0: Variabile d'ambiente necessaria per l'iiwa
    # ========================================================================
    set_env_var = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value='/home/user/ros2_ws/src/ros2_iiwa/iiwa_description/models'
    )

    # ========================================================================
    # STEP 1: Gazebo e BCR Bot (Tempo: 0s)
    # ========================================================================
    bcr_bot_ign = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bcr_bot_dir, 'launch', 'ign.launch.py')),
        launch_arguments={
            'camera_enabled': 'True',
            'stereo_camera_enabled': 'False',
            'two_d_lidar_enabled': 'True',
            'position_x': '-2.5',
            'position_y': '0.0',
            'orientation_yaw': '0.0',
            'odometry_source': 'world',
            'world_file': 'small_warehouse.sdf'
        }.items()
    )

    # ========================================================================
    # STEP 2: IIWA Bringup (Tempo: 5s)
    # ========================================================================
    iiwa_bringup = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(iiwa_bringup_dir, 'launch', 'iiwa.launch.py')),
                launch_arguments={
                    'command_interface': 'velocity',
                    'robot_controller': 'velocity_controller'
                }.items()
            )
        ]
    )

    # ========================================================================
    # STEP 3: ROS 2 KDL per l'iiwa (Tempo: 10s)
    # ========================================================================
    ros2_kdl = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(ros2_kdl_dir, 'launch', 'ros2_kdl.launch.py')),
                launch_arguments={
                    'cmd_interface': 'velocity_ctrl_null'
                }.items()
            )
        ]
    )

    # ========================================================================
    # STEP 4: Navigation 2 (Tempo: 15s)
    # ========================================================================
    nav2_stack = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bcr_bot_dir, 'launch', 'nav2.launch.py')),
                launch_arguments={'namespace': ''}.items() # Correzione per il problema del doppio slash //
            )
        ]
    )

    # ========================================================================
    # STEP 5: Generazione Ostacolo Dinamico (Tempo: 20s) - ORA CONDIZIONALE!
    # ========================================================================
    ostacolo_dinamico = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='bcr_bot',
                executable='ostacolo_dinamico.py',
                name='ostacolo_dinamico_node',
                output='screen',
                condition=IfCondition(use_obstacle) # <--- LA MAGIA È QUI
            )
        ]
    )

    # ========================================================================
    # STEP 6: Visual Coordinator (Tempo: 25s)
    # ========================================================================
    visual_coordinator = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='bcr_bot',
                executable='visual_coordinator.py',
                name='visual_coordinator_node',
                output='screen'
            )
        ]
    )

    # ========================================================================
    # STEP 7: Back and Forth (IL ROBOT PARTE!) - (Tempo: 40s)
    # ========================================================================
    back_and_forth = TimerAction(
        period=40.0,
        actions=[
            Node(
                package='bcr_bot',
                executable='back_and_forth.py',
                name='nav2_back_and_forth_controller',
                output='screen'
            )
        ]
    )

    # ========================================================================
    # ASSEMBLAGGIO FINALE DEL LAUNCH DESCRIPTION
    # ========================================================================
    ld = LaunchDescription()
    
    ld.add_action(use_obstacle_arg) # Non dimentichiamo di aggiungerlo qui!
    ld.add_action(set_env_var)
    ld.add_action(bcr_bot_ign)
    ld.add_action(iiwa_bringup)
    ld.add_action(ros2_kdl)
    ld.add_action(nav2_stack)
    ld.add_action(ostacolo_dinamico)
    ld.add_action(visual_coordinator)
    ld.add_action(back_and_forth)

    return ld