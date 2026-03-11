### Configuration

The launch file accepts multiple launch arguments,
```bash
ros2 launch bcr_bot ign.launch.py \
	camera_enabled:=True \
	stereo_camera_enabled:=False \
	two_d_lidar_enabled:=True \
	position_x:=-1.5 \
	position_y:=0.0  \
	orientation_yaw:=0.0 \
	odometry_source:=world \
	world_file:=small_warehouse.sdf
```

### Mapping with SLAM Toolbox

SLAM Toolbox is an open-source package designed to map the environment using laser scans and odometry, generating a map for autonomous navigation.

To start mapping:
```bash
ros2 launch bcr_bot mapping.launch.py
```

Use the teleop twist keyboard to control the robot and map the area:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/bcr_bot/cmd_vel
```

To save the map:
```bash
cd src/bcr_bot/config
ros2 run nav2_map_server map_saver_cli -f bcr_map
```

### COOPERATIVE TASK

#STEP 0: Setting up

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-rviz-plugins -y
colcon build
. install/setup.bash
```

#STEP 1: Executing all nodes
First of all, open a terminal and start the differential drive robot in the world:

```
ros2 launch bcr_bot ign.launch.py \
	camera_enabled:=True \
	stereo_camera_enabled:=False \
	two_d_lidar_enabled:=True \
	position_x:=-2.5  \
	position_y:=0.0  \
	orientation_yaw:=0.0 \
	odometry_source:=world \
	world_file:=small_warehouse.sdf
```	

Start the iiwa robot (with the controllers) in another shell and spawn it in Gazebo:

```bash
. install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=/home/user/ros2_ws/src/ros2_iiwa/iiwa_description/models
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

Then in another terminal, the kdl_package (with the velocity_ctrl_null) is launched:

```bash
. install/setup.bash
ros2 launch ros2_kdl_package ros2_kdl.launch.py cmd_interface:=velocity_ctrl_null
```

Then in another terminal, the node responsible for navigation (Nav2) is launched:	

```bash
. install/setup.bash
ros2 launch bcr_bot nav2.launch.py
```	


Then in another terminal, the node that spawns dynamically an obstacle is launched:	

```bash
. install/setup.bash
cd src/my-final-project/bcr_bot/scripts
python3 ostacolo_dinamico.py
```	
	
Then in another terminal, the node that starts the motion of the differential drive robot is launched:	

```bash
. install/setup.bash
cd src/my-final-project/bcr_bot/scripts
python3 back_and_forth.py
```	
	
Then in another terminal, the node that handles the visual coordination and pick and place task is launched:	

```bash
. install/setup.bash
cd src/my-final-project/bcr_bot/scripts
python3 visual_coordinator.py
```		


	
	
	
	


