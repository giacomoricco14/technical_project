# :robot: BCR Bot & KUKA iiwa: Cooperative Task

## :hammer_and_wrench: Configuration

First, make sure your system is up to date and the required SLAM packages are installed:

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox
colcon build
source install/setup.bash
```

The main simulation launch file accepts multiple launch arguments. Here is an example:

```bash
ros2 launch bcr_bot ign.launch.py \
    camera_enabled:=True \
    stereo_camera_enabled:=False \
    two_d_lidar_enabled:=True \
    position_x:=-2.5 \
    position_y:=0.0  \
    orientation_yaw:=0.0 \
    odometry_source:=world \
    world_file:=small_warehouse.sdf
```

---

## :world_map: Mapping with SLAM Toolbox

SLAM Toolbox is an open-source package designed to map the environment using laser scans and odometry, generating a 2D map for autonomous navigation.

**1. Start mapping:**
```bash
source install/setup.bash
ros2 launch bcr_bot mapping.launch.py
```

**2. Drive the robot:**
Use the teleop twist keyboard to control the robot manually and map the area:
```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/bcr_bot/cmd_vel
```

**3. Save the map:**
Once you are satisfied with the map, open a new terminal and save it:
```bash
source install/setup.bash
cd src/bcr_bot/config
ros2 run nav2_map_server map_saver_cli -f bcr_map
```

---

## :handshake: COOPERATIVE TASK

### :building_construction: Step 0: Setting up

Install the required Navigation 2 packages and rebuild your workspace:

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-rviz-plugins -y
colcon build
source install/setup.bash
```

### :rocket: Step 1: Executing the system

You have two options to run the cooperative task: using the automated master launch file (recommended) or launching each node manually in separate terminals.

#### :star2: Option A: Automated Launch (Recommended)
To start everything (Gazebo, Nav2, KDL, Vision, and the autonomous logic) in the correct synchronized order, simply run:

```bash
ros2 launch bcr_bot cooperative_task.py 
```

*Don't want the dynamic obstacle?* You can easily disable it using the launch argument:

```bash
ros2 launch bcr_bot cooperative_task.py use_obstacle:=False
```

#### :desktop_computer: Option B: Step-by-step Execution
If you prefer to debug or launch each node individually, open a **new terminal** for each of the following commands (remembering to run `source install/setup.bash` in each one):

**1. Start the differential drive robot (AMR):**
```bash
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

**2. Spawn the KUKA iiwa robot & controllers:**
```bash
export IGN_GAZEBO_RESOURCE_PATH=/home/user/ros2_ws/src/ros2_iiwa/iiwa_description/models
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

**3. Start the KDL kinematics package:**
```bash
ros2 launch ros2_kdl_package ros2_kdl.launch.py cmd_interface:=velocity_ctrl_null
```

**4. Start Navigation 2 (Nav2):**
```bash
ros2 launch bcr_bot nav2.launch.py
```

**5. Spawn the dynamic obstacle:**
```bash
ros2 run bcr_bot ostacolo_dinamico.py
```

**6. Start the Visual Coordinator (Camera logic):**
```bash
ros2 run bcr_bot visual_coordinator.py
```

**7. Start the Autonomous Navigation routine (Back & Forth):**
```bash
ros2 run bcr_bot back_and_forth.py
```
