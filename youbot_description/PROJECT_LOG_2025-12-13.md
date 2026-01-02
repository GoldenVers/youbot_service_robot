# YouBot Project Log - December 13, 2025

This document logs the progress, configurations, and fixes made to the YouBot robot description package for ROS2 Humble.

---

## Table of Contents
1. [Project Overview](#1-project-overview)
2. [Package Structure](#2-package-structure)
3. [Gazebo Classic Simulation](#3-gazebo-classic-simulation)
4. [MuJoCo Simulation](#4-mujoco-simulation)
5. [RViz Visualization](#5-rviz-visualization)
6. [Issues Fixed Today](#6-issues-fixed-today)
7. [Commands Reference](#7-commands-reference)
8. [Next Steps](#8-next-steps)

---

## 1. Project Overview

### Robot: KUKA YouBot
The YouBot is a mobile manipulator robot consisting of:
- **Mobile Base**: Omnidirectional platform with 4 mecanum wheels
- **Robotic Arm**: 5-DOF manipulator arm
- **Gripper**: 2-finger parallel gripper
- **Sensors**: Hokuyo laser scanner

### Environment
- **OS**: Linux (Ubuntu 22.04)
- **ROS Version**: ROS2 Humble
- **Simulators**: Gazebo Classic 11, MuJoCo
- **Display Server**: Wayland (with X11/XCB compatibility layer)

---

## 2. Package Structure

```
youbot_description/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package manifest
├── setup.py                # Python package setup
├── setup.cfg               # Setup configuration
│
├── config/
│   └── youbot.yaml         # Controller configuration
│
├── controller/
│   └── ros_controller.urdf.xacro
│
├── launch/
│   ├── gazebo.launch.py    # ✅ Main Gazebo launch file (FIXED)
│   ├── gazebo_urdf.launch.py
│   └── rviz.launch.py      # RViz visualization
│
├── meshes/
│   ├── youbot_base/        # Base meshes (.dae, .stl)
│   ├── youbot_arm/         # Arm meshes
│   ├── youbot_gripper/     # Gripper meshes
│   ├── youbot_plate/       # Plate meshes
│   └── sensors/            # Sensor meshes (Hokuyo)
│
├── mujoco/
│   ├── youbot.xml          # MuJoCo MJCF model
│   ├── view_mujoco.py      # MuJoCo viewer script
│   ├── convert.py          # URDF to MJCF converter
│   └── assets/             # MuJoCo assets
│
├── robots/
│   ├── youbot.urdf.xacro
│   ├── youbot_arm_only.urdf.xacro
│   ├── youbot_base_only.urdf.xacro
│   └── youbot_dual_arm.urdf.xacro
│
├── rviz/
│   └── view.rviz           # RViz configuration
│
├── sdf/                    # SDF format files
├── srdf/                   # Semantic Robot Description
│
└── urdf/
    └── youbot.urdf         # ✅ Main URDF file (FIXED)
```

---

## 3. Gazebo Classic Simulation

### 3.1 Launch File: `gazebo.launch.py`

The main Gazebo launch file has been fixed and now works correctly.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'youbot_description'
    pkg_share = get_package_share_directory(package_name)
    
    # Get the URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'youbot.urdf')
    
    # Read URDF content
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    
    # Set GAZEBO_MODEL_PATH
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(pkg_share, '..') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 
                        'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'verbose': 'true'}.items()
    )

    # Spawn robot entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'youbot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_model_path,
        robot_state_publisher,
        gazebo,
        spawn_entity,
    ])
```

### 3.2 Gazebo Plugins Configured in URDF

The following Gazebo plugins are configured in `urdf/youbot.urdf`:

#### Joint State Publisher Plugin

The ros2 control plugins are available and were initially integrated to the system, however there is an issue with odometry publishing with the mecanum plugin for motion, which hindered the navigation2 stack integration, so I proceeded with the planar move plugin for now

#### Planar Move Plugin (Base Controller)
```xml
<gazebo>
  <plugin name='base_controller' filename='libgazebo_ros_planar_move.so'>
    <ros>
      <namespace>/</namespace>
    </ros>
    <update_rate>100</update_rate>
    <publish_rate>10</publish_rate>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <robot_base_frame>base_footprint</robot_base_frame>
  </plugin>
</gazebo>
```

#### Laser Scanner Plugin
```xml
<gazebo reference="base_laser_front_link">
  <sensor type="ray" name="hokuyo_front">
    <plugin name="gazebo_ros_head_hokuyo_controller" 
            filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/</namespace>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

### 3.3 ROS Topics Available After Launch

When Gazebo is running with the YouBot, these topics are available:

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands for base |
| `/odom` | `nav_msgs/Odometry` | Odometry from base controller |
| `/joint_states` | `sensor_msgs/JointState` | All joint states |
| `/scan` | `sensor_msgs/LaserScan` | Laser scanner data |
| `/robot_description` | `std_msgs/String` | URDF robot description |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |

---

## 4. MuJoCo Simulation

### 4.1 MuJoCo Model: `mujoco/youbot.xml`

A complete MuJoCo (MJCF) model has been created for the YouBot robot.

**Key Features:**
- Uses STL mesh files from the `meshes/` directory
- Full kinematic chain from base to gripper
- Proper inertial properties
- Actuators for all joints
- Sensors for joint positions/velocities

### 4.2 Joints in MuJoCo Model

| Joint Name | Type | Range | Description |
|------------|------|-------|-------------|
| `wheel_joint_fl` | continuous | - | Front-left wheel |
| `wheel_joint_fr` | continuous | - | Front-right wheel |
| `wheel_joint_bl` | continuous | - | Back-left wheel |
| `wheel_joint_br` | continuous | - | Back-right wheel |
| `arm_joint_1` | revolute | 0 to 5.89 rad | Base rotation |
| `arm_joint_2` | revolute | 0 to 2.70 rad | Shoulder |
| `arm_joint_3` | revolute | -5.18 to 0 rad | Elbow |
| `arm_joint_4` | revolute | 0 to 3.57 rad | Wrist pitch |
| `arm_joint_5` | revolute | 0 to 5.84 rad | Wrist roll |
| `gripper_finger_joint_l` | prismatic | 0 to 0.011 m | Left finger |
| `gripper_finger_joint_r` | prismatic | 0 to 0.011 m | Right finger |

### 4.3 Running MuJoCo Viewer

**Installation:**
```bash
pip install mujoco
```

**View the model:**
```bash
cd ~/youbot/src/youbot_description/mujoco
python3 view_mujoco.py
```

**Viewer Controls:**
- Mouse drag: Rotate camera
- Scroll: Zoom in/out
- Double click: Center on object
- Space: Pause/Resume simulation
- Backspace: Reset simulation
- Esc: Quit

### 4.4 MuJoCo for Deep Reinforcement Learning

Example Gymnasium environment template:

```python
import gymnasium as gym
import mujoco
import numpy as np

class YouBotEnv(gym.Env):
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path('youbot.xml')
        self.data = mujoco.MjData(self.model)
        
        # Define observation and action spaces
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, 
            shape=(self.model.nq + self.model.nv,), dtype=np.float64
        )
        self.action_space = gym.spaces.Box(
            low=-1, high=1, 
            shape=(self.model.nu,), dtype=np.float64
        )
    
    def step(self, action):
        self.data.ctrl[:] = action
        mujoco.mj_step(self.model, self.data)
        obs = self._get_obs()
        reward = self._compute_reward()
        done = False
        return obs, reward, done, False, {}
    
    def reset(self, seed=None):
        mujoco.mj_resetData(self.model, self.data)
        return self._get_obs(), {}
```

---

## 5. RViz Visualization

### Launch RViz only:
```bash
ros2 launch youbot_description rviz.launch.py
```

### RViz Launch File: `rviz.launch.py`

This launch file starts:
- `robot_state_publisher` - Publishes robot description and TF
- `joint_state_publisher_gui` - GUI for manually controlling joints
- `rviz2` - 3D visualization

### Configuration
The RViz config file is located at: `rviz/view.rviz`

---

## 6. Issues Fixed Today (December 13, 2025)

### Issue 1: Gazebo gzclient Crashing (Exit Code -9)

**Symptom:** 
- gzclient process dying with exit code -9 (SIGKILL)
- Wayland compatibility issues

**Root Cause:**
- System running Wayland display server
- Gazebo Classic has issues with Wayland

**Solution:**
- Set `QT_QPA_PLATFORM=xcb` environment variable
- Added `SetEnvironmentVariable` in launch file

---

### Issue 2: gzserver Segmentation Fault (Exit Code -11)

**Symptom:**
```
[ERROR] [gzserver-2]: process has died [pid 48955, exit code -11]
```

**Root Cause:**
Two problems in the URDF:

1. **Duplicate `base_controller` plugin** (appeared twice in URDF)
   - Lines 40-58 and 333-358 both defined the same plugin
   - Error: `Found multiple nodes with same name: /youbot/base_controller`

2. **`gazebo_ros_p3d` plugin with invalid body_name**
   - Plugin referenced `pen` link
   - Error: `body_name: pen does not exist`
   - Gazebo's URDF-to-SDF converter renames links

**Solution:**

1. Removed the duplicate `base_controller` plugin (kept only one instance)

2. Disabled the `gazebo_ros_p3d` plugin by commenting it out:
```xml
<!-- p3d plugin disabled - causes issues with link name conversion
<gazebo>
  <plugin name="gazebo_ros_p3d" filename="libgazebo_ros_p3d.so">
    ...
  </plugin>
</gazebo>
-->
```

---

### Issue 3: spawn_entity Timeout

**Symptom:**
```
[spawn_entity.py-6] [ERROR] Service /spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?
```

**Root Cause:**
- gzserver crashing before spawn_entity could connect

**Solution:**
- Fixed by resolving Issues 1 and 2 above

---

### Issue 4: Material 'black' Undefined Warning

**Symptom:**
```
Warning: link 'base_laser_front_link' material 'black' undefined.
```

**Status:** Non-critical warning, doesn't affect functionality

**Cause:** The URDF references a material named 'black' that isn't defined

**Potential Fix:** Add material definition to URDF:
```xml
<material name="black">
  <color rgba="0.1 0.1 0.1 1.0"/>
</material>
```

---

## 7. Commands Reference

### Build Package
```bash
cd ~/youbot/src/youbot_description
colcon build --packages-select youbot_description --symlink-install
source install/setup.bash
```

### Launch Gazebo Simulation
```bash
source ~/youbot/src/youbot_description/install/setup.bash
ros2 launch youbot_description gazebo.launch.py
```

### Launch RViz Only
```bash
source ~/youbot/src/youbot_description/install/setup.bash
ros2 launch youbot_description rviz.launch.py
```

### View MuJoCo Model
```bash
cd ~/youbot/src/youbot_description/mujoco
python3 view_mujoco.py
```

### Kill Gazebo Processes
```bash
pkill -9 gzserver; pkill -9 gzclient
```

### Check ROS Topics
```bash
ros2 topic list
ros2 topic echo /youbot/joint_states
```

### Move Robot Base
```bash
ros2 topic pub /youbot/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

---

## 8. Next Steps

### Potential Improvements:

1. **Add ros2_control integration**
   - Enable hardware interface for arm control
   - Implement trajectory controllers

2. **MoveIt2 Integration**
   - Motion planning for the arm
   - Pick and place tasks

3. **Navigation Stack**
   - Nav2 integration for autonomous navigation
   - SLAM with the laser scanner

4. **MuJoCo + ROS2 Bridge**
   - Create ROS2 node for MuJoCo simulation
   - Enable same control interface as Gazebo

5. **Migrate to Gazebo Sim (Ignition)**
   - Gazebo Classic reaches EOL January 2025
   - Convert URDF/SDF for new Gazebo

6. **Fix remaining warnings**
   - Define missing 'black' material
   - Clean up `self_collide` warnings

---

## Summary

| Component | Status | Notes |
|-----------|--------|-------|
| URDF Model | ✅ Working | Fixed duplicate plugins |
| Gazebo Launch | ✅ Working | Robot spawns correctly |
| MuJoCo Model | ✅ Working | Full robot with actuators |
| RViz | ✅ Working | Visualization ready |
| Base Control | ✅ Working | `/youbot/cmd_vel` topic |
| Joint States | ✅ Working | Published on `/youbot/joint_states` |
| Laser Scanner | ✅ Working | Published on `/youbot/scan` |
| Arm Control | ⚠️ Partial | Needs trajectory controller |

---

*Log created: December 13, 2025*
