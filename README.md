
# KUKA YouBot ROS 2 Simulation Workspace

This workspace contains a complete ROS 2 simulation and manipulation environment for the KUKA YouBot robot. It is a demonstration of mobile manipulation, navigation, and simulation using ROS 2 and Gazebo.

## Project Structure

The workspace is organized into several main packages and folders:

- **youbot_description/**: Contains robot description files (URDF, meshes, config, launch, RViz, and world files) for simulating the KUKA YouBot in Gazebo and other environments.
- **youbot_manipulation/**: Provides manipulation-related code, launch files, and resources for controlling the YouBot's arm and gripper.
- **youbot_navigation/**: Implements navigation, SLAM, localization, and path planning using the Navigation2 stack. Includes launch files, configuration, and a Jupyter tutorial.
- **youbot_perception/**: This one implements YOLOv8 system to the robot, as of now its still under development, but its successfully working navigational wise. 


## How to Use

1. **Build the workspace:**
	```bash
	cd /home/youssef/fyp_ws/src
	colcon build
	source install/setup.bash
	```

2. **Launch the robot simulation:**
	- See the README.md in each package (e.g., youbot_navigation/README.md) for detailed instructions.

3. **Run navigation, SLAM, or manipulation:**
	- Use the provided launch files in each package. For example, to launch navigation:
	  ```bash
	  ros2 launch youbot_navigation navigation.launch.py
	  ```

## Package Details

### youbot_description
- Robot model, URDF, meshes, and simulation assets
- Launch files for spawning in Gazebo and RViz

### youbot_manipulation
- Arm and gripper control
- Manipulation demos and scripts

### youbot_navigation
- Navigation2 stack integration
- SLAM, localization, and mapping
- Example Jupyter notebook for navigation workflow

### youbot_perception
- YOLOv8 integration to detect humans
- Move toward the human and stop at a certain distance
-further improvement: move the robotic arm to either give or take something from the human

## Video Demos

### navigation stack test:

Below is a demonstration video of Navigation2 in a TurtleBot3 house environment:

<video width="640" height="480" controls>
  <source src="nav2 simulation in turtlebot3 house env.mp4" type="video/mp4">
</video>

If the video does not play in your markdown viewer, you can open it directly from the workspace at:

`nav2 simulation in turtlebot3 house env.mp4`

### Manipulation package test:

<video width="640" height="480" controls>
  <source src="manipulation test in gazebo.mp4" type="video/mp4">
</video>

### Perception package test:

<video width="640" height="480" controls>
  <source src="robot_human_detection_test.mp4" type="video/mp4">
</video>