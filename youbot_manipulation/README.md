### Manipulation with the kuka youbot 

I was able to test the manipulation (finally)

First I added the ros2 controllers to the urdf file

Then in the youbot_navigation/config I have made the YAML file that reads the controller.  The controllers are for both the gripper and the arms.

After that, I added the launch command in the manipulation.launch.py 

The launch file will launch the description launch file and then will launch the ros2 controllers, I kept this way to keep everything separate. 

Finally after the test and ros2 topic list, I tested the arm, and it actually worked!



## Commands 

First source and colcon build 

Then, 

```
ros2 launch youbot_manipulation manipulation.launch.py

```

you should see in the termminal the launching of the ros2 controllers 

for testing you should run 

```
ros2 topic pub --once \
/joint_trajectory_controller/joint_trajectory \
trajectory_msgs/msg/JointTrajectory "
header:
  stamp:
    sec: 0
    nanosec: 0
joint_names:
  - arm_joint_1
  - arm_joint_2
  - arm_joint_3
  - arm_joint_4
  - arm_joint_5
points:
  - positions: [0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start:
      sec: 2
      nanosec: 0

  - positions: [0.6, 0.4, -0.4, 0.5, 0.0]
    time_from_start:
      sec: 4
      nanosec: 0

  - positions: [0.0, 0.0, 0.0, 0.0, 0.0]
    time_from_start:
      sec: 6
      nanosec: 0
"
```

the robot arm should be moving


for the gripper, There is a small problem. Even though there is a mimic in the URDF for the gripper, they are not moving together for some reason. 


To test the gripper,

```
ros2 action send_goal /gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand "{
  command: {
    position: 0.04,
    max_effort: 20.0
  }
}"
```


the gripper should open 

To close it, just change the position value. 

Everything is shown in the video. 


Now I will move onto YOLO, and proper kinematics. 