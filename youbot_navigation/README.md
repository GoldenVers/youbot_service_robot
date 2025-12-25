# NAVIGATION FOR THE YOUBOT

The navigation2 stack is integrated to the current kuka youbot system, this time its actually working properly, with the online asynce and the nav2 stack

## commands right now 

first off, launch the description package

```
ros2 launch youbot_navigation.launch.py
```

this one will bringup the youbot in gazebo and rviz, with the joint state publisher and the xterm teleop twist menu, 

for slam run:

```
ros2 launch youbot_navigation slam.launch.py slam_params_file:=/home/youssef/fyp_ws/src/youbot_navigation/config/slam_toolbox_params.yaml use_sim_time:=true
```


after saving and serializing the maps, we go to the mapper_params_online_async.yaml and add the path to the serialized map with the extenstion (.serial or .yaml)

after that, we launch this 

```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/youssef/fyp_ws/src/youbot_navigation/config/mapper_params_online_async.yaml

```

then we expect to see the map that was already created appear in rviz, and ofcourse change the fixed frame to /map

that's localization 

then for the navigation, while the previous command is running, we run this one 

```
ros2 launch youbot_navigation nav2.launch.py 

```

that will automatically launch the nav2 stack, in rviz we choose the /global_cost_map and use goal_pose to move around. 



## video demo 

<video width="640" height="480" controls>
  <source src="/home/youssef/fyp_ws/src/nav2 simulation in turtlebot3 house env.mp4" type="video/mp4">
</video>


## future note

the nav2 params needs to be tweaked, as its made for turtlebot3 originally, right now the robot struggle with tight spaces, and the global costmap is rendered big