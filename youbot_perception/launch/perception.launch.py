from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    #this is a perception stack launch file
    #I have already made the description package and that one loads the urdf file of the youbot with mecanum wheel controller
    #now here I are just going to call that lauch file and then after that I am going to luanch other perception related stuff
    

    #calling out the description package

    description_pkg = get_package_share_directory('youbot_description')
    nav_pkg = get_package_share_directory('youbot_navigation')
    manip_pkg = get_package_share_directory('youbot_manipulation')


    #launch the youbot description in gazebo and rviz2

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource
        (
            [os.path.join(description_pkg,'launch','gazebo.launch.py')]
        )
    )

    yolo_node = Node(
        package='youbot_perception',
        executable='yolo_ros2_pt',
        output='screen'
    )
    yolov8_node = Node(
        package='youbot_perception',
        executable='yolov8_ros2_subscriber',
        output='screen'
    )
    follow_node = Node(
        package='youbot_perception',
        executable='follow_human',
        output='screen'
    )
    human_pos_node = Node(
        package='youbot_perception',
        executable='human_position_pub',
        output='screen'
    )

    return LaunchDescription([
        description,
        yolo_node,
        yolov8_node,
        follow_node,
        human_pos_node
    ])

