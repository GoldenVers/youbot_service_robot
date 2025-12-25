from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    #this is a manipulation stack launch file
    #I have already made the description package and that one loads the urdf file of the youbot with the joint_state_broadcaster controller
    #now here I am just going to call that launch file and then after that we are going to launch other stuff manipulation stuff as well

    #calling out the description package

    description_pkg = get_package_share_directory('youbot_description')
    nav_pkg = get_package_share_directory('youbot_navigation')
    manip_pkg = get_package_share_directory('youbot_manipulation')
    manip_config = os.path.join(manip_pkg,'config','arm_controller.yaml')


    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource
        (
            [os.path.join(description_pkg,'launch','gazebo.launch.py')]
        )
    )



    trajectory = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"]
    )

    gripper = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_action_controller"
        ]
    )





   
    return LaunchDescription([
        description,
        trajectory,
        gripper
    ])

 


   #adding the ros2 controllers for the arm trajectory and the gripper action 

