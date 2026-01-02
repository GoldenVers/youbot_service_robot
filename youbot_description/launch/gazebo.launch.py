import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# Fix for VS Code snap environment pollution that breaks RViz
# Remove snap-related GTK paths that interfere with ROS 2
if 'GTK_PATH' in os.environ and 'snap' in os.environ.get('GTK_PATH', ''):
    del os.environ['GTK_PATH']
if 'GTK_EXE_PREFIX' in os.environ and 'snap' in os.environ.get('GTK_EXE_PREFIX', ''):
    del os.environ['GTK_EXE_PREFIX']


def generate_launch_description():

    package_name = 'youbot_description'
    pkg_share = get_package_share_directory(package_name)
    
    # Get the URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'youbot.urdf')
    
    # Read URDF content
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    
    # Set GAZEBO_MODEL_PATH to include the package share directory
    # This helps Gazebo find the meshes referenced with package:// URIs
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(pkg_share, '..') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )
    
    # Robot State Publisher - publishes robot_description topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }],

    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'verbose': 'true'}.items()
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'youbot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rviz.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    teleop = Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            remappings=[
                ('cmd_vel', '/youbot/cmd_vel')
            ],
            output='screen',
            prefix='xterm -e'
        )
    
    #for future debugging, the mecanum ros2 controllers are not initiated.
    
    mecanum = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_cont"
        ]
    )

    joint_broad = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster"
        ]
    )






    
    return LaunchDescription([
        gazebo_model_path,
        gazebo,
        spawn_entity,
        rsp,
        teleop,
        joint_broad,
    ])