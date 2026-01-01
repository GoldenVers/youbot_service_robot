from setuptools import find_packages, setup

package_name = 'youbot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['youbot_perception', 'youbot_perception.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'rosidl_default_runtime',
    ],
    zip_safe=True,
    maintainer='youssef',
    maintainer_email='goldenyouss.art@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image = youbot_perception.image:main',
            'yolo_ros2_pt = youbot_perception.yolo_ros2_pt:main',
            'yolov8_ros2_subscriber = youbot_perception.yolov8_ros2_subscriber:main',
            'follow_human = youbot_perception.follow_human:main',
            'human_position_pub = youbot_perception.human_position_pub:main',
        ],
    },
)
