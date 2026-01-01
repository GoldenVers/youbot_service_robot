cmake_minimum_required(VERSION 3.5)
project(youbot_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InferenceResult.msg"
  "msg/Yolov8Inference.msg"
  "msg/HumanPositionArray.msg"
  "msg/HumanPosition.msg"
  DEPENDENCIES std_msgs
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)


ament_package()
