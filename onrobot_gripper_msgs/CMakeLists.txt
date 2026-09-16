cmake_minimum_required(VERSION 3.8)

project(onrobot_gripper_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperState.msg"
  "msg/RealtimeCommand.msg"
  "msg/RealtimeState.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

install(DIRECTORY include/onrobot_gripper_msgs/
        DESTINATION include/onrobot_gripper_msgs/onrobot_gripper_msgs
        PATTERN "__pycache__" EXCLUDE
        PATTERN "*.pyc" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
