cmake_minimum_required(VERSION 3.16)
project(onrobot_ros2 NONE)

find_package(ament_cmake REQUIRED)

install(FILES README.md "${CMAKE_CURRENT_SOURCE_DIR}/../CHANGELOG.md"
  DESTINATION share/${PROJECT_NAME})

ament_package()
