cmake_minimum_required(VERSION 3.8)
project(onrobot_gripper_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(onrobot_gripper_msgs REQUIRED)
find_package(onrobot_tool_api 1.1.0 REQUIRED CONFIG)

add_library(onrobot_gripper_semantics src/gripper_semantics.cpp)
target_compile_features(onrobot_gripper_semantics PUBLIC cxx_std_17)
target_include_directories(onrobot_gripper_semantics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(onrobot_gripper_semantics PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(onrobot_gripper_semantics PUBLIC onrobot::tool_api)

add_library(onrobot_gripper_hardware SHARED
  src/onrobot_gripper_system.cpp
  src/onrobot_isaac_system.cpp
  src/onrobot_parallel_gripper_fake_system.cpp
  src/onrobot_rg_system.cpp
  src/onrobot_three_finger_system.cpp)
target_compile_features(onrobot_gripper_hardware PUBLIC cxx_std_17)
target_include_directories(onrobot_gripper_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(onrobot_gripper_hardware
  hardware_interface onrobot_gripper_msgs pluginlib rclcpp rclcpp_lifecycle
  sensor_msgs)
target_link_libraries(onrobot_gripper_hardware
  onrobot_gripper_semantics onrobot::tool_api)

# The Tool API is installed as a separate, ROS-independent prefix.
# Keep isolated source/bundle installs relocatable so
# pluginlib can load this hardware plugin after the ROS workspace is sourced,
# without requiring users to export LD_LIBRARY_PATH manually. System package
# installs still use the platform loader cache.
set_target_properties(onrobot_gripper_hardware PROPERTIES
  INSTALL_RPATH "$ORIGIN/../../onrobot_tool_api/lib")

add_executable(joint_state_effort_filter src/joint_state_effort_filter.cpp)
target_compile_features(joint_state_effort_filter PRIVATE cxx_std_17)
ament_target_dependencies(joint_state_effort_filter rclcpp sensor_msgs)
target_link_libraries(joint_state_effort_filter onrobot_gripper_semantics)

pluginlib_export_plugin_description_file(hardware_interface onrobot_gripper_hardware.xml)

set(onrobot_internal_tests "${CMAKE_CURRENT_SOURCE_DIR}/../internal/${PROJECT_NAME}/tests.cmake")
if(BUILD_TESTING AND EXISTS "${onrobot_internal_tests}")
  include("${onrobot_internal_tests}")
endif()

install(TARGETS onrobot_gripper_semantics onrobot_gripper_hardware
  EXPORT export_onrobot_gripper_hardware
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS joint_state_effort_filter
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES onrobot_gripper_hardware.xml README.md
  DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_onrobot_gripper_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(onrobot_tool_api)
ament_export_libraries(onrobot_gripper_semantics)
ament_package()
