cmake_minimum_required(VERSION 3.8)
project(onrobot_gripper_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(onrobot_gripper_msgs 1.2.0 REQUIRED)
find_package(onrobot_tool_api 1.2.0 CONFIG REQUIRED)
find_package(parallel_gripper_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_srvs REQUIRED)

add_library(onrobot_gripper_controller SHARED
  src/gripper_recovery_controller.cpp
  src/gripper_state_broadcaster.cpp
  src/onrobot_realtime_controller.cpp
  src/parallel_gripper_action_controller.cpp)
target_compile_features(onrobot_gripper_controller PUBLIC cxx_std_17)
target_include_directories(onrobot_gripper_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(onrobot_gripper_controller
  controller_interface diagnostic_msgs hardware_interface onrobot_gripper_msgs
  parallel_gripper_controller pluginlib rclcpp rclcpp_action rclcpp_lifecycle
  realtime_tools std_srvs)
target_link_libraries(onrobot_gripper_controller onrobot::tool_api)
ament_export_dependencies(onrobot_tool_api onrobot_gripper_msgs)

pluginlib_export_plugin_description_file(controller_interface onrobot_gripper_controller.xml)

set(onrobot_internal_tests "${CMAKE_CURRENT_SOURCE_DIR}/../internal/${PROJECT_NAME}/tests.cmake")
if(BUILD_TESTING AND EXISTS "${onrobot_internal_tests}")
  include("${onrobot_internal_tests}")
endif()

install(TARGETS onrobot_gripper_controller
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES onrobot_gripper_controller.xml README.md
  DESTINATION share/${PROJECT_NAME})
ament_package()
