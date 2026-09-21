cmake_minimum_required(VERSION 3.8)
project(onrobot_gripper_rviz_plugins)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(onrobot_gripper_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Test Widgets)

add_library(onrobot_gripper_rviz SHARED
  include/onrobot_gripper_rviz_plugins/branded_panel.hpp
  include/onrobot_gripper_rviz_plugins/gripper_control_panel.hpp
  include/onrobot_gripper_rviz_plugins/force_history_model.hpp
  include/onrobot_gripper_rviz_plugins/force_history_panel.hpp
  include/onrobot_gripper_rviz_plugins/realtime_control_panel.hpp
  include/onrobot_gripper_rviz_plugins/panel_settings.hpp
  include/onrobot_gripper_rviz_plugins/three_finger_control_panel.hpp
  resource/onrobot_brand.qrc
  src/branded_panel.cpp src/gripper_control_panel.cpp
  src/force_history_panel.cpp
  src/realtime_control_panel.cpp src/panel_settings.cpp
  src/three_finger_control_panel.cpp)
target_compile_features(onrobot_gripper_rviz PUBLIC cxx_std_17)
target_include_directories(onrobot_gripper_rviz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(onrobot_gripper_rviz control_msgs controller_manager_msgs
  onrobot_gripper_msgs pluginlib rcl_interfaces rclcpp rclcpp_action rviz_common sensor_msgs std_msgs)
target_link_libraries(onrobot_gripper_rviz Qt5::Widgets)

add_executable(gripper_effort_marker src/gripper_effort_marker.cpp)
ament_target_dependencies(gripper_effort_marker geometry_msgs rclcpp sensor_msgs visualization_msgs)

pluginlib_export_plugin_description_file(rviz_common onrobot_gripper_rviz.xml)

set(onrobot_internal_tests "${CMAKE_CURRENT_SOURCE_DIR}/../internal/${PROJECT_NAME}/tests.cmake")
if(BUILD_TESTING AND EXISTS "${onrobot_internal_tests}")
  include("${onrobot_internal_tests}")
endif()

install(TARGETS onrobot_gripper_rviz gripper_effort_marker
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES onrobot_gripper_rviz.xml README.md
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
ament_package()
