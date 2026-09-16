cmake_minimum_required(VERSION 3.8)
project(onrobot_gripper_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(onrobot_gripper_msgs REQUIRED)
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

pluginlib_export_plugin_description_file(controller_interface onrobot_gripper_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Keep deterministic action-server interleaving hooks out of customer
  # libraries and installed headers. Tests compile the same local declaration.
  target_compile_definitions(onrobot_gripper_controller PRIVATE
    ONROBOT_GRIPPER_CONTROLLERS_TESTING)
  ament_add_gtest(test_controller_plugin tests/test_controller_plugin.cpp)
  ament_target_dependencies(test_controller_plugin controller_interface pluginlib)
  set_tests_properties(test_controller_plugin PROPERTIES
    ENVIRONMENT "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")

  ament_add_gtest(test_realtime_controller tests/test_realtime_controller.cpp)
  target_include_directories(test_realtime_controller PRIVATE include)
  target_link_libraries(test_realtime_controller onrobot_gripper_controller)
  ament_target_dependencies(
    test_realtime_controller controller_interface hardware_interface
    onrobot_gripper_msgs rclcpp rclcpp_lifecycle realtime_tools)
  set_tests_properties(test_realtime_controller PROPERTIES
    ENVIRONMENT "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")

  ament_add_gtest(test_gripper_state_broadcaster
    tests/test_gripper_state_broadcaster.cpp)
  target_include_directories(test_gripper_state_broadcaster PRIVATE include)
  target_link_libraries(test_gripper_state_broadcaster
    onrobot_gripper_controller)
  ament_target_dependencies(
    test_gripper_state_broadcaster controller_interface diagnostic_msgs
    onrobot_gripper_msgs rclcpp rclcpp_lifecycle realtime_tools)
  set_tests_properties(test_gripper_state_broadcaster PROPERTIES
    ENVIRONMENT "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")

  ament_add_gtest(test_parallel_gripper_action_controller
    tests/test_parallel_gripper_action_controller.cpp)
  target_compile_definitions(test_parallel_gripper_action_controller PRIVATE
    ONROBOT_GRIPPER_CONTROLLERS_TESTING)
  target_include_directories(test_parallel_gripper_action_controller PRIVATE include)
  target_link_libraries(test_parallel_gripper_action_controller
    onrobot_gripper_controller)
  ament_target_dependencies(
    test_parallel_gripper_action_controller controller_interface hardware_interface
    control_msgs parallel_gripper_controller rclcpp rclcpp_action rclcpp_lifecycle)
  set_tests_properties(test_parallel_gripper_action_controller PROPERTIES
    ENVIRONMENT "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")
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
