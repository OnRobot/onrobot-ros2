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
find_package(onrobot_tool_api REQUIRED CONFIG)

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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_library(UTIL_LIBRARY util REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  # The resource-manager integration test loads the controller through the
  # plugin index.  A test_depend determines colcon ordering, but does not put
  # the controller's prefix in a standalone CTest environment.
  find_package(onrobot_gripper_controllers REQUIRED)
  get_filename_component(ONROBOT_GRIPPER_CONTROLLERS_PREFIX
    "${onrobot_gripper_controllers_DIR}/../../.." ABSOLUTE)
  add_launch_test(tests/test_joint_state_effort_filter.py TIMEOUT 30)
  set_tests_properties(tests_test_joint_state_effort_filter.py PROPERTIES
    ENVIRONMENT "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")
  ament_add_gtest(test_gripper_semantics tests/test_gripper_semantics.cpp)
  target_compile_features(test_gripper_semantics PRIVATE cxx_std_17)
  target_link_libraries(test_gripper_semantics onrobot_gripper_semantics)

  ament_add_gtest(test_onrobot_gripper_system tests/test_onrobot_gripper_system.cpp)
  target_compile_features(test_onrobot_gripper_system PRIVATE cxx_std_17)
  ament_target_dependencies(test_onrobot_gripper_system
    hardware_interface pluginlib rclcpp rclcpp_lifecycle)
  target_link_libraries(test_onrobot_gripper_system onrobot_gripper_hardware)
  target_link_libraries(test_onrobot_gripper_system "${UTIL_LIBRARY}")
  set_tests_properties(test_onrobot_gripper_system PROPERTIES
    ENVIRONMENT "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")

  ament_add_gtest(test_physical_read_handoff tests/test_physical_read_handoff.cpp)
  target_compile_features(test_physical_read_handoff PRIVATE cxx_std_17)
  ament_target_dependencies(test_physical_read_handoff
    hardware_interface rclcpp rclcpp_lifecycle)
  target_link_libraries(test_physical_read_handoff onrobot_gripper_hardware)
  # The test executable supplies a deterministic handoff at the public API seam;
  # the installed hardware and lifecycle callbacks execute without runtime hooks.
  target_link_options(test_physical_read_handoff PRIVATE -Wl,--export-dynamic)

  ament_add_gtest(test_resource_manager tests/test_resource_manager.cpp)
  target_compile_features(test_resource_manager PRIVATE cxx_std_17)
  ament_target_dependencies(test_resource_manager
    controller_manager control_msgs hardware_interface lifecycle_msgs
    rclcpp rclcpp_action rclcpp_lifecycle sensor_msgs)
  target_compile_definitions(test_resource_manager PRIVATE
    N02_CONTROLLER_PARAMS_FILE="${CMAKE_CURRENT_SOURCE_DIR}/tests/n02_controller_manager.yaml")
  target_link_libraries(test_resource_manager onrobot::tool_api "${UTIL_LIBRARY}")
  set_tests_properties(test_resource_manager PROPERTIES
    ENVIRONMENT "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:${ONROBOT_GRIPPER_CONTROLLERS_PREFIX}:$ENV{AMENT_PREFIX_PATH}")

  # Exercise feedback and lifecycle behavior with an isolated DDS peer.
  ament_add_gtest(test_isaac_feedback tests/test_isaac_feedback.cpp)
  target_compile_features(test_isaac_feedback PRIVATE cxx_std_17)
  ament_target_dependencies(test_isaac_feedback
    hardware_interface rclcpp rclcpp_lifecycle sensor_msgs)
  target_link_libraries(test_isaac_feedback onrobot_gripper_hardware)
  set_tests_properties(test_isaac_feedback PROPERTIES
    ENVIRONMENT "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")
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
