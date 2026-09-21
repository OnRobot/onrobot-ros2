cmake_minimum_required(VERSION 3.16)
project(onrobot_gripper_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(onrobot_gripper_msgs REQUIRED)
find_package(onrobot_gripper_description REQUIRED)

add_executable(physical_gripper_adapter src/physical_gripper_adapter.cpp)
target_compile_features(physical_gripper_adapter PRIVATE cxx_std_17)
ament_target_dependencies(physical_gripper_adapter rclcpp rclcpp_action
  control_msgs sensor_msgs onrobot_gripper_msgs onrobot_gripper_description)
target_link_libraries(physical_gripper_adapter
  onrobot_gripper_description::onrobot_gripper_description_profiles)
install(TARGETS physical_gripper_adapter RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config doc launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/moveit_gripper_smoke.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME moveit_gripper_smoke)

set(onrobot_internal_tests "${CMAKE_CURRENT_SOURCE_DIR}/../internal/${PROJECT_NAME}/tests.cmake")
if(BUILD_TESTING AND EXISTS "${onrobot_internal_tests}")
  include("${onrobot_internal_tests}")
endif()

ament_package()
