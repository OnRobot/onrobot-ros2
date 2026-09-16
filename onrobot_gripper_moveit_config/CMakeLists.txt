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

if(BUILD_TESTING)
  find_package(moveit_core REQUIRED)
  find_package(srdfdom REQUIRED)
  find_package(urdf REQUIRED)
  add_executable(probe_planning_geometry tests/probe_planning_geometry.cpp)
  target_compile_features(probe_planning_geometry PRIVATE cxx_std_17)
  ament_target_dependencies(probe_planning_geometry moveit_core srdfdom urdf onrobot_gripper_description)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(planning_geometry tests/test_planning_geometry.py TIMEOUT 90
    APPEND_ENV "PLANNING_GEOMETRY_PROBE=$<TARGET_FILE:probe_planning_geometry>")
  ament_add_pytest_test(physical_adapter tests/test_physical_adapter.py TIMEOUT 180)
  ament_add_pytest_test(
    moveit_contract
    tests/test_moveit_contract.py
    TIMEOUT 30
  )
  ament_add_pytest_test(
    moveit_execution
    tests/test_moveit_execution.py
    TIMEOUT 480
  )
endif()

ament_package()
