cmake_minimum_required(VERSION 3.8)
project(onrobot_gripper_demos)

find_package(ament_cmake REQUIRED)

install(DIRECTORY doc launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/measure_realtime_ros_path.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME measure_realtime_ros_path)
install(PROGRAMS scripts/record_gripper_session.py
  DESTINATION lib/${PROJECT_NAME} RENAME record_gripper_session)
install(PROGRAMS scripts/replay_gripper_session.py
  DESTINATION lib/${PROJECT_NAME} RENAME replay_gripper_session)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_showcase_launch test/test_showcase_launch.py)
  ament_add_pytest_test(
    test_realtime_ros_path test/test_realtime_ros_path.py)
  ament_add_pytest_test(
    test_session_tools test/test_session_tools.py)
endif()

ament_package()
