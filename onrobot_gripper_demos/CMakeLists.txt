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

set(onrobot_internal_tests "${CMAKE_CURRENT_SOURCE_DIR}/../internal/${PROJECT_NAME}/tests.cmake")
if(BUILD_TESTING AND EXISTS "${onrobot_internal_tests}")
  include("${onrobot_internal_tests}")
endif()

ament_package()
