cmake_minimum_required(VERSION 3.16)
project(onrobot_gripper_bringup)

find_package(ament_cmake REQUIRED)

set(onrobot_internal_tests "${CMAKE_CURRENT_SOURCE_DIR}/../internal/${PROJECT_NAME}/tests.cmake")
if(BUILD_TESTING AND EXISTS "${onrobot_internal_tests}")
  include("${onrobot_internal_tests}")
endif()

install(DIRECTORY doc launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
