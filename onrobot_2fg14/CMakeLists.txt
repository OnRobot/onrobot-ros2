cmake_minimum_required(VERSION 3.8)
project(onrobot_2fg14)

find_package(ament_cmake REQUIRED)

set(onrobot_internal_tests "${CMAKE_CURRENT_SOURCE_DIR}/../internal/${PROJECT_NAME}/tests.cmake")
if(BUILD_TESTING AND EXISTS "${onrobot_internal_tests}")
  include("${onrobot_internal_tests}")
endif()

ament_package()

install(
  DIRECTORY
    launch
    config
    rviz
    urdf
    meshes
    srdf
  DESTINATION
    share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
