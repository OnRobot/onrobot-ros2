cmake_minimum_required(VERSION 3.8)
project(onrobot_3fg25)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Asset files do not carry per-file copyright banners.
  set(ament_cmake_copyright_FOUND TRUE)
  # This package contains no C or C++ source files.
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(
  DIRECTORY
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
