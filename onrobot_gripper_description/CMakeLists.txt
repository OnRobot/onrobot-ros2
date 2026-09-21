cmake_minimum_required(VERSION 3.16)
project(onrobot_gripper_description)

find_package(ament_cmake REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_profiles SHARED
  src/gripper_profile.cpp)
target_compile_features(${PROJECT_NAME}_profiles PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_profiles PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_profiles PUBLIC yaml-cpp::yaml-cpp OpenSSL::Crypto)
set_target_properties(${PROJECT_NAME}_profiles PROPERTIES
  OUTPUT_NAME onrobot_gripper_profiles)

add_executable(resolve_gripper_profile src/resolve_gripper_profile.cpp)
target_compile_features(resolve_gripper_profile PRIVATE cxx_std_17)
target_link_libraries(resolve_gripper_profile PRIVATE ${PROJECT_NAME}_profiles)

install(TARGETS ${PROJECT_NAME}_profiles
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS resolve_gripper_profile RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenSSL yaml-cpp)

install(DIRECTORY config doc DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY examples DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

set(onrobot_internal_tests "${CMAKE_CURRENT_SOURCE_DIR}/../internal/${PROJECT_NAME}/tests.cmake")
if(BUILD_TESTING AND EXISTS "${onrobot_internal_tests}")
  include("${onrobot_internal_tests}")
endif()

ament_package()
