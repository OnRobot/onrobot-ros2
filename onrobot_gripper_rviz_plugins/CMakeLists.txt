cmake_minimum_required(VERSION 3.8)
project(onrobot_gripper_rviz_plugins)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(onrobot_gripper_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Test Widgets)

add_library(onrobot_gripper_rviz SHARED
  include/onrobot_gripper_rviz_plugins/branded_panel.hpp
  include/onrobot_gripper_rviz_plugins/gripper_control_panel.hpp
  include/onrobot_gripper_rviz_plugins/force_history_model.hpp
  include/onrobot_gripper_rviz_plugins/force_history_panel.hpp
  include/onrobot_gripper_rviz_plugins/realtime_control_panel.hpp
  include/onrobot_gripper_rviz_plugins/panel_settings.hpp
  include/onrobot_gripper_rviz_plugins/three_finger_control_panel.hpp
  resource/onrobot_brand.qrc
  src/branded_panel.cpp src/gripper_control_panel.cpp
  src/force_history_panel.cpp
  src/realtime_control_panel.cpp src/panel_settings.cpp
  src/three_finger_control_panel.cpp)
target_compile_features(onrobot_gripper_rviz PUBLIC cxx_std_17)
target_include_directories(onrobot_gripper_rviz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(onrobot_gripper_rviz control_msgs controller_manager_msgs
  onrobot_gripper_msgs pluginlib rcl_interfaces rclcpp rclcpp_action rviz_common sensor_msgs std_msgs)
target_link_libraries(onrobot_gripper_rviz Qt5::Widgets)

add_executable(gripper_effort_marker src/gripper_effort_marker.cpp)
ament_target_dependencies(gripper_effort_marker geometry_msgs rclcpp sensor_msgs visualization_msgs)

pluginlib_export_plugin_description_file(rviz_common onrobot_gripper_rviz.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_realtime_panel_semantics tests/test_realtime_panel_semantics.cpp)
  target_compile_features(test_realtime_panel_semantics PRIVATE cxx_std_17)
  target_include_directories(test_realtime_panel_semantics PRIVATE include)
  ament_add_gtest(test_gripper_control_panel tests/test_gripper_control_panel.cpp)
  target_compile_features(test_gripper_control_panel PRIVATE cxx_std_17)
  target_link_libraries(test_gripper_control_panel onrobot_gripper_rviz)
  ament_add_gtest(test_force_history_model tests/test_force_history_model.cpp)
  target_compile_features(test_force_history_model PRIVATE cxx_std_17)
  target_include_directories(test_force_history_model PRIVATE include)
  ament_add_gtest(test_panel_configuration tests/test_panel_configuration.cpp)
  target_compile_features(test_panel_configuration PRIVATE cxx_std_17)
  target_link_libraries(test_panel_configuration onrobot_gripper_rviz Qt5::Test)
  set_tests_properties(test_panel_configuration PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
  ament_add_gtest_executable(test_panel_integration tests/test_panel_integration.cpp)
  target_compile_features(test_panel_integration PRIVATE cxx_std_17)
  ament_target_dependencies(test_panel_integration control_msgs
    onrobot_gripper_msgs rcl_interfaces rclcpp rviz_common)
  target_link_libraries(test_panel_integration onrobot_gripper_rviz Qt5::Test)
  foreach(panel_model 2fg7 2fg14 rg2 rg6)
    ament_add_gtest_test(test_panel_integration
      TEST_NAME panel_integration_${panel_model}
      TIMEOUT 180
      ENV
        "QT_QPA_PLATFORM=offscreen"
        "OPENBLAS_NUM_THREADS=1"
        "OMP_NUM_THREADS=1"
        "GTEST_FILTER=PanelIntegrationFixture.ActualQtInputMovesEveryFakeModelThroughControllers"
        "ONROBOT_PANEL_TEST_ONLY_MODEL=${panel_model}"
        "ONROBOT_PANEL_TEST_EVIDENCE_DIR=${CMAKE_CURRENT_BINARY_DIR}/panel-test-evidence/${panel_model}")
  endforeach()
  ament_add_gtest_test(test_panel_integration
    TEST_NAME panel_integration_unit
    TIMEOUT 60
    ENV
      "QT_QPA_PLATFORM=offscreen"
      "OPENBLAS_NUM_THREADS=1"
      "OMP_NUM_THREADS=1"
      "GTEST_FILTER=-PanelIntegrationFixture.ActualQtInputMovesEveryFakeModelThroughControllers:PanelIntegrationFixture.ThreeFingerLivePanelClickAndCapture")
  ament_add_gtest(test_rviz_plugin tests/test_rviz_plugin.cpp)
  ament_target_dependencies(test_rviz_plugin pluginlib rviz_common)
  set_tests_properties(test_rviz_plugin PROPERTIES
    ENVIRONMENT "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")
endif()

install(TARGETS onrobot_gripper_rviz gripper_effort_marker
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES onrobot_gripper_rviz.xml README.md
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
ament_package()
