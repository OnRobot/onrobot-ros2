cmake_minimum_required(VERSION 3.16)
project(onrobot_gripper_isaac)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(ONROBOT_ISAAC_ASSET_REPOSITORY "" CACHE PATH
  "Directory containing the pinned non-ROS onrobot-isaac-sim repository")
set(asset_path_args --package-root "${CMAKE_CURRENT_SOURCE_DIR}")
if(ONROBOT_ISAAC_ASSET_REPOSITORY)
  list(APPEND asset_path_args --asset-repository "${ONROBOT_ISAAC_ASSET_REPOSITORY}")
endif()
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/isaac_model_contract.py"
    ${asset_path_args}
  RESULT_VARIABLE asset_path_result OUTPUT_VARIABLE asset_repository
  OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_VARIABLE asset_path_error)
if(NOT asset_path_result EQUAL 0)
  message(FATAL_ERROR "${asset_path_error}")
endif()
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${asset_repository}/tools/validate_assets.py"
    --manifest-only --root "${asset_repository}"
  RESULT_VARIABLE asset_check_result ERROR_VARIABLE asset_check_error)
if(NOT asset_check_result EQUAL 0)
  message(FATAL_ERROR "Isaac asset integrity check failed: ${asset_check_error}")
endif()
install(DIRECTORY "${asset_repository}/assets" DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

install(
  DIRECTORY config doc resource
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS
    scripts/author_2fg14_asset.py
    scripts/author_rg6_asset.py
    scripts/run_articulation_test.py
    scripts/run_hardware_in_loop.py
    scripts/run_ros_bridge.py
    scripts/run_kinematic_qualification.py
    scripts/run_drive_step_response.py
    scripts/run_2fg_dynamic_contact_test.py
    scripts/run_physical_motion_showcase.py
    scripts/run_workpiece_showcase.py
    scripts/run_physical_motion_showcase_matrix.py
    scripts/run_payload_retention_matrix.py
    scripts/run_payload_retention_test.py
    scripts/run_synthetic_grip_learning.py
    scripts/run_rg2_tip_contact_test.py
    scripts/validate_asset_contract.py
    scripts/validate_physical_motion_results.py
    scripts/validate_payload_retention_results.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    scripts/asset_visual_materials.py
    scripts/isaac_model_contract.py
    scripts/grip_drive_control.py
    scripts/isaac_runtime_compat.py
    scripts/showcase_branding.py
    scripts/synthetic_grip_learning.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

set(onrobot_internal_tests "${CMAKE_CURRENT_SOURCE_DIR}/../internal/${PROJECT_NAME}/tests.cmake")
if(BUILD_TESTING AND EXISTS "${onrobot_internal_tests}")
  include("${onrobot_internal_tests}")
endif()

ament_package()
