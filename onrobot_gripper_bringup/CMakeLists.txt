cmake_minimum_required(VERSION 3.16)
project(onrobot_gripper_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_pytest_test(state_broadcaster_contract
    tests/test_state_broadcaster_contract.py)
  ament_add_pytest_test(description_quality
    tests/test_description_quality.py)
  ament_add_pytest_test(release_metadata
    tests/test_release_metadata.py)
  ament_add_pytest_test(launch_defaults
    tests/test_launch_defaults.py)
  ament_add_pytest_test(usb_rtu_guide
    tests/test_usb_rtu_guide.py)
  ament_add_pytest_test(support_guides tests/test_support_guides.py)
  add_launch_test(tests/test_standard_action_behavior.py TIMEOUT 90)
  add_launch_test(tests/test_diagnostics_launch.py TIMEOUT 180)
  add_launch_test(tests/test_2fg14_visual_state.py TIMEOUT 90)
  add_launch_test(tests/test_standard_action_stall.py TIMEOUT 90)
  add_launch_test(tests/test_dual_instance.py TIMEOUT 120)
  add_launch_test(tests/test_mixed_model_instances.py TIMEOUT 240)
  add_launch_test(tests/test_controller_startup_readiness.py TIMEOUT 240)
  ament_add_pytest_test(dependency_closure
    tests/test_dependency_closure.py)
  ament_add_pytest_test(customer_bundle
    tests/test_customer_bundle.py TIMEOUT 90)
  ament_add_pytest_test(docker_bundle
    tests/test_docker_bundle.py TIMEOUT 90)
endif()

install(DIRECTORY doc launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
