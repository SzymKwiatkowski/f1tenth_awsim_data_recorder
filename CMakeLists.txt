cmake_minimum_required(VERSION 3.14)
project(f1tenth_awsim_data_recorder)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/f1tenth_awsim_data_recorder/f1tenth_awsim_data_recorder.hpp
  include/f1tenth_awsim_data_recorder/visibility_control.hpp
  src/f1tenth_awsim_data_recorder.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  include/f1tenth_awsim_data_recorder/f1tenth_awsim_data_recorder_node.hpp
  src/f1tenth_awsim_data_recorder_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "f1tenth_awsim_data_recorder::F1tenthAwsimDataRecorderNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_f1tenth_awsim_data_recorder
    test/test_f1tenth_awsim_data_recorder.cpp
  )
  ament_target_dependencies(test_f1tenth_awsim_data_recorder ${TEST_SOURCES})
  target_link_libraries(test_f1tenth_awsim_data_recorder ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
