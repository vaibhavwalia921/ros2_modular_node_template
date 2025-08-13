# Find test dependencies
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)

# Unit tests (no ROS dependencies)
ament_add_gtest(test_algorithm
  unit/test_algorithm.cpp
)
target_link_libraries(test_algorithm
  ${PROJECT_NAME}_core
)

ament_add_gtest(test_state_machine
  unit/test_state_machine.cpp
)
target_link_libraries(test_state_machine
  ${PROJECT_NAME}_core
)

# Integration tests (with ROS dependencies)
ament_add_gtest(test_parameter_handler
  unit/test_parameter_handler.cpp
)
target_link_libraries(test_parameter_handler
  ${PROJECT_NAME}_middleware
)
ament_target_dependencies(test_parameter_handler
  rclcpp
)

ament_add_gtest(test_lifecycle_transitions
  integration/test_lifecycle_transitions.cpp
)
target_link_libraries(test_lifecycle_transitions
  ${PROJECT_NAME}_middleware
)
ament_target_dependencies(test_lifecycle_transitions
  rclcpp
  rclcpp_lifecycle
)

ament_add_gtest(test_ros_interfaces
  integration/test_ros_interfaces.cpp
)
target_link_libraries(test_ros_interfaces
  ${PROJECT_NAME}_middleware
)
ament_target_dependencies(test_ros_interfaces
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  diagnostic_msgs
)

# Add code coverage if requested
if(CODE_COVERAGE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")
endif()