cmake_minimum_required(VERSION 3.8)
project(template_node)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lifecycle_msgs REQUIRED)


# Include directories
include_directories(include)

# Core library (no ROS dependencies)
add_library(${PROJECT_NAME}_core
  src/core/state_machine.cpp
  src/algorithms/example_algorithm.cpp
)

# Middleware library
add_library(${PROJECT_NAME}_middleware
  src/middleware/lifecycle_node_wrapper.cpp
  src/middleware/parameter_handler.cpp
  src/middleware/ros_interfaces.cpp
)

target_link_libraries(${PROJECT_NAME}_middleware
  ${PROJECT_NAME}_core
)

ament_target_dependencies(${PROJECT_NAME}_middleware
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  rclcpp_components
  std_msgs
  geometry_msgs
  sensor_msgs
  example_interfaces
  rcl_interfaces
  nav_msgs          
  diagnostic_msgs 
  std_srvs 
)

# Register component
rclcpp_components_register_node(${PROJECT_NAME}_middleware
  PLUGIN "template_node::middleware::LifecycleNodeWrapper"
  EXECUTABLE ${PROJECT_NAME}_node
)

# Main executable
add_executable(${PROJECT_NAME}_main src/main.cpp)
target_link_libraries(${PROJECT_NAME}_main
  ${PROJECT_NAME}_middleware
)

# Install
install(TARGETS
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_middleware
  ${PROJECT_NAME}_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
  
  # Unit tests
  ament_add_gtest(test_algorithm
    test/unit/test_algorithm.cpp
  )
  target_link_libraries(test_algorithm
    ${PROJECT_NAME}_core
  )
  
  ament_add_gtest(test_state_machine
    test/unit/test_state_machine.cpp
  )
  target_link_libraries(test_state_machine
    ${PROJECT_NAME}_core
  )
  
  # Integration tests
  ament_add_gtest(test_lifecycle_transitions
    test/integration/test_lifecycle_transitions.cpp
  )
  target_link_libraries(test_lifecycle_transitions
    ${PROJECT_NAME}_middleware
  )
  ament_target_dependencies(test_lifecycle_transitions
    rclcpp
    rclcpp_lifecycle
  )
endif()

ament_package()