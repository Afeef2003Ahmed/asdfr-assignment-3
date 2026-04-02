cmake_minimum_required(VERSION 3.8)
project(closed_loop_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(relbot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Create executable
add_executable(closed_loop_follower src/closed_loop_follower.cpp)

# Target dependencies
ament_target_dependencies(closed_loop_follower
  rclcpp
  relbot_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

# Install executable
install(TARGETS
  closed_loop_follower
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()