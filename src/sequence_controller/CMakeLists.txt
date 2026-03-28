cmake_minimum_required(VERSION 3.8)
project(sequence_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(relbot_msgs REQUIRED) 

add_executable(sequence_controller src/sequence_controller.cpp)
ament_target_dependencies(sequence_controller rclcpp relbot_msgs) 

install(TARGETS sequence_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()