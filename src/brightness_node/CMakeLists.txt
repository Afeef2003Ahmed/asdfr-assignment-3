cmake_minimum_required(VERSION 3.8)
project(brightness_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(brightness_node src/brightness_node.cpp)
ament_target_dependencies(brightness_node rclcpp sensor_msgs std_msgs)

install(TARGETS brightness_node DESTINATION lib/${PROJECT_NAME})
ament_package()