cmake_minimum_required(VERSION 3.8)
project(object_follower)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(relbot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(object_follower src/object_follower.cpp)
ament_target_dependencies(object_follower 
  rclcpp 
  relbot_msgs 
  geometry_msgs
)

install(TARGETS object_follower
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()