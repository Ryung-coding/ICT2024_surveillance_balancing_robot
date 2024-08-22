cmake_minimum_required(VERSION 3.5)
project(controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(controller_main src/controller_main.cpp)

ament_target_dependencies(controller_main rclcpp sensor_msgs)

install(TARGETS
  controller_main
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
