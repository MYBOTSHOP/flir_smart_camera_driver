cmake_minimum_required(VERSION 3.5)
project(flir_smart_camera_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(flir_msgs REQUIRED)

include_directories(
  ${PROJECT_NAME}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${flir_msgs_INCLUDE_DIRS}
)

add_executable(driver_node nodes/driver_node)
ament_target_dependencies(driver_node
  rclcpp
  std_msgs
  flir_msgs
)

install(TARGETS driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
