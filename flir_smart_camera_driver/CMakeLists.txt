cmake_minimum_required(VERSION 3.0.2)
project(flir_smart_camera_driver)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  flir_msgs
)
catkin_python_setup()
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
install(PROGRAMS
   nodes/driver_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)