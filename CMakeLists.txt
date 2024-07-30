cmake_minimum_required(VERSION 3.0.2)
project(livox_converter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  livox_ros_driver
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros livox_ros_driver
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(livox_converter_node src/livox_converter_node.cpp)
target_link_libraries(livox_converter_node ${catkin_LIBRARIES})

install(TARGETS livox_converter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)