cmake_minimum_required(VERSION 3.0.2)
project(localizer)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS
  message_filters
  nav_msgs
  nodelet
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2_ros
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES localizer
#  CATKIN_DEPENDS message_filters nav_msgs nodelet pluginlib roscpp sensor_msgs std_msgs tf tf2_ros
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
aux_source_directory(src/icps SRC_LIST)

add_executable(localizer_node src/localizer_node.cpp src/localizer_ros.cpp ${SRC_LIST})
target_link_libraries(localizer_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})