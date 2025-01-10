cmake_minimum_required(VERSION 3.0.2)
project(voxel_mapping)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  nodelet
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
  tf
)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES voxel_mapping
#  CATKIN_DEPENDS geometry_msgs nav_msgs nodelet pluginlib roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
aux_source_directory(src/map_builder SRC_LIST)

add_executable(voxel_mapping_node src/voxel_mapping_node.cpp src/voxel_mapping_ros.cpp ${SRC_LIST})
target_link_libraries(voxel_mapping_node ${catkin_LIBRARIES})
