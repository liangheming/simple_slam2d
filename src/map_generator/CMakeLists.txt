cmake_minimum_required(VERSION 3.0.2)
project(map_generator)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  nodelet
  pluginlib
  tf
  roscpp
  sensor_msgs
  message_filters
  interfaces
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# find opencv
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_generator
#  CATKIN_DEPENDS geometry_msgs nav_msgs nodelet pluginlib roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
aux_source_directory(src/generators SRC_LIST)

add_executable(map_generator_node src/map_generator_node.cpp src/map_generator_ros.cpp ${SRC_LIST})
target_link_libraries(map_generator_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} yaml-cpp)

