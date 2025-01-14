cmake_minimum_required(VERSION 3.0.2)
project(interfaces)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)
add_service_files(
  FILES
  StringTrigger.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)