cmake_minimum_required(VERSION 2.8.3)
project(lab3)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  std_msgs
)
catkin_package(
)
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(puppet_hand src/puppetHand.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(puppet_hand ${catkin_LIBRARIES})
