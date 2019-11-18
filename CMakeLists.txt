cmake_minimum_required(VERSION 2.8.3)
project(turtle_walker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  move_base_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtle_walker
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(walker src/main.cpp src/walker.cpp)
target_link_libraries(walker ${catkin_LIBRARIES})
