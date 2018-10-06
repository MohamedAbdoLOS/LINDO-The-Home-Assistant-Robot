cmake_minimum_required(VERSION 2.8.3)
project(lindorobot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp 
    rospy 
    tf
    nav_msgs 
    lindo_msgs 
    geometry_msgs 
    sensor_msgs 
    std_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES lindorobot
  CATKIN_DEPENDS roscpp rospy tf nav_msgs lindo_msgs geometry_msgs sensor_msgs std_msgs 
  DEPENDS system_lib
)
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lindo_base_node src/lindo_base_node.cpp)
target_link_libraries(lindo_base_node ${catkin_LIBRARIES})
