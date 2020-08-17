cmake_minimum_required(VERSION 3.0.2)
project(smc_g2_operator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  std_srvs
  smc_g2_io
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES smc_g2_operator
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(smc_g2_operator
  src/smc_g2_operator.cpp
)
target_link_libraries(smc_g2_operator
  ${catkin_LIBRARIES}
)

add_executable(operator_node src/operator_node.cpp src/smc_g2_operator.cpp)
target_link_libraries(operator_node ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############
