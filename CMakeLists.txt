cmake_minimum_required(VERSION 3.0.2)
project(evaluation_nodes)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  gazebo_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
)

# Generate the python library
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  Count.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fre_counter
  CATKIN_DEPENDS roscpp urdf gazebo_msgs geometry_msgs std_msgs nav_msgs geometry_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(fre_counter
  src/fre_counter.cpp
)

add_dependencies(fre_counter ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(fre_counter
   ${catkin_LIBRARIES}
)

add_executable(fre_counter_node src/fre_counter_node.cpp)

#############
## Install ##
#############

target_link_libraries(fre_counter_node
   ${catkin_LIBRARIES}
   fre_counter
)

install(PROGRAMS scripts/evaluation_nodes
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
