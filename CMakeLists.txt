cmake_minimum_required(VERSION 3.0.2)
project(shared_autonomy)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  trajectory_msgs
  dynamic_reconfigure
)

catkin_package(
  INCLUDE_DIRS include
  lib
  LIBRARIES turtle_control
  cvxgen_turtle
)

include_directories(
  include
  lib
  ${catkin_INCLUDE_DIRS}
)

cs_add_library(turtle_control
  src/turtle_control.cpp
)

cs_add_library(cvxgen_turtle 
  lib/ldl.c
  lib/solver.c
  lib/matrix_support.c
  lib/util.c
  lib/variables_definition.c
)

target_link_libraries(turtle_control cvxgen_turtle)

cs_add_executable(turtle_control_node
  src/turtle_control.cpp
  src/turtle_control_node.cpp
)

target_link_libraries(turtle_control_node
  turtle_control
  cvxgen_turtle
  ${catkin_LIBRARIES}
)

cs_install()
cs_export()