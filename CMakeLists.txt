cmake_minimum_required(VERSION 2.8.3)
project(reactive_planner)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  nav_msgs
  fcu_common
  roscpp
  rospy
  cmake_modules
  dynamic_reconfigure
)
find_package(Eigen REQUIRED)


generate_dynamic_reconfigure_options(
  cfg/Planner.cfg
)
catkin_package()

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(reactive_planner
#   src/${PROJECT_NAME}/reactive_planner.cpp
# )

## Declare a cpp executable
add_executable(reactive_planner_node src/reactive_planner_node.cpp src/reactive_planner.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(reactive_planner_node relative_nav_msgs_generate_messages_cpp)

target_link_libraries(reactive_planner_node
  ${catkin_LIBRARIES}
)
