cmake_minimum_required(VERSION 2.8.3)
project(planners_manager)

find_package(catkin REQUIRED COMPONENTS
  toaster_msgs 
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  roboergosum_msgs
  BP_experiment
)

catkin_package(
   CATKIN_DEPENDS toaster_msgs roscpp rospy std_msgs roboergosum_msgs BP_experiment geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  $ENV{TOASTER_DIR}/devel/include
)

add_executable(planners_manager src/main.cpp src/planners_manager.cpp)
target_link_libraries( 
  planners_manager
  ${catkin_LIBRARIES}
)
add_dependencies(planners_manager roboergosum_msgs_generate_messages_cpp)
