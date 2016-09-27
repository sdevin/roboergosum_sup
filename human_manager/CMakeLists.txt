cmake_minimum_required(VERSION 2.8.3)
project(human_manager)

find_package(catkin REQUIRED COMPONENTS
  toaster_msgs 
  message_generation
  roscpp
  rospy
  std_msgs
  roboergosum_msgs
)

catkin_package(
   CATKIN_DEPENDS toaster_msgs roscpp rospy std_msgs roboergosum_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  $ENV{TOASTER_DIR}/devel/include
)

add_executable(human_manager src/main.cpp src/human_manager.cpp)
target_link_libraries( 
  human_manager
  ${catkin_LIBRARIES}
)
add_dependencies(human_manager roboergosum_msgs_generate_messages_cpp)
