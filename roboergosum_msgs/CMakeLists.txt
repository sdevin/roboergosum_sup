cmake_minimum_required(VERSION 2.8.3)
project(roboergosum_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  actionlib
  actionlib_msgs
  toaster_msgs
  std_msgs
)

add_message_files(
   FILES
   Action.msg
   Plan.msg
   Link.msg
)

add_service_files(
  FILES
  HumanAction.srv
  String.srv
)

add_action_files(
   FILES
   ActionManager.action
)

generate_messages(
   DEPENDENCIES
   actionlib_msgs
   toaster_msgs
   std_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs message_generation message_runtime actionlib actionlib_msgs toaster_msgs
)

include_directories(
  include
  $ENV{TOASTER_DIR}/devel/include
)


