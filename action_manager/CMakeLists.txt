cmake_minimum_required(VERSION 2.8.3)
project(action_manager)

find_package(PkgConfig)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  rospy
  std_msgs
  roboergosum_msgs
  gtp_ros_msg
)

pkg_check_modules(pr2motion-genom3 REQUIRED pr2motion-genom3)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs roscpp rospy std_msgs roboergosum_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${pr2motion-genom3_INCLUDE_DIRS}
  $ENV{ROBOTPKG_BASE}/include
)

link_directories(${pr2motion-genom3_LIBRARY_DIRS})

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp src/Actions/*.cpp)

add_executable(action_manager ${SOURCES})
target_link_libraries( 
  action_manager
  ${catkin_LIBRARIES}
  ${pr2motion-genom3_LIBRARIES}
)
add_dependencies(action_manager roboergosum_msgs_generate_messages_cpp)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_action_executor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
