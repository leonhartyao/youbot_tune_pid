cmake_minimum_required(VERSION 2.8.3)
project(youbot_pid)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  brics_actuator
  geometry_msgs
)

 CATKIN_DEPENDS 
 moveit_core 
 moveit_ros_planning_interface
 geometry_msgs
 brics_actuator
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_executable(pid_tune src/tune_pid_gains.cpp)

target_link_libraries(pid_tune
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

