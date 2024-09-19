cmake_minimum_required(VERSION 2.8.3)
project(gem_pure_pursuit_sim_new_interface)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  ackermann_msgs
  visualization_msgs
  gem_manager
)

catkin_package(
 CATKIN_DEPENDS
 gem_manager
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


