cmake_minimum_required(VERSION 2.8.3)
project(optitrack_utils)

find_package(catkin REQUIRED COMPONENTS
  dji_sdk
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
)

find_package(DJIOSDK REQUIRED)

catkin_package()
