cmake_minimum_required(VERSION 2.8.3)
project(filter_point_cloud)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nodelet
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

