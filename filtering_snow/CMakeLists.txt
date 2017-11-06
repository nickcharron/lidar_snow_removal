cmake_minimum_required(VERSION 2.8.3)
project(filtering_snow)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(radiusOutlierFilter src/radiusOutlierRemoval.cpp)
add_executable(voxelGridFilter src/voxelGridFilter.cpp)

#target_link_libraries(radiusOutlierFilter ${catkin_LIBRARIES})
target_link_libraries(voxelGridFilter ${catkin_LIBRARIES})
