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

find_package(PCL 1.3 REQUIRED)
#find_package(wave REQUIRED matching containers)

catkin_package(
	#INCLUDE_DIRS include
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

#target_link_libraries(${PROJECT_NAME}
 # ${PCL_LIBRARIES}
  #wave::matching
  #wave::containers
  #)

add_definitions(${PCL_DEFINITIONS})

add_executable(radiusOutlierFilter src/radiusOutlierFilter.cpp)
add_executable(dynamicRadiusOutlierFilter src/dynamicRadiusOutlierFilter.cpp)
add_executable(voxelGridFilter src/voxelGridFilter.cpp)
add_executable(statsOutlierFilter src/statsOutlierFilter.cpp)
add_executable(cropBoxFilter2 src/cropBoxFilter2.cpp)
add_executable(restamp_tf src/restamp_tf.cpp)


target_link_libraries(radiusOutlierFilter ${catkin_LIBRARIES})
target_link_libraries(dynamicRadiusOutlierFilter ${catkin_LIBRARIES})
target_link_libraries(voxelGridFilter ${catkin_LIBRARIES})
target_link_libraries(statsOutlierFilter ${catkin_LIBRARIES})
target_link_libraries(cropBoxFilter2 ${catkin_LIBRARIES})
target_link_libraries(restamp_tf ${catkin_LIBRARIES})
