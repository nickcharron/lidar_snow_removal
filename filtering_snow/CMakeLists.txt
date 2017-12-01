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

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(radiusOutlierFilter src/radiusOutlierFilter.cpp)
add_executable(radiusOutlierFilterCustom src/radiusOutlierFilterCustom.cpp)
add_executable(voxelGridFilter src/voxelGridFilter.cpp)
add_executable(statsOutlierFilter src/statsOutlierFilter.cpp)
#add_executable(bilateralFilter src/bilateralFilter.cpp)
#add_executable(medianFilter src/medianFilter.cpp)


target_link_libraries(radiusOutlierFilter ${catkin_LIBRARIES})
target_link_libraries(radiusOutlierFilterCustom ${catkin_LIBRARIES})
target_link_libraries(voxelGridFilter ${catkin_LIBRARIES})
target_link_libraries(statsOutlierFilter ${catkin_LIBRARIES})
#target_link_libraries(bilateralFilter ${catkin_LIBRARIES})
#target_link_libraries(medianFilter ${catkin_LIBRARIES})
