cmake_minimum_required(VERSION 2.8.3)
project(filtering_snow)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  #pcl_conversions
)

find_package(PCL 1.8 REQUIRED)

catkin_package(
	INCLUDE_DIRS include
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
)

## Declare a cpp library
add_library(${PROJECT_NAME}_core
        src/DROR.cpp
        )

add_dependencies(${PROJECT_NAME}_core
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(${PROJECT_NAME}_core
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_definitions(${PCL_DEFINITIONS})

add_executable(dynamicRadiusOutlierFilter src/dynamicRadiusOutlierFilter.cpp)
add_executable(radiusOutlierFilter src/radiusOutlierFilter.cpp)
add_executable(voxelGridFilter src/voxelGridFilter.cpp)
add_executable(statsOutlierFilter src/statsOutlierFilter.cpp)
add_executable(cropBoxFilter src/cropBoxFilter.cpp)

target_link_libraries(dynamicRadiusOutlierFilter ${catkin_LIBRARIES}
                                          ${PCL_LIBRARIES} ${PROJECT_NAME}_core)
target_link_libraries(radiusOutlierFilter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(voxelGridFilter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(statsOutlierFilter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(cropBoxFilter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
