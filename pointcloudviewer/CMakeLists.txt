cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(point_cloud_viewer) 

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(PCL 1.5 REQUIRED)

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(COMMON_SOURCES
	src/data_model.cpp
)

add_executable(pointXYZIRNLRGB_cloud_viewer_main
	src/pointXYZIRNLRGB_cloud_viewer_main.cpp
	${COMMON_SOURCES}
)
target_link_libraries(pointXYZIRNLRGB_cloud_viewer_main
	${OPENGL_LIBRARIES}
	${GLUT_LIBRARIES}
	${PCL_LIBRARIES}
)




