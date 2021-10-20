cmake_minimum_required(VERSION 3.1)
project(ros_orb_slam3)

if (NOT CMAKE_BUILD_TYPE)
	message(STATUS "No build type selected, default to Release")
	set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(${PROJECT_NAME} src/ros_orb_slam3.cpp)
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-deprecated)

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# DBoW2
include_directories("../../thirdparty/DBoW2")
target_link_libraries(${PROJECT_NAME} "${PROJECT_SOURCE_DIR}/../../thirdparty/DBoW2/lib/libDBoW2.so")

# g2o
include_directories("../../thirdparty/g2o")
target_link_libraries(${PROJECT_NAME} "${PROJECT_SOURCE_DIR}/../../thirdparty/g2o/lib/libg2o.so")

# orb_slam3
include_directories("../../orb_slam3/include")
target_link_libraries(${PROJECT_NAME} "${PROJECT_SOURCE_DIR}/../../orb_slam3/lib/liborb_slam3.so")
# target_link_libraries(${PROJECT_NAME} -lboost_system)

# catkin
find_package(catkin REQUIRED COMPONENTS roscpp tf image_transport sensor_msgs cv_bridge)
include_directories(${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})