cmake_minimum_required(VERSION 2.8)
project(orb_slam3_build)

add_subdirectory(thirdparty/g2o)
add_subdirectory(thirdparty/DBoW2)
add_subdirectory(orb_slam3)
add_subdirectory(ros/ros_orb_slam3)