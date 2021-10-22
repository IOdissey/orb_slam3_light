cmake_minimum_required(VERSION 2.8)
project(orb_slam3)

IF(NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Disable some warning.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-reorder -Wno-sign-compare")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-reorder -Wno-sign-compare")

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-deprecated -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	add_definitions(-DCOMPILEDWITHC11)
	message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
	add_definitions(-DCOMPILEDWITHC0X)
	message(STATUS "Using flag -std=c++0x.")
else()
	message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.0)
if(NOT OpenCV_FOUND)
	find_package(OpenCV 3.0)
	if(NOT OpenCV_FOUND)
		message(FATAL_ERROR "OpenCV > 3.0 not found.")
	endif()
endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 REQUIRED)
add_definitions(-DEIGEN_NO_DEPRECATED_WARNING)

include_directories(
	${PROJECT_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/include/CameraModels
	${EIGEN3_INCLUDE_DIR}
	${PROJECT_SOURCE_DIR}/../thirdparty/g2o
	${PROJECT_SOURCE_DIR}/../thirdparty/DBoW2
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
	src/System.cc
	src/Tracking.cc
	src/Map.cc
	src/LocalMapping.cc
	src/LoopClosing.cc
	src/ORBextractor.cc
	src/ORBmatcher.cc
	src/Converter.cc
	src/MapPoint.cc
	src/KeyFrame.cc
	src/Atlas.cc
	src/Optimizer.cc
	src/Frame.cc
	src/KeyFrameDatabase.cc
	src/Sim3Solver.cc
	src/Initializer.cc
	src/ImuTypes.cc
	src/G2oTypes.cc
	src/CameraModels/Pinhole.cpp
	src/CameraModels/KannalaBrandt8.cpp
	src/OptimizableTypes.cpp
	src/MLPnPsolver.cpp
	src/TwoViewReconstruction.cc
	include/System.h
	include/Tracking.h
	include/Map.h
	include/LocalMapping.h
	include/LoopClosing.h
	include/ORBextractor.h
	include/ORBmatcher.h
	include/Converter.h
	include/MapPoint.h
	include/KeyFrame.h
	include/Atlas.h
	include/Optimizer.h
	include/Frame.h
	include/KeyFrameDatabase.h
	include/Sim3Solver.h
	include/Initializer.h
	include/ImuTypes.h
	include/G2oTypes.h
	include/CameraModels/GeometricCamera.h
	include/CameraModels/Pinhole.h
	include/CameraModels/KannalaBrandt8.h
	include/OptimizableTypes.h
	include/MLPnPsolver.h
	include/TwoViewReconstruction.h
	include/Config.h
)

target_link_libraries(${PROJECT_NAME}
	${OpenCV_LIBS}
	${EIGEN3_LIBS}
	${PROJECT_SOURCE_DIR}/../thirdparty/DBoW2/lib/libDBoW2.so
	${PROJECT_SOURCE_DIR}/../thirdparty/g2o/lib/libg2o.so
	-lboost_serialization
	-lcrypto
)