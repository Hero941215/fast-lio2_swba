cmake_minimum_required(VERSION 3.0.2)
project(hav_mseba)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++14")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# set( CMAKE_CXX_FLAGS "-std=c++14 -O3 -DEIGEN_DONT_PARALLELIZE" )

ADD_COMPILE_OPTIONS(-std=c++14 )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions") ## 启用 -pthread 多线程
 
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz) 
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  pcl_conversions
  pcl_ros
)

# ignore warning
## add_definitions(-w)

find_package(Eigen3 3.3.3 REQUIRED)
# set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
find_package(PCL REQUIRED)
find_package(OpenCV 3.4.1 REQUIRED)
find_package(Ceres REQUIRED)

find_package(OpenMP REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(CMAKE_BUILD_TYPE Release)

#Dynamic reconfigure stuff
# generate_dynamic_reconfigure_options(cfg/Detector.cfg)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs 
  DEPENDS EIGEN3 OpenCV PCL
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /opt/intel/oneapi/mkl/latest/include/
)

link_directories(
  ${catkin_LIB_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
  /opt/intel/oneapi/mkl/latest/lib/
 )

###########
## Build ##
###########

add_library(hav_mseba SHARED
src/H_AV_MAP.cpp
src/WindowFrame.cpp
)

target_link_libraries(hav_mseba
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_executable(be_node_pose_correct src/ros_node/be_node_pose_correct.cpp)
add_dependencies(be_node_pose_correct ${catkin_EXPORTED_TARGETS})
target_compile_options(be_node_pose_correct PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(be_node_pose_correct ${catkin_LIBRARIES} ${PCL_LIBRARIES}
 ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} libmkl_rt.so hav_mseba)

