cmake_minimum_required(VERSION 3.5)
project(grodom_ros1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_BUILD_TYPE "Release")

set(BUILD_LIB TRUE)

if(BUILD_LIB)
  add_subdirectory(slam_core)
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf
    rosbag
)

find_package(Eigen3)
find_package(PCL REQUIRED)

include_directories(
slam_core
ros1_adapter
${catkin_INCLUDE_DIRS} 
${EIGEN3_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
)

if(BUILD_LIB) 
  set(LIDAR_SLAM_LIB slam_lib)
else()
  if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    message("CMAKE_SYSTEM_PROCESSOR MATCHES x86_64")
    file(GLOB LIDAR_SLAM_LIB ${PROJECT_SOURCE_DIR}/slam_core/lib/x86/libslam_lib.so)
  elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64") 
    message("CMAKE_SYSTEM_PROCESSOR MATCHES aarch64")
    file(GLOB LIDAR_SLAM_LIB ${PROJECT_SOURCE_DIR}/slam_core/lib/arm/libslam_lib.so)
  endif ()
endif()


catkin_package()

add_executable(grodom_ros1 
   ros1_adapter/grslam_ros1.cpp
)

target_link_libraries(grodom_ros1
   ${catkin_LIBRARIES}
   ${LIDAR_SLAM_LIB}
   ${Boost_SYSTEM_LIBRARY}
   ${PCL_LIBRARIES}
   -lglog
)
