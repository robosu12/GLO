cmake_minimum_required(VERSION 3.5)
project(grslam_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()

set(CMAKE_BUILD_TYPE Release)

set(BUILD_LIB TRUE)
# set(BUILD_LIB FALSE)

if(BUILD_LIB)
  add_subdirectory(slam_core)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(Eigen3)
find_package(PCL REQUIRED QUIET)
find_package(visualization_msgs REQUIRED)

include_directories(
  slam_core
  ros2_adapter
	${PCL_INCLUDE_DIRS}
  ${rtt_executor_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
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



#############
#executable##
#############
add_executable(grslam_ros2 
   ros2_adapter/grslam_ros2.cpp
   ros2_adapter/load_param_ros2.cpp
)

ament_target_dependencies(grslam_ros2
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "std_msgs"
  "sensor_msgs"
  "visualization_msgs"
  "tf2"
  "tf2_ros"
  "tf2_sensor_msgs"
  "tf2_msgs"
  Boost
  )

target_link_libraries(grslam_ros2
   ${LIDAR_SLAM_LIB}
   ${Boost_SYSTEM_LIBRARY}
   ${PCL_LIBRARIES}
   -lglog
)


#############
## Install ##
#############
install(TARGETS
grslam_ros2 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS grslam_ros2
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY params/
  DESTINATION params/)


if(BUILD_LIB)
else()
  message("Precompiled slamcore lib will be installed")
  if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    install(
      FILES  slam_core/lib/x86/libslam_lib.so
      DESTINATION lib)
  elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64") 
    install(
      FILES  slam_core/lib/arm/libslam_lib.so
      DESTINATION lib)
  endif ()

endif()


# install(
#   DIRECTORY config/
#   DESTINATION config/)

ament_export_libraries(slam_lib)
  
#############
## Testing ##
#############
ament_package()
