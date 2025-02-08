
cmake_minimum_required(VERSION 3.5)
project(grslam_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic )
# endif()

set(CMAKE_BUILD_TYPE Release)

# set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
message("Start build slamcore lib")

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(yaml-cpp REQUIRED)

include_directories(
  slam_core
  ${PROJECT_SOURCE_DIR}
	${PCL_INCLUDE_DIRS}
  ${Ceres_INCLUDE_DIRS}
  yaml-cpp
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${Ceres_LIBRARY_DIRS}
)

add_library(slam_lib SHARED
            common/config/system_config.cpp
            common/data_struct/keyframe.cpp
            frontend/feature_extractor.cpp
            load_params/load_params.cpp
          
            lidar_odom/lidar_odom.cpp
            map_track/map_track.cpp

            system/system.cpp
            )

#############
###library###
#############

target_link_libraries(slam_lib
   ${Boost_SYSTEM_LIBRARY}
   ${PCL_LIBRARIES}
   ${CERES_LIBRARIES}
   -lglog
   yaml-cpp
)



#############
## Install ##
#############

install(
  TARGETS  slam_lib 
  DESTINATION lib)


#############
##  test   ##
#############

# if(BUILD_TESTING)

#   add_executable(test_state_machine state_machine/test_state_machine.cpp 
#                                     state_machine/slam_state_machine.cpp 
#                                     state_machine/stateMachine.cpp)
#   target_link_libraries(test_state_machine -lglog)

# endif()