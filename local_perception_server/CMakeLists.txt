cmake_minimum_required(VERSION 3.0.2)
project(local_perception_server)

## Compile as C++14 = PCL 1.11
## Compile as C++11 = PCL 1.81
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  actionlib
  local_perception_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED) #PCL from system or from ros_ws/ros_ws_workchain
#find_package(PCL 1.8 EXACT REQUIRED PATHS /usr/local/share/pcl-1.8)
#find_package(PCL 1.7.2 EXACT REQUIRED PATHS /usr/lib/x86_64-linux-gnu/cmake/pcl)
find_package(Eigen3 REQUIRED) # To matrix operations

## Headers
file(GLOB PROGRAM_HEADERS RELATIVE ${PROJECT_SOURCE_DIR} "include/${PROJECT_NAME}/*.h")

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        #${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(common
            src/common/verbosity_levels.cpp
            src/common/pcl_complement.cpp
            )

add_library(${PROJECT_NAME}
            src/local_perception_server.cpp
            )

# Set dependencies of the library
target_link_libraries(common
                      ${PCL_LIBRARIES}
                      ${catkin_LIBRARIES}
                      )

target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                      ${PCL_LIBRARIES}
                      #common
                      )

## Define a executable
add_executable(${PROJECT_NAME}_node
                src/${PROJECT_NAME}_node.cpp)

## Define the executable dependencies
target_link_libraries(${PROJECT_NAME}_node
                      common
                      ${catkin_LIBRARIES}
                      #${OpenCV_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${PROJECT_NAME})
