cmake_minimum_required(VERSION 2.8.3)
project(loadply)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
set(cv_bridge_DIR /usr/local/cv_bridge341/share/cv_bridge/cmake)

find_package(PCL 1.8 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    tf
    cv_bridge
    pcl_ros
    pcl_conversions
    message_filters
    )

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3  REQUIRED)
include_directories(include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(loadply src/Load_ply.cpp
                   src/parameters.cpp)
target_link_libraries(loadply ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_INCLUDE_DIRS})
