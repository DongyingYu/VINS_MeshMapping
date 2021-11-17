cmake_minimum_required(VERSION 2.8.3)
project(yolov3)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(cv_bridge_DIR /usr/local/cv_bridge341/share/cv_bridge/cmake)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    camera_model
    message_filters
    )

find_package(OpenCV 3.4.10 REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3  REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(yolov3
    src/yolov3_node.cpp
    src/parameters.cpp
    )

target_link_libraries(yolov3 ${catkin_LIBRARIES} ${OpenCV_LIBS})
