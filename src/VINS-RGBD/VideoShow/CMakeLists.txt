cmake_minimum_required(VERSION 2.8.3)
project(videoshow)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  image_transport 
  cv_bridge
)

find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

# add the publisher example  

add_executable(image_subscriber 
videoshow/image_subscriber.cpp
)  
target_link_libraries(image_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})  

add_executable(image_publisher 
videoshow/image_publisher.cpp
)  
target_link_libraries(image_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})  