cmake_minimum_required(VERSION 2.8)
project(velodyne_color)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  std_srvs
  message_generation 
  pcl_conversions
  pcl_ros
  cv_bridge
  image_transport 
)

find_package(PCL REQUIRED)
find_package(OpenCV)
find_package(Threads)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(velodyne_color src/velodyne_color.cpp)
target_link_libraries(velodyne_color ${PCL_LIBRARIES}  ${catkin_LIBRARIES}  ${OpenCV_LIBS})

add_executable(deepImage src/deep_image.cpp)
target_link_libraries(deepImage ${PCL_LIBRARIES} ${catkin_LIBRARIES})


add_executable(velodyne_pub src/lidar_pub.cpp)
target_link_libraries(velodyne_pub ${PCL_LIBRARIES} ${catkin_LIBRARIES})





