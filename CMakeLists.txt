cmake_minimum_required(VERSION 2.8.3)
project(loam_itm)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime 
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)


add_executable(iscanRegistration src/scanRegistration.cpp)
target_link_libraries(iscanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ilaserOdometry src/laserOdometry.cpp)
target_link_libraries(ilaserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(ilaserMapping src/laserMapping.cpp)
target_link_libraries(ilaserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(itm src/icpTrajectory.cpp)
target_link_libraries(itm ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})



