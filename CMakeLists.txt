cmake_minimum_required(VERSION 2.8.3)
project(gesture_rec)

find_package(catkin
  REQUIRED COMPONENTS
  eigen
  roscpp
  rospy
  openni_tracker
  tf
  visualization_msgs
  geometry_msgs
)
include_directories(${Eigen_INCLUDE_DIRS})


catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

add_executable(gesture src/gesture.cpp)

target_link_libraries(
	gesture
	${catkin_LIBRARIES}
)
