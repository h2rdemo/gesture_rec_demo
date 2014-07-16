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
  object_recognition_msgs
  pcl_ros
  sensor_msgs
)
include_directories(${Eigen_INCLUDE_DIRS} ${pcl_ros_INCLUDE_DIRS})


catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

add_executable(gesture src/h2r_gesture.cpp)
add_executable(rec_obj_relay src/rec_obj_relay.cpp)

target_link_libraries(
	gesture
	${catkin_LIBRARIES}
)
target_link_libraries(
  rec_obj_relay
  ${catkin_LIBRARIES}
)

