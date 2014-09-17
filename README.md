gesture_rec
===========
If you want speech recognition to work you need to set it up to run on localhost/ros_speech

Playback bag:
rosparam set /use_sim_time true
roslaunch openni_launch openni.launch device_id:=invalid camera:=tracker_camera
roslaunch openni_launch openni.launch device_id:=invalid camera:=ork_camera
(this will generate point clouds from playback data)
rosbag play --clock my_stuff.bag



TO RUN:
roscore
roslaunch openni_launch openni.launch depth_registration:=True camera:="openni"
rosrun openni_tracker openni_tracker
roslaunch gesture_rec speech_launch.launch #you will need to hit allow
rosrun gesture_rec h2r_gesture.py