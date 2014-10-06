gesture_rec
===========
Dependencies:
openni_tracker
NITE
ros-hydro

TO RUN:
roscore
roslaunch openni_launch openni.launch depth_registration:=True camera:="openni"
rosrun openni_tracker openni_tracker 
roslaunch gesture_rec speech_launch.launch #you will need to hit allow
rosrun gesture_rec h2r_gesture.py