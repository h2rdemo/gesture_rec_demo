gesture_rec_demo
================

Dependencies
------------
```
openni_tracker
NITE
ros-hydro
ros_web_video (built with --enable-vpx)
```

To Run
------
```
roscore
roslaunch openni_launch openni.launch depth_registration:=True camera:="openni"
rosrun openni_tracker openni_tracker 
rosrun web_video_server web_video_server
roslaunch gesture_rec rosbridge_ws.launch
rosrun gesture_rec h2r_gesture.py ~/Desktop/grc/src/gesture_rec_demo/src/object_lists/list1.txt
```

Notes
-----
`psi.png` was made based on a work in the public domain.


To Do
-----
 - [X] Pose for Calibration
 - [ ] Launch script
   - [ ] Including opening the browser
 - [X] Image overlay engine
