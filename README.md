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
rosrun ros_web_video ros_web_video 
roslaunch gesture_rec rosbridge_ws.launch
rosrun gesture_rec h2r_gesture.py src/ros_gesture/src/object_lists/list1.txt
```

To Do:
 - [ ] Scape `openni_launch` 