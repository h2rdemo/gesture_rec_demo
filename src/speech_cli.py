#!/usr/bin/env python

import roslib
roslib.load_manifest("gesture_rec")
import rospy
from std_msgs.msg import String
import sys

if __name__ == "__main__":
    rospy.init_node("speech_command_line")
    pub = rospy.Publisher("speech_recognition", String, queue_size=1)
    while rospy.is_shutdown():
        speech=sys.stdin.readline()
        if speech:
            pub.publish(speech)