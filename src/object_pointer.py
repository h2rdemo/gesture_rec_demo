#!/usr/bin/env python

import roslib
roslib.load_manifest("gesture_rec")
import rospy
from std_msgs.msg import String
import sys
import time
import random

if __name__ == "__main__":
	if len(sys.argv) != 2:
		print sys.argv
		print "Usage:\n\trosrun gesture_rec object_point.py <file name>\n\tWhere <file name> contains the objects, one per line."
		exit(0)
	with open(sys.argv[1]) as f:
		objects = f.read().split('\n')
	print objects
	rospy.init_node("object_pointer")
	pub = rospy.Publisher("current_object", String, queue_size=1)
	rate = rospy.Rate(30.0);
	start = time.time()
	obj = random.choice(objects)
	rospy.loginfo("OBJECT SET TO " + obj)
	while not rospy.is_shutdown():
		if sys.stdin.readline():
			obj = random.choice(objects)
			rospy.loginfo("OBJECT SET TO " + obj)
			start = time.time()
		if(time.time() - start >= 15.0):
			obj = random.choice(objects)
			rospy.loginfo("OBJECT SET TO " + obj)
			start = time.time()
		pub.publish(obj)