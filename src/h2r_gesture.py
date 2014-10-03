#!/usr/bin/env python

import time
import sys
import roslib
roslib.load_manifest("gesture_rec")
import rospy

import tf
import math
import scipy.stats
from numpy import dot
import matplotlib
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
from std_msgs.msg import Header, String, Float32MultiArray
from tf.transformations import quaternion_inverse, quaternion_matrix

global left_arm_origin
global right_arm_origin
global head_origin
global left_arm_point
global right_arm_point
global head_point
global speech
global left_foot
global right_foot
global ground_truth
global storage
global write_speech
storage = None
ground_truth = "None"
speech = []
write_speech = []
global state_dist
state_dist = dict()
global objects
objects = []
#TEMP HACK
#objects = [("pink_box", (1.4,-0.2,-0.5)), ("purple_cylinder", (1.4, 0.05, -0.5))]
objects = [("silver_spoon", (1.7, 0.1, -0.5)),("plastic_spoon", (1.7, -0.4, -0.5)), ("metal_bowl",(1.4, -0.37, -0.47)), ("color_bowl",(1.4, 0.22,-0.47))]
global t
t = 0.005
global variance
variance = 1.0
global word_probabilities
global vocabulary
global eps
eps = 0.0001

global pub
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

#vector utilities
def norm(vec):
    total = 0.0
    for i in range(len(vec)):
        total += vec[i] * vec[i]
    return math.sqrt(total)
def sub_vec(v1,v2):
    ret = []
    for i in range(len(v1)):
        ret += [v1[i]-v2[i]]
    return tuple(ret)
def add_vec(v1,v2):
    ret = []
    for i in range(len(v1)):
        ret += [v1[i]+v2[i]]
    return tuple(ret)
def angle_between(origin, p1, p2):
    v1 = sub_vec(p1, origin)
    v2 = sub_vec(p2, origin)
    return math.acos(dot(v1, v2)/(norm(v1)* norm(v2)))


#callbacks
def speech_callback(input):
    global speech
    speech = input.data.split()
def object_callback(input):
    #process into
    # (object_id, (x,y,z))
    pass
def truth_callback(input):
    global ground_truth
    ground_truth = input.data



def is_arm_null_gesture(arm_origin, arm_point):
    if (arm_origin == None or arm_point == None):
        return True
    else:
        min_angle = 10.0 #greater than 3.14, so should always be greatest angle
        for obj in objects:
            if angle_between(arm_origin, arm_point, obj[1]) < min_angle:
                min_angle = angle_between(arm_origin, arm_point, obj[1])
        return min_angle > angle_between(arm_origin, arm_point, right_foot) or min_angle > angle_between(arm_origin, arm_point, left_foot)
def is_head_null_gesture(origin, point):
    return (origin == None or point == None)

def prob_of_sample(sample):
    return scipy.stats.norm(0.0, math.sqrt(variance)).pdf(sample)


#fills body points from openni data
def fill_points(tfl):
    try:
        global left_arm_origin
        global right_arm_origin
        global head_origin
        global head_point
        global left_arm_point
        global right_arm_point
        global left_foot
        global right_foot
        frame = "/openni_link" #"camera_link"
        (to_left_elbow,_) = tfl.lookupTransform(frame,"/left_elbow_1", rospy.Time(0))
        (to_right_elbow,_) = tfl.lookupTransform(frame,"/right_elbow_1", rospy.Time(0))
        (to_left_hand,_) = tfl.lookupTransform(frame,"/left_hand_1", rospy.Time(0))
        (to_right_hand,_) = tfl.lookupTransform(frame,"/right_hand_1", rospy.Time(0))
        (right_foot,_) = tfl.lookupTransform(frame, "/right_foot_1", rospy.Time(0))
        (left_foot,_) = tfl.lookupTransform(frame, "/left_foot_1", rospy.Time(0))
        (to_head,head_rot) = tfl.lookupTransform(frame,"/head_1", rospy.Time(0))
        left_arm_origin = to_left_hand
        left_arm_point = add_vec(to_left_hand, sub_vec(to_left_hand, to_left_elbow))
        right_arm_origin = to_right_hand
        right_arm_point = add_vec(to_right_hand, sub_vec(to_right_hand, to_right_elbow))
        head_origin = to_head
        head_temp = dot((0.0,0.0,-1.0,1.0), quaternion_matrix(quaternion_inverse(head_rot)))
        head_point = (head_temp[0] + to_head[0], head_temp[1] + to_head[1], head_temp[2] + to_head[2])
        
        #visualization for testing (verify head vector)
        # marker = Marker()
        # marker.header.frame_id = "camera_link"
        # marker.header.stamp = rospy.Time(0)
        # marker.type = marker.POINTS
        # marker.action = marker.ADD
        # marker.scale.x = 0.2
        # marker.scale.y = 0.2
        # marker.scale.z = 0.2
        # marker.color.a = 1.0
        # p1 = Point(right_arm_origin[0],right_arm_origin[1],right_arm_origin[2])
        # p2 = Point(right_arm_point[0],right_arm_point[1],right_arm_point[2])
        # p3 = Point(left_arm_origin[0],left_arm_origin[1],left_arm_origin[2])
        # p4 = Point(left_arm_point[0],left_arm_point[1],left_arm_point[2])
        # p5 = Point(head_origin[0],head_origin[1],head_origin[2])
        # p6 = Point(head_point[0],head_point[1],head_point[2])
        # marker.points += [p1, p2, p3, p4, p5, p6]
        # pub.publish(marker)

        return True
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        left_arm_point = None
        right_arm_origin = None
        left_arm_origin = None
        right_arm_point = None
        head_point = None
        head_origin = None
        left_foot = None
        right_foot = None
        return False

def baxter_init_response():
    plt.ion()
    plt.show()
def baxter_respond():
    plt.clf()
    x = []
    for word in state_dist.keys():
        x.append(word.replace('_', ' '))
    plt.bar(range(len(state_dist.keys())), state_dist.values(), align='center')
    plt.xticks(range(len(state_dist.keys())), x, size='small')
    font = {'family' : 'normal','weight' : 'bold','size'   : 25}
    matplotlib.rc('font', **font)
    plt.ylim([0,1.0])
    plt.draw()

def update_model():
    global state_dist
    global speech
    prev_dist = state_dist
    state_dist = dict()
    #if we have no previous model, set to uniform
    if len(prev_dist.keys()) == 0:
        l = len(objects) *1.0
        for obj in objects: #make sure this is a UID
            prev_dist[obj[0]] = 1.0/l
    for obj in  objects:
        obj_id = obj[0]
        state_dist[obj_id] = 0.0
        # transition update
        for prev_id in prev_dist.keys():
            if prev_id == obj_id:
                state_dist[obj_id] += (1-t)*prev_dist[prev_id]
            else:
                state_dist[obj_id] += t*prev_dist[prev_id]
        # left arm
        if not is_arm_null_gesture(left_arm_origin, left_arm_point):
            state_dist[obj_id] *= prob_of_sample(angle_between(left_arm_origin, left_arm_point, obj[1]))
        #right arm
        if not is_arm_null_gesture(right_arm_origin, right_arm_point):
            state_dist[obj_id] *= prob_of_sample(angle_between(right_arm_origin, right_arm_point, obj[1]))
        #head
        if False and not is_head_null_gesture(head_origin, head_point):
            state_dist[obj_id] *= prob_of_sample(angle_between(head_origin, head_point, obj[1]))
        #speech
        for word in speech:
            if word in vocabulary:
                state_dist[obj_id] *= word_probabilities[obj_id].get(word, eps)
    #normalize
    total = sum(state_dist.values())
    for obj in state_dist.keys():
        state_dist[obj] = state_dist[obj] / total
    global write_speech
    write_speech = speech
    speech = []

def load_dict(filename):
    global word_probabilities
    global vocabulary
    word_probabilities = dict()
    vocabulary = set()
    with open(filename) as f:
        lines = f.read().split('\n')
        for line in lines:
            words = line.split()
            word_probabilities[words[0]] = dict()
            for i in range(1, len(words)):
                word_probabilities[words[0]][words[i]] = word_probabilities[words[0]].get(words[i], 0.0) + 1.0
                vocabulary.add(words[i])
    for word in word_probabilities.keys():
        total = sum(word_probabilities[word].values())
        for x in word_probabilities[word]:
            word_probabilities[word][x] = word_probabilities[word][x]/ total



def write_output():
    global write_speech
    if storage:
        output = [head_origin, head_point, left_arm_origin, left_arm_point, \
                    right_arm_origin, right_arm_point, left_foot, right_foot,\
                    ground_truth, write_speech, objects, max(state_dist.keys(), key=lambda x: state_dist[x]), time.clock()]
        storage.write(str(output) + "\n")
    write_speech = []
    #objects
    #arms
    #head
    #speech
    #ground truth


def main():
    global speech
    rospy.init_node('h2r_gesture')
    load_dict(sys.argv[1])
    tfl = tf.TransformListener()
    rospy.Subscriber('speech_recognition', String, speech_callback, queue_size=1)
    rospy.Subscriber('objects', Float32MultiArray, object_callback, queue_size=1)
    rospy.Subscriber('current_object', String, truth_callback, queue_size=1)
    rate = rospy.Rate(30.0)
    global storage
    if len(sys.argv) > 2:
        storage = storage = open(sys.argv[2], 'w')


    global pub
    pub = rospy.Publisher("test_marker", Marker)
    marker = Marker()
    marker.header.frame_id = "openni_link" #"camera_link"
    marker.header.stamp = rospy.Time(0)
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    # depth, right left, up down
    p1 = Point(1.4, 0.22,-0.47) # color bowl
    p2 = Point(1.4, -0.37, -0.47) #metal bowl
    p3 = Point(1.7, -0.4, -0.5) #plastic spoon
    p4 = Point(1.7, 0.1, -0.5) #silver spoon
    marker.points += [p1,p2,p3,p4]
    baxter_init_response()
    while not rospy.is_shutdown():
        pub.publish(marker)
        fill_points(tfl)
        update_model()
        if not len(state_dist.keys()) == 0:
            baxter_respond()
            write_output()
        rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "Usage: rosrun gesture_rec h2r_gesture.py <language model file> <storage file (optional)>"
        sys.exit()
    main()
