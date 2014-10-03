import random
import math
import numpy as np
import sys
import scipy.stats
from numpy import dot
global word_probabilities
global vocabulary
t = 0.005
variance = 1.0
global head_belief
global arm_belief
global speech_belief
global multimodal_belief
global everything_belief
head_belief = dict()
speech_belief = dict()
arm_belief = dict()
multimodal_belief = dict()
everything_belief = dict() # including head (i think this will be bad)
eps = 0.0001
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
def prob_of_sample(sample):
    return scipy.stats.norm(0.0, math.sqrt(variance)).pdf(sample)
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
def is_arm_null_gesture(arm_origin, arm_point, left_foot, right_foot, objects):
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
def update_model(data):
    global head_belief
    global arm_belief
    global speech_belief
    global multimodal_belief
    global everything_belief
    head_origin = data[0]
    head_point = data[1]
    left_arm_origin = data[2]
    left_arm_point = data[3]
    right_arm_origin = data[4]
    right_arm_point = data[5]
    left_foot = data[6]
    right_foot = data[7]
    ground_truth = data[8]
    speech = data[9]
    objects = data[10]
    new_head_belief = dict()
    new_speech_belief = dict()
    new_arm_belief = dict()
    new_multimodal_belief = dict()
    new_everything_belief = dict()
    for i in range(len(objects)):
        curr_object = objects[i]
        object_name = curr_object[0]
        object_point = curr_object[1]
        #init new values
        new_head_belief[object_name] = 0.0
        new_arm_belief[object_name] = 0.0
        new_multimodal_belief[object_name] = 0.0
        new_speech_belief[object_name] = 0.0
        new_everything_belief[object_name] = 0.0
        #transition probabilities
        for j in range(len(objects)): #assumes number of objects does not change
            temp_object = objects[j]
            temp_object_name = temp_object[0]
            if i == j: trans_prob = 1 - t
            else: trans_prob = t
            new_head_belief[object_name] += trans_prob * head_belief[temp_object_name]
            new_arm_belief[object_name] += trans_prob * arm_belief[temp_object_name]
            new_multimodal_belief[object_name] += trans_prob * multimodal_belief[temp_object_name]
            new_speech_belief[object_name] += trans_prob * speech_belief[temp_object_name]
            new_everything_belief[object_name] += trans_prob * everything_belief[temp_object_name]
        #check the head
        if not is_head_null_gesture(head_origin, head_point):
            head_prob = prob_of_sample(angle_between(head_origin, head_point, object_point))
            new_head_belief[object_name] *= head_prob
            new_everything_belief[object_name] *= head_prob
        #check the arms
        if not is_arm_null_gesture(left_arm_origin, left_arm_point, left_foot, right_foot, objects):
            left_arm_prob = prob_of_sample(angle_between(left_arm_origin, left_arm_point, object_point))
            new_arm_belief[object_name] *= left_arm_prob
            new_everything_belief[object_name] *= left_arm_prob
            new_multimodal_belief[object_name] *= left_arm_prob
        if not is_arm_null_gesture(right_arm_origin, right_arm_point, left_foot, right_foot, objects):
            right_arm_prob = prob_of_sample(angle_between(right_arm_origin, right_arm_point, object_point))
            new_arm_belief[object_name] *= right_arm_prob
            new_everything_belief[object_name] *= right_arm_prob
            new_multimodal_belief[object_name] *= right_arm_prob
        #check the speech
        for word in speech:
            if word in vocabulary:
                new_everything_belief[object_name] *= word_probabilities[object_name].get(word, eps)
                new_multimodal_belief[object_name] *= word_probabilities[object_name].get(word, eps)
                new_speech_belief[object_name] *= word_probabilities[object_name].get(word, eps)
    for d in [new_speech_belief, new_head_belief, new_multimodal_belief, new_arm_belief, new_everything_belief]:
        total = sum(d.values())
        for obj in d.keys():
            d[obj] = d[obj] / total
    head_belief = new_head_belief
    speech_belief = new_speech_belief
    arm_belief = new_arm_belief
    multimodal_belief = new_multimodal_belief
    everything_belief = new_everything_belief
    return speech_belief, head_belief, arm_belief, multimodal_belief, everything_belief


def main():
    speech_total = 0
    head_total = 0
    arm_total = 0
    multimodal_total = 0
    everything_total = 0
    total = 0
    speech_list =[]
    head_list = []
    arm_list = []
    multi_list = []
    every_list = []
    real_list = []
    load_dict(sys.argv[1])
    with open(sys.argv[2]) as f:
        x = f.readline()
        objects = eval(x)[10]
        for obj in objects: #initialize object dist
            head_belief[obj[0]] = 1.0/len(objects)
            speech_belief[obj[0]] = 1.0/len(objects)
            arm_belief[obj[0]] = 1.0/len(objects)
            multimodal_belief[obj[0]] = 1.0/len(objects)
            everything_belief[obj[0]] = 1.0/len(objects)
        while(not (x == "" or x == "\n")):
            data = eval(x)
            ground_truth = data[8]
            sb, hb, ab, mb, eb = update_model(data)
            if not ground_truth == "None":
                real_list.append(ground_truth)
                head_list.append(max(hb.keys(), key=lambda x: hb[x]))
                arm_list.append(max(ab.keys(), key=lambda x: ab[x]))
                multi_list.append(max(mb.keys(), key=lambda x: mb[x]))
                every_list.append(max(eb.keys(), key=lambda x: eb[x]))
                total += 1
                speech_list.append(max(sb.keys(), key=lambda x: sb[x]))
                if max(sb.keys(), key=lambda x: sb[x]) == ground_truth:
                    speech_total += 1
                if max(hb.keys(), key=lambda x: hb[x]) == ground_truth:
                    head_total += 1
                if max(ab.keys(), key=lambda x: ab[x]) == ground_truth:
                    arm_total += 1
                if max(mb.keys(), key=lambda x: mb[x]) == ground_truth:
                    multimodal_total += 1
                if max(eb.keys(), key=lambda x: eb[x]) == ground_truth:
                    everything_total += 1
            x = f.readline()
        print "Speech", speech_total*1.0 / total
        print "Head", head_total*1.0 / total
        print "Arms", arm_total*1.0 / total
        print "Multimodal", multimodal_total*1.0 / total
        print "Everything", everything_total*1.0 / total
        """
        prev = "silver_spoon"
        speech_total = 0.0
        head_total = 0.0
        arm_total = 0.0
        multimodal_total = 0.0
        everything_total = 0.0
        total = 0.0

        for i in range(1,len(real_list)):
            if not (real_list[i] == prev):
                prev = real_list[i]
                with open("data/speech.txt", 'a') as f:
                    f.write(str(speech_total/total) + "\n")
                with open("data/head.txt", 'a') as f:
                    f.write(str(head_total/total) + "\n")
                with open("data/arm.txt", 'a') as f:
                    f.write(str(arm_total/total) + "\n")
                with open("data/multi.txt", 'a') as f:
                    f.write(str(multimodal_total/total) + "\n")
                with open("data/every.txt", 'a') as f:
                    f.write(str(everything_total/total) + "\n")
                speech_total = 0.0
                head_total = 0.0
                arm_total = 0.0
                multimodal_total = 0.0
                everything_total = 0.0
                total = 0.0
            if speech_list[i] == real_list[i]: speech_total += 1.0
            if head_list[i] == real_list[i]: head_total += 1.0
            if arm_list[i] == real_list[i]: arm_total += 1.0
            if multi_list[i] == real_list[i]: multimodal_total += 1.0
            if every_list[i] == real_list[i]: everything_total += 1.0
            total += 1.0
        with open("data/speech.txt", 'a') as f:
            f.write(str(speech_total/total) + "\n")
        with open("data/head.txt", 'a') as f:
            f.write(str(head_total/total) + "\n")
        with open("data/arm.txt", 'a') as f:
            f.write(str(arm_total/total) + "\n")
        with open("data/multi.txt", 'a') as f:
            f.write(str(multimodal_total/total) + "\n")
        with open("data/every.txt", 'a') as f:
            f.write(str(everything_total/total) + "\n")
        """
        """
        speech_total = 0
        head_total = 0
        arm_total = 0
        multimodal_total = 0
        everything_total = 0
        prev = "None"
        f = 5
        for i in range(1,len(real_list)):
            if not (real_list[i] == prev):
                prev = real_list[i]
                if speech_list[i-f] == real_list[i-1]: speech_total += 1
                if head_list[i-f] == real_list[i-1]: head_total += 1
                if arm_list[i-f] == real_list[i-1]: arm_total += 1
                if multi_list[i-f] == real_list[i-1]: multimodal_total += 1
                if every_list[i-f] == real_list[i-1]: everything_total += 1
        i = len(real_list)
        if speech_list[i-f] == real_list[i-1]: speech_total += 1
        if head_list[i-f] == real_list[i-1]: head_total += 1
        if arm_list[i-f] == real_list[i-1]: arm_total += 1
        if multi_list[i-f] == real_list[i-1]: multimodal_total += 1
        if every_list[i-f] == real_list[i-1]: everything_total += 1
        print speech_total
        print head_total
        print arm_total
        print multimodal_total
        print everything_total
        """



if __name__ == "__main__":
    if len(sys.argv) < 3:
        print "Usage: rosrun gesture_rec gesture_relay.py <language model> <data file>"
        sys.exit(0)
    main()