import argparse
import random
import math
import numpy as np
import scipy.stats

global t
global num_objects
global variance
global num_tests
global num_steps
global world_x
global world_y
global world_z
global time_per_object
world_x = world_y = world_z = 10.0
descriptors = ["red", "blue", "green", "orange", "purple", "black", "white", "square", "circle", "triangle", "bridge", "soft", "hard", "round", "shiny", "dull", "bright", "sharp", "large", "small", "long", "short"]

# vector utilities
def dot(v1, v2):
    total = 0.0
    for i in range(len(v1)):
        total += v1[i] * v2[i]
    return total
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
def angle_between(v1, v2):
    return math.acos(dot(v1, v2)/(norm(v1)* norm(v2)))


def gen_vector(x,y,z):
    origin = (random.uniform(0.0, world_x), random.uniform(0.0, world_y), random.uniform(0.0, world_z))
    found = False
    radians = abs(np.random.normal(scale=math.sqrt(variance))) #draws a sample from a gaussian
    tries = 0
    while not found:
        tries += 1
        point = (random.uniform(0.0, world_x), random.uniform(0.0, world_y), random.uniform(0.0, world_z))
        sample_radians = abs(angle_between(sub_vec(point, origin), sub_vec((x,y,z), origin)))
        if sample_radians >= radians - 0.05 and sample_radians <= radians + 0.05:
            return (origin, point)
        if tries > 100:
            return gen_vector(x,y,z)
    
def gen_speech(word_dict):
    return [np.random.choice(word_dict.keys(), p=word_dict.values())]

def gen_object():
    word_probabilities = dict()
    for word in descriptors:
        word_probabilities[word] = 0.0
    for _ in range(5): # how many descriptors should each object have?
        word_probabilities[random.choice(descriptors)] += 0.2
    for word in descriptors:
        if word_probabilities[word] == 0.0:
            word_probabilities[word] = 0.1 #eps?
    total = sum(word_probabilities.values())
    for word in descriptors:
        word_probabilities[word] = word_probabilities[word]/total
    return (random.uniform(0.0, world_x), random.uniform(0.0, world_y), random.uniform(0.0, world_z), word_probabilities)

def prob_of_sample(sample):
    return scipy.stats.norm(0.0, math.sqrt(variance)).pdf(sample)
    #return 1 - (sample/3.14)

def run_tests():
    correct_head = 0
    correct_speech = 0
    correct_arm = 0
    correct_multimodal = 0
    total = num_tests * num_steps * 1.0
    for test in range(num_tests):
        print "Test number", test
        objects = []
        head_belief = dict()
        speech_belief = dict()
        arm_belief = dict()
        multimodal_belief = dict()
        #generate the objects for this test
        for i in range(num_objects):
            objects+=[(gen_object())]
            head_belief[i] = speech_belief[i] = arm_belief[i] = multimodal_belief[i] = 1.0/num_objects
        for i in range(num_steps):
            #switch objects at the set rate
            if i % time_per_object == 0:
                true_object_index = random.choice(range(num_objects))
                true_object = objects[true_object_index]
            #generate random observations
            arm1_origin, arm1_point = gen_vector(true_object[0], true_object[1], true_object[2])
            arm2_origin, arm2_point = gen_vector(true_object[0], true_object[1], true_object[2])
            head_origin, head_point = gen_vector(true_object[0], true_object[1], true_object[2])
            speech = gen_speech(true_object[3])
            new_head_belief = dict()
            new_speech_belief = dict()
            new_arm_belief = dict()
            new_multimodal_belief = dict()
            for o1 in range(num_objects):
                curr_object = objects[o1]
                object_point = (curr_object[0],curr_object[1],curr_object[2])
                #do the transition probabilities
                new_head_belief[o1] = 0.0
                new_arm_belief[o1] = 0.0
                new_multimodal_belief[o1] = 0.0
                new_speech_belief[o1] = 0.0
                for o2 in range(num_objects):
                    if o1 == o2: trans_prob = 1-t
                    else: trans_prob = t
                    new_head_belief[o1] += head_belief[o2] * trans_prob
                    new_speech_belief[o1] += speech_belief[o2] * trans_prob
                    new_arm_belief[o1] += arm_belief[o2] * trans_prob
                    new_multimodal_belief[o1] += multimodal_belief[o2] * trans_prob
                #check the head
                head_prob = prob_of_sample(angle_between(sub_vec(head_point, head_origin), sub_vec(object_point, head_origin)))
                new_head_belief[o1] *= head_prob
                new_multimodal_belief[o1] *= head_prob
                #check the arms
                arm1_prob = prob_of_sample(angle_between(sub_vec(arm1_point, arm1_origin), sub_vec(object_point, arm1_origin)))
                arm2_prob = prob_of_sample(angle_between(sub_vec(arm2_point, arm2_origin), sub_vec(object_point, arm2_origin)))
                new_arm_belief[o1] *= arm1_prob * arm2_prob
                new_multimodal_belief[o1] *= arm1_prob * arm2_prob
                #check the speech
                speech_prob = 1.0
                for word in speech:
                    speech_prob *= curr_object[3][word]
                new_speech_belief[o1] *= speech_prob
                new_multimodal_belief[o1] *= speech_prob
            #normalize!
            m_total = sum(new_multimodal_belief.values())
            s_total = sum(new_speech_belief.values())
            a_total = sum(new_arm_belief.values())
            h_total = sum(new_head_belief.values())
            for o in range(num_objects):
                new_multimodal_belief[o] = new_multimodal_belief[o]/m_total
                new_speech_belief[o] = new_speech_belief[o]/s_total
                new_arm_belief[o] = new_arm_belief[o]/a_total
                new_head_belief[o] = new_head_belief[o]/h_total
            head_belief = new_head_belief
            arm_belief = new_arm_belief
            speech_belief = new_speech_belief
            multimodal_belief = new_multimodal_belief
            if max(head_belief.keys(), key=lambda x: head_belief[x]) == true_object_index: correct_head += 1
            if max(arm_belief.keys(), key=lambda x: arm_belief[x]) == true_object_index: correct_arm += 1
            if max(speech_belief.keys(), key=lambda x: speech_belief[x]) == true_object_index: correct_speech += 1
            if max(multimodal_belief.keys(), key=lambda x: multimodal_belief[x]) == true_object_index: correct_multimodal += 1
    print "Head only:",correct_head/total 
    print "Arms only:", correct_arm/total
    print "Speech only:", correct_speech/total
    print "Multimodal:", correct_multimodal/total

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--numtests', help="The number of tests to run", type=int, default=100)
    parser.add_argument('-s', '--timesteps', help='The number of timesteps per test', type=int, default=100)
    parser.add_argument('-o', '--objects', help="The number of objects per test", type=int, default=5)
    parser.add_argument('-v', '--variance', help="The variance (radians) on the guassian for each gesture", type=float, default=0.5)
    parser.add_argument('-t', '--transition', help='The probability of transitioning to a different state', type=float, default=0.01)
    parser.add_argument('-x', '--timeallotted', help='The time spent indicating a single object', type=int, default=10)
    args = parser.parse_args()
    global t
    t = args.transition
    global num_objects
    num_objects = args.objects
    global variance
    variance = args.variance
    global num_tests
    num_tests = args.numtests
    global num_steps
    num_steps = args.timesteps
    global time_per_object
    time_per_object = args.timeallotted
    run_tests()