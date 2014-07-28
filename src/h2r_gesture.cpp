#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <object_recognition_msgs/RecognizedObject.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <Eigen/Dense>
#include <std_msgs/String.h>
#include <math.h>

ros::Subscriber obj_sub; //subscriber for objects
ros::Subscriber speech_sub; //subscriber for objects
//visualization_msgs::MarkerArray clusters; //clusters from tabletop
object_recognition_msgs::RecognizedObjectArray objects; //objects from ork
std::string tracker_namespace = "tracker_camera"; //namespace for tracker camera topic
std::string ork_namespace = "ork_camera"; //namespace for ork camera topic

//points for vectors (observations)
Eigen::Vector3d right_origin;
Eigen::Vector3d right_point;
Eigen::Vector3d left_origin;
Eigen::Vector3d left_point;
Eigen::Vector3d head_origin;
Eigen::Vector3d head_point;
//state estimates
std::map<std::string, double> previous_estimate;
std::map<std::string, double> new_estimate;
double stay_prob = 0.9;
std::string recent_speech = "";


/*
 * clusterCallback : loads a recognized object array msg into a global variable
 */
void objectCallback(const object_recognition_msgs::RecognizedObjectArray& msg){
    objects = msg;
}
/*
 * speechCallback : receives and stores the speech instance
 */
void speechCallback(const std_msgs::String& msg){
    recent_speech = msg.data;
}

/*
 * angle_between : calculates the angle (radians) between two origin based vectors
 * IN : v1 - a 3d vector
 *      v2 - a 3d vector
 * OUT: a double representing the angle in radians between v1 and v2
 */
double angle_between(Eigen::Vector3d v1, Eigen::Vector3d v2){
    return std::acos(v1.dot(v2)/(v1.norm() * v2.norm()));
}

/*
 * fill_points : fills the *_origin and *_point variables with the most recent info
 * IN: tfl - a transform listener to read the transforms
 * OUT: true if success, false if failure due to error.
 */
bool fill_points(tf::TransformListener& tfl){
    try{
        tf::StampedTransform to_left_elbow;
        tf::StampedTransform to_right_elbow;
        tf::StampedTransform to_right_hand;
        tf::StampedTransform to_left_hand;
        tf::StampedTransform to_head;
        Eigen::Vector3d temp_helper;
        tf::Vector3 temp_vec;
        tf::Transform temp_tf;
        std::string frame = "/world";
        //std::string frame = std::string("/") + tracker_namespace + std::string("_depth_optical_frame");
        tfl.lookupTransform(frame,"/left_elbow_1", ros::Time(0), to_left_elbow);
        tfl.lookupTransform(frame,"/right_elbow_1", ros::Time(0), to_right_elbow);
        tfl.lookupTransform(frame,"/left_hand_1", ros::Time(0), to_left_hand);
        tfl.lookupTransform(frame,"/right_hand_1", ros::Time(0), to_right_hand);
        tfl.lookupTransform(frame,"/head_1", ros::Time(0), to_head);
        right_origin = Eigen::Vector3d(to_right_elbow.getOrigin().x(), to_right_elbow.getOrigin().y(),to_right_elbow.getOrigin().z());
        right_point = Eigen::Vector3d(to_right_hand.getOrigin().x(), to_right_hand.getOrigin().y(),to_right_hand.getOrigin().z());
        temp_helper = right_point - right_origin;
        right_origin = right_point;
        right_point = right_point + temp_helper; //deal with vector out of hand not elbow
        left_origin = Eigen::Vector3d(to_left_elbow.getOrigin().x(), to_left_elbow.getOrigin().y(),to_left_elbow.getOrigin().z());
        left_point = Eigen::Vector3d(to_left_hand.getOrigin().x(), to_left_hand.getOrigin().y(),to_left_hand.getOrigin().z());
        temp_helper = left_point - left_origin;
        left_origin = left_point;
        left_point = left_point + temp_helper; //deal with vector out of hand not elbow
        head_origin = Eigen::Vector3d(to_head.getOrigin().x(), to_head.getOrigin().y(),to_head.getOrigin().z());
        temp_tf = to_head.inverse();
        temp_vec = temp_tf(tf::Vector3(0.1,0.1,0));
        head_point = Eigen::Vector3d(temp_vec.x(), temp_vec.y(), temp_vec.z());
        return true;
    }catch(tf::TransformException ex){
        //ROS_ERROR("Failed to connect to openni transforms");
        return false;
    }
}

double log_guassian(double mu, double std_dev, double sample){
    return -log(std_dev*sqrt(2*M_PI)) - (((sample-mu) * (sample-mu))/(2*std_dev*std_dev));
}

double point_log_prob(Eigen::Vector3d origin, Eigen::Vector3d p1, Eigen::Vector3d p2){
    double theta = angle_between(p1 - origin, p2 - origin);
    return log_guassian(0.0, 0.5, theta);
}

/*
 * object_prob : given a gesture calculates p(l | x) for object x
 */
double object_log_prob(Eigen::Vector3d origin, Eigen::Vector3d p1, object_recognition_msgs::RecognizedObject object){
    return 1.0;
}
double speech_log_prob(object_recognition_msgs::RecognizedObject object){
    return 1.0;
}

void object_determiner(tf::TransformListener& tfl){
    fill_points(tfl); // get up to date head and arm measurements
    object_recognition_msgs::RecognizedObject obj;
    double log_prob;
    std::map<std::string, double> temp;
    if(objects.objects.size() > 0){
        //initialization
        if(previous_estimate.empty()){
            log_prob = log(1.0 / objects.objects.size());
            for(int i = 0; i<objects.objects.size(); i++){
                obj = objects.objects[i];
                previous_estimate[obj.type.key] = log_prob;
            }
        }
        for(int i = 0; i<objects.objects.size(); i++){
            obj = objects.objects[i];
            //double check the ork publishes point clouds
            log_prob = object_log_prob(left_origin, left_point, objects.objects[i]);
            log_prob += object_log_prob(right_origin, right_point, objects.objects[i]);
            log_prob += object_log_prob(head_origin, head_point, objects.objects[i]);
            log_prob += speech_log_prob(objects.objects[i]);
        }
    }else{
        ROS_ERROR("NO OBJECTS DETECTED");
    }
}



void tester(){
    //TEST 1
    previous_estimate["obj1"] = 0.5;
    previous_estimate["obj2"] = 0.5;
    Eigen::Vector3d origin = Eigen::Vector3d(0.0,0.0,0.0);
    Eigen::Vector3d arm1[7] = {Eigen::Vector3d(1.0, 0.0, 0.0),
                                Eigen::Vector3d(0.9, 0.44, 0.0),
                                Eigen::Vector3d(0.8, 0.6, 0.0),
                                Eigen::Vector3d(0.7, 0.7, 0.0),
                                Eigen::Vector3d(0.6, 0.8, 0.0),
                                Eigen::Vector3d(0.44, 0.9, 0.0),
                                Eigen::Vector3d(0.0, 1.0, 0.0)};
    Eigen::Vector3d obj1 = Eigen::Vector3d(2.0, 0.0, 0.0);
    Eigen::Vector3d obj2 = Eigen::Vector3d(0.0, 2.0, 0.0);
    double prob1;
    double prob2;
    double total;
    std::map<std::string, double> temp;
    for(int i = 0; i<7; i++){
        ROS_INFO("*******TIME STEP %d******", i);
        prob1 =  exp(point_log_prob(origin, arm1[i], obj1));
        prob2 = exp(point_log_prob(origin, arm1[i], obj2));
        new_estimate["obj1"] = (previous_estimate["obj1"] * 0.9 + previous_estimate["obj2"] * 0.1) * prob1;
        new_estimate["obj2"] = (previous_estimate["obj2"] * 0.9 + previous_estimate["obj1"] * 0.1) * prob2;
        total = new_estimate["obj1"] + new_estimate["obj2"];
        new_estimate["obj1"] = new_estimate["obj1"] / total;
        new_estimate["obj2"] = new_estimate["obj2"] / total;
        ROS_INFO("Object 1: %f", new_estimate["obj1"]);
        ROS_INFO("Object 2: %f", new_estimate["obj2"]);
        previous_estimate.clear();
        temp = previous_estimate;
        previous_estimate = new_estimate;
        new_estimate = temp;
    }
    //TEST 2
    previous_estimate["obj1"] = 0.5;
    previous_estimate["obj2"] = 0.5;
    /*Eigen::Vector3d arm2[7] = {Eigen::Vector3d(0.0, 1.0, 0.0),
                                Eigen::Vector3d(0.44, 0.9, 0.0),
                                Eigen::Vector3d(0.6, 0.8, 0.0),
                                Eigen::Vector3d(0.7, 0.7, 0.0),
                                Eigen::Vector3d(0.8, 0.6, 0.0),
                                Eigen::Vector3d(0.9, 0.44, 0.0),
                                Eigen::Vector3d(1.0, 0.0, 0.0)};*/
    Eigen::Vector3d arm2[7] = {Eigen::Vector3d(0.0, 1.0, 0.0),
                                Eigen::Vector3d(0.0, 1.0, 0.0),
                                Eigen::Vector3d(0.0, 1.0, 0.0),
                                Eigen::Vector3d(0.0, 1.0, 0.0),
                                Eigen::Vector3d(0.0, 1.0, 0.0),
                                Eigen::Vector3d(0.0, 1.0, 0.0),
                                Eigen::Vector3d(0.0, 1.0, 0.0)};
    //revere of arm1
    for(int i = 0; i<7; i++){
        ROS_INFO("*******TIME STEP %d******", i);
        prob1 =  exp(point_log_prob(origin, arm1[i], obj1)) * exp(point_log_prob(origin, arm2[i], obj1));
        prob2 = exp(point_log_prob(origin, arm1[i], obj2)) *exp(point_log_prob(origin, arm2[i], obj2));
        new_estimate["obj1"] = (previous_estimate["obj1"] * 0.9 + previous_estimate["obj2"] * 0.1) * prob1;
        new_estimate["obj2"] = (previous_estimate["obj2"] * 0.9 + previous_estimate["obj1"] * 0.1) * prob2;
        total = new_estimate["obj1"] + new_estimate["obj2"];
        new_estimate["obj1"] = new_estimate["obj1"] / total;
        new_estimate["obj2"] = new_estimate["obj2"] / total;
        ROS_INFO("Object 1: %f", new_estimate["obj1"]);
        ROS_INFO("Object 2: %f", new_estimate["obj2"]);
        previous_estimate.clear();
        temp = previous_estimate;
        previous_estimate = new_estimate;
        new_estimate = temp;
    }
    return;
}

bool test=  true;
int main(int argc, char **argv){
    //ros setup
    ros::init(argc, argv, "h2r_gesture");
    if(test){
        tester();
        exit(0);
    }
    tf::TransformListener tfl;
    ros::NodeHandle node;
    //subscribe to clusters (maybe switch to rec obj array)
    obj_sub = node.subscribe("/recognized_objects_array", 1, &objectCallback);
    speech_sub = node.subscribe("/speech_recognition", 1, &speechCallback);
    //vis publisher
    ros::Rate r(30); //vis
    ros::spinOnce();
    while(node.ok()){
        object_determiner(tfl);
        r.sleep();
        ros::spinOnce();
    }
}