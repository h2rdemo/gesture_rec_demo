#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h> 
#include <Eigen/Dense>

ros::Subscriber sub; //subscriber for clusters
visualization_msgs::MarkerArray clusters; //clusters from tabletop
std::string tracker_namespace = "tracker_camera"; //namespace for tracker camera topic
std::string ork_namespace = "ork_camera"; //namespace for ork camera topic

//points for vectors
Eigen::Vector3d right_origin;
Eigen::Vector3d right_point;
Eigen::Vector3d left_origin;
Eigen::Vector3d left_point;
Eigen::Vector3d head_origin;
Eigen::Vector3d head_point;

/*
 * clusterCallback : loads a marker array msg into a global variable
 */
void clusterCallback(const visualization_msgs::MarkerArray& msg){
    clusters = msg;
}
/*
 * angle_between : calculates the angle (radians) between two origin based vectors
 * IN : v1 - a 3d vector
 *      v2 - a 3d vector
 * OUT: a float representing the angle in radians between v1 and v2
 */
float angle_between(Eigen::Vector3d v1, Eigen::Vector3d v2){
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

int main(int argc, char **argv){
    //ros setup
    ros::init(argc, argv, "h2r_gesture");
    tf::TransformListener tfl;
    ros::NodeHandle node;
    //subscribe to clusters (maybe switch to rec obj array)
    sub = node.subscribe("/tabletop/clusters", 1, &clusterCallback);
    //vis publisher
    ros::Rate r(30); //vis
    while(node.ok()){
        fill_points(tfl);
        r.sleep();
        ros::spinOnce();          
    }
}