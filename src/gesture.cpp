#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h> 
#include <Eigen/Dense>
#include <limits>
#include <math.h>
#include <geometry_msgs/PointStamped.h>


ros::Subscriber sub; //subscriber for clusters
ros::Publisher marker_pub; //publisher for visualization
visualization_msgs::MarkerArray clusters; //clusters from tabletop


visualization_msgs::Marker left_points, right_points, head_points, lines; //markers for visualization

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
        tfl.lookupTransform("/camera_depth_optical_frame","/left_elbow_1", ros::Time(0), to_left_elbow);
        tfl.lookupTransform("/camera_depth_optical_frame","/right_elbow_1", ros::Time(0), to_right_elbow);
        tfl.lookupTransform("/camera_depth_optical_frame","/left_hand_1", ros::Time(0), to_left_hand);
        tfl.lookupTransform("/camera_depth_optical_frame","/right_hand_1", ros::Time(0), to_right_hand);
        tfl.lookupTransform("/camera_depth_optical_frame","/head_1", ros::Time(0), to_head);
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
        ROS_ERROR("Failed to connect to openni transforms");
        return false;
    }
}

/*
 * setup_viz : sets up the colors and whatnot for visualization
 */
void setup_viz(){
    //left arm (red)
    left_points.header.frame_id = "/camera_depth_optical_frame";
    left_points.type = visualization_msgs::Marker::POINTS;
    left_points.action = visualization_msgs::Marker::ADD;
    left_points.id = 1;
    left_points.ns = "gesture_marker_left";
    left_points.color.r = 1.0;
    left_points.color.a = 1.0;
    left_points.scale.x = 0.1;
    left_points.scale.y = 0.1;
    //right arm (green)
    right_points.header.frame_id = "/camera_depth_optical_frame";
    right_points.type = visualization_msgs::Marker::POINTS;
    right_points.action = visualization_msgs::Marker::ADD;
    right_points.id = 2;
    right_points.ns = "gesture_marker_right";
    right_points.color.g = 1.0f;
    right_points.color.a = 1.0;
    right_points.scale.x = 0.1;
    right_points.scale.y = 0.1;
    //head (blue)
    head_points.header.frame_id = "/camera_depth_optical_frame";
    head_points.type = visualization_msgs::Marker::POINTS;
    head_points.action = visualization_msgs::Marker::ADD;
    head_points.id = 3;
    head_points.ns = "gesture_marker_head";
    head_points.color.b = 1.0;
    head_points.color.a = 1.0;
    head_points.scale.x = 0.1;
    head_points.scale.y = 0.1;
    //vector vis
    lines.header.frame_id = "/camera_depth_optical_frame";
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.action = visualization_msgs::Marker::ADD;
    lines.id = 4;
    lines.ns = "gesture_marker";
    lines.color.g = 1.0f;
    lines.color.a = 1.0;
    lines.scale.x = 0.1;
}
/*
 * update_viz : recomputes all vectors and clusters and publishes them
 */
void update_viz(){
    right_points.points.clear(); //clear array of points
    left_points.points.clear();
    head_points.points.clear();
    lines.points.clear();
    //create the arm and head lines
    geometry_msgs::Point p;
    p.x = right_origin.x();
    p.y = right_origin.y();
    p.z = right_origin.z();
    lines.points.push_back(p);
    p.x = right_point.x();
    p.y = right_point.y();
    p.z = right_point.z();
    lines.points.push_back(p);
    p.x = left_origin.x();
    p.y = left_origin.y();
    p.z = left_origin.z();
    lines.points.push_back(p);
    p.x = left_point.x();
    p.y = left_point.y();
    p.z = left_point.z();
    lines.points.push_back(p);
    p.x = head_origin.x();
    p.y = head_origin.y();
    p.z = head_origin.z();
    lines.points.push_back(p);
    p.x = head_point.x();
    p.y = head_point.y();
    p.z = head_point.z();
    lines.points.push_back(p);
    //clusters
    Eigen::Vector3d cloud_point;
    float angle;
    for(int i= 0; i < clusters.markers.size(); i++){
        for(int j = 0; j<clusters.markers[i].points.size(); j++){
            cloud_point = Eigen::Vector3d(clusters.markers[i].points[j].x,clusters.markers[i].points[j].y,clusters.markers[i].points[j].z);
            angle = angle_between(right_point-right_origin,cloud_point-right_origin);
            if(angle < 0.6 && angle > -0.6){
                right_points.points.push_back(clusters.markers[i].points[j]);
            }
            angle = angle_between(left_point-left_origin,cloud_point-left_origin);
            if(angle < 0.6 && angle > -0.6){
                left_points.points.push_back(clusters.markers[i].points[j]);
            }
            angle = angle_between(head_point-head_origin,cloud_point-head_origin);
            if(angle < 0.6 && angle > -0.6){
                head_points.points.push_back(clusters.markers[i].points[j]);
            }
        }
    }
    //update all time stamps
    left_points.header.stamp = ros::Time::now();
    right_points.header.stamp = ros::Time::now();
    head_points.header.stamp = ros::Time::now();
    lines.header.stamp = ros::Time::now();
    //and publish
    marker_pub.publish(lines);
    marker_pub.publish(right_points);
    marker_pub.publish(left_points);
    marker_pub.publish(head_points);  
}


int main(int argc, char **argv){
    //ros setup
    ros::init(argc, argv, "gesture_looker");
    tf::TransformListener tfl;
    ros::NodeHandle node;
    //subscribe to clusters (maybe switch to rec obj array)
    sub = node.subscribe("/tabletop/clusters", 1, &clusterCallback);
    //vis publisher
    marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10); //visualization
    ros::Rate r(30); //vis

    //set up conversion from tf to optical
    tf::TransformBroadcaster br; //switch to static_transform_publisher
    tf::Transform transform;
    tf::Quaternion q;
    q.setRPY(M_PI*0.0, M_PI*1.5, M_PI*0.5); //rotate openni skeleton to depth frame
    transform.setOrigin(tf::Vector3(0,0,0));
    transform.setRotation(q);

    setup_viz();
    while(node.ok()){
        //broadcast transform
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "openni_depth_frame"));
        fill_points(tfl);
        update_viz();
        r.sleep();
        ros::spinOnce();          
    }
}
