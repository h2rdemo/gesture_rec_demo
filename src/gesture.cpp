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
visualization_msgs::MarkerArray clusters;

//gets the cluster info
void clusterCallback(const visualization_msgs::MarkerArray& msg){
    clusters = msg;
}
//calculates the angle (radians) between the two vectors (same origin)
float angle_between(Eigen::Vector3d v1, Eigen::Vector3d v2){
    return std::acos(v1.dot(v2)/(v1.norm() * v2.norm()));
}

int main(int argc, char **argv){
    //ros setup
    ros::init(argc, argv, "gesture_looker");
    tf::TransformListener tfl;
    ros::NodeHandle node;
    //subscribe to clusters (maybe switch to rec obj array)
    ros::Subscriber sub = node.subscribe("/tabletop/clusters", 1, &clusterCallback);
    //vis publisher
    ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10); //visualization
    ros::Rate r(30); //vis

    //set up conversion from tf to optical
    tf::TransformBroadcaster br; //switch to static_transform_publisher
    tf::Transform transform;
    tf::Quaternion q;
    q.setRPY(M_PI*0.0, M_PI*1.5, M_PI*0.5); //rotate openni skeleton to depth frame
    transform.setOrigin(tf::Vector3(0,0,0));
    transform.setRotation(q);

    //define variables
    Eigen::Vector3d cloud_point;
    Eigen::Vector3d right_origin;
    Eigen::Vector3d right_point;
    Eigen::Vector3d left_origin;
    Eigen::Vector3d left_point;
    Eigen::Vector3d head_origin;
    Eigen::Vector3d head_point;


    //transform holders
    tf::StampedTransform to_left_elbow;
    tf::StampedTransform to_right_elbow;
    tf::StampedTransform to_right_hand;
    tf::StampedTransform to_left_hand;
    tf::StampedTransform to_head;

    //variables
    float dist;
    float cluster_dist;
    float min_dist;
    float angle;
    tf::Vector3 temp_vec;
    tf::Transform temp_tf;
    Eigen::Vector3d temp_helper;

    //visualize best item
    visualization_msgs::Marker best_points;
    best_points.header.frame_id = "/camera_depth_optical_frame";
    best_points.type = visualization_msgs::Marker::POINTS;
    best_points.action = visualization_msgs::Marker::ADD;
    best_points.id = 0;
    best_points.ns = "gesture_marker";
    best_points.color.r = 1.0;
    best_points.color.a = 1.0;
    best_points.scale.x = 0.1;
    best_points.scale.y = 0.1;
    while(node.ok()){
        //broadcast transform
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "openni_depth_frame"));
        try{
            //get all the transforms from the perspective point to the arms
            tfl.lookupTransform("/camera_depth_optical_frame","/left_elbow_1", ros::Time(0), to_left_elbow);
            tfl.lookupTransform("/camera_depth_optical_frame","/right_elbow_1", ros::Time(0), to_right_elbow);
            tfl.lookupTransform("/camera_depth_optical_frame","/left_hand_1", ros::Time(0), to_left_hand);
            tfl.lookupTransform("/camera_depth_optical_frame","/right_hand_1", ros::Time(0), to_right_hand);
            tfl.lookupTransform("/camera_depth_optical_frame","/head_1", ros::Time(0), to_head);
            
            best_points.header.stamp = ros::Time::now(); //update the time stamp on the markers


            //create all points for the vectors
            right_origin = Eigen::Vector3d(to_right_elbow.getOrigin().x(), to_right_elbow.getOrigin().y(),to_right_elbow.getOrigin().z());
            right_point = Eigen::Vector3d(to_right_hand.getOrigin().x(), to_right_hand.getOrigin().y(),to_right_hand.getOrigin().z());
            temp_helper = right_point - right_origin;
            right_origin = right_point;
            right_point = right_point + temp_helper; //deal with vector out of hand not elbow
            left_origin = Eigen::Vector3d(to_left_elbow.getOrigin().x(), to_left_elbow.getOrigin().y(),to_left_elbow.getOrigin().z());
            left_point = Eigen::Vector3d(to_left_hand.getOrigin().x(), to_left_hand.getOrigin().y(),to_left_hand.getOrigin().z());
            temp_helper = left_point - left_origin;
            left_origin = left_point;
            left_point = left_point + left_helper; //deal with vector out of hand not elbow
            head_origin = Eigen::Vector3d(to_head.getOrigin().x(), to_head.getOrigin().y(),to_head.getOrigin().z());
            temp_tf = to_head.inverse();
            temp_vec = temp_tf(tf::Vector3(0.1,0.1,0));
            head_point = Eigen::Vector3d(temp_vec.x(), temp_vec.y(), temp_vec.z());
            /*HIGHLIGHT RGHT ARM POINT
            //initilize minimum distance
            min_dist = std::numeric_limits<float>::infinity();
            //find the closest cluster for the right arm
            //currently just right arm
            for(int i = 0; i<clusters.markers.size(); i++){
                cluster_dist = std::numeric_limits<float>::infinity();
                for(int j=0; j<clusters.markers[i].points.size(); j++){
                    cloud_point = Eigen::Vector3d(clusters.markers[i].points[j].x,clusters.markers[i].points[j].y,clusters.markers[i].points[j].z);
                    dist = ((right_point - right_origin).cross(right_origin - cloud_point)).norm()/(right_point-right_origin).norm(); // distance perpendicular to line
                    if(dist < cluster_dist){
                        cluster_dist = dist;
                    }
                    angle = angle_between(right_point-right_origin,cloud_point-right_origin);
                    if(angle > 1.57 || angle < -1.57){
                       cluster_dist = std::numeric_limits<float>::infinity();
                       break; 
                    }
                }
                if(cluster_dist < min_dist){
                    min_dist = cluster_dist;
                    best_points.points = clusters.markers[i].points;
                }
            }
            marker_pub.publish(best_points);
            */
            //test code
            
            visualization_msgs::Marker lines;
            lines.header.frame_id = "/camera_depth_optical_frame";
            lines.type = visualization_msgs::Marker::LINE_LIST;
            lines.action = visualization_msgs::Marker::ADD;
            lines.header.stamp = ros::Time::now();
            lines.id = 1;
            lines.ns = "gesture_marker";
            lines.color.g = 1.0f;
            lines.color.a = 1.0;
            lines.scale.x = 0.1;
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
            marker_pub.publish(lines);
            r.sleep();
            ros::spinOnce();          
        }catch(tf::TransformException ex){
           ROS_ERROR("Failed to connect to openni transforms"); 
        }
    }
}
