#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h> 
#include <Eigen/Dense>
#include <limits>
#include <math.h>
visualization_msgs::MarkerArray clusters;

void clusterCallback(const visualization_msgs::MarkerArray& msg){
    clusters = msg;
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
    Eigen::Vector3d x0;
    Eigen::Vector3d x1;
    Eigen::Vector3d x2;
    Eigen::Vector3d x3;
    Eigen::Vector3d x4;
    tf::StampedTransform to_left_elbow;
    tf::StampedTransform to_right_elbow;
    tf::StampedTransform to_right_hand;
    tf::StampedTransform to_left_hand;
    float dist;
    float cluster_dist;
    float min_dist;
    //float angle;

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
            best_points.header.stamp = ros::Time::now();
            //find the closest cluster for the right arm
            x1 = Eigen::Vector3d(to_right_elbow.getOrigin().x(), to_right_elbow.getOrigin().y(),to_right_elbow.getOrigin().z());
            x2 = Eigen::Vector3d(to_right_hand.getOrigin().x(), to_right_hand.getOrigin().y(),to_right_hand.getOrigin().z());
            x3 = Eigen::Vector3d(to_left_elbow.getOrigin().x(), to_left_elbow.getOrigin().y(),to_left_elbow.getOrigin().z());
            x4 = Eigen::Vector3d(to_left_hand.getOrigin().x(), to_left_hand.getOrigin().y(),to_left_hand.getOrigin().z());
            min_dist = std::numeric_limits<float>::infinity();
            //currently just right arm
            for(int i = 0; i<clusters.markers.size(); i++){
                cluster_dist = std::numeric_limits<float>::infinity();
                for(int j=0; j<clusters.markers[i].points.size(); j++){
                    x0 = Eigen::Vector3d(clusters.markers[i].points[j].x,clusters.markers[i].points[j].y,clusters.markers[i].points[j].z);
                    dist = ((x2 - x1).cross(x1 - x0)).norm()/(x2-x1).norm(); // distance perpendicular to line
                    //angle = std::atan2(((x2-x1).cross(x0 - x1)).norm(),(x2 - x1)*(x0-x1)); // double check
                    if(dist < cluster_dist){
                        cluster_dist = dist;
                    }
                    /*if(angle > 1.57){
                       cluster_dist = std::numeric_limits<float>::infinity();
                       break; 
                    }*/
                }
                if(cluster_dist < min_dist){
                    min_dist = cluster_dist;
                    best_points.points = clusters.markers[i].points;
                }
            }
            marker_pub.publish(best_points);
            //test code
            /*
            visualization_msgs::Marker right_arm;
            right_arm.header.frame_id = "/camera_depth_optical_frame";
            right_arm.type = visualization_msgs::Marker::LINE_LIST;
            right_arm.action = visualization_msgs::Marker::ADD;
            right_arm.header.stamp = ros::Time::now();
            right_arm.id = 1;
            right_arm.ns = "gesture_marker";
            right_arm.color.g = 1.0f;
            right_arm.color.a = 1.0;
            right_arm.scale.x = 0.1;
            geometry_msgs::Point p;
            p.x = x1.x();
            p.y = x1.y();
            p.z = x1.z();
            right_arm.points.push_back(p);
            p.x = x2.x();
            p.y = x2.y();
            p.z = x2.z();
            right_arm.points.push_back(p);
            marker_pub.publish(right_arm);
            */
            r.sleep();
            ros::spinOnce();          
        }catch(tf::TransformException ex){
           ROS_ERROR("Failed to connect to openni transforms"); 
        }
    }
}
