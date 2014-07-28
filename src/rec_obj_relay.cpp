#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h> 
#include <sensor_msgs/PointCloud2.h>
#include <object_recognition_msgs/RecognizedObject.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
ros::Publisher pub;
/*
 * clusterCallback : loads a marker array msg into a global variable
 */
void clusterCallback(const visualization_msgs::MarkerArray& clusters){
    object_recognition_msgs::RecognizedObjectArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "camera_depth_optical_frame";
    msg.header.seq = 0;
    PointCloud::Ptr pc (new PointCloud);

    for(int i= 0; i < clusters.markers.size(); i++){
        for(int j = 0; j<clusters.markers[i].points.size(); j++){
            pc->points.push_back(pcl::PointXYZ(clusters.markers[i].points[j].x, clusters.markers[i].points[j].y, clusters.markers[i].points[j].z));
        }
        sensor_msgs::PointCloud2 cloud;
        object_recognition_msgs::RecognizedObject obj;
        pcl::toROSMsg(*pc, cloud);
        pc->points.clear();
        //name the object
        std::stringstream sstm;
        sstm << "Object " << i;
        obj.type.key = sstm.str();
        //add object point cloud
        obj.point_clouds.push_back(cloud);
        //add timestamp
        obj.header.stamp = msg.header.stamp;
        obj.header.frame_id = "camera_depth_optical_frame";
        //misc fields
        obj.header.seq = i + 1;
        obj.type.db = "none";
        msg.objects.push_back(obj);
    }
    pub.publish(msg);
    return;
}
int main(int argc, char **argv){
    //ros setup
    ros::init(argc, argv, "h2r_relay");
    ros::NodeHandle node;
    pub = node.advertise<object_recognition_msgs::RecognizedObjectArray>("recognized_objects_array", 100);
    ros::Subscriber sub = node.subscribe("/tabletop/clusters", 1, &clusterCallback);
    ros::spin();
}