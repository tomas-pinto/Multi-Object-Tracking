#include <adda_lidar_detection/adda_lidar_detection.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "visualization_msgs/Marker.h"
#include <adda_worldmodel_msgs/Percepts.h>

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "lidar_detection_node");

  ros::NodeHandle nh;

  ros::Publisher lidar_detection_percepts = nh.advertise<adda_worldmodel_msgs::Percepts>("lidar_percepts", 1, true);
  ros::Publisher lidar_detection_cloud = nh.advertise<sensor_msgs::PointCloud2>("lidar_cropped_cloud", 1, true);
  ros::Publisher lidar_detection_visualization = nh.advertise<visualization_msgs::MarkerArray>("lidar_detection_visualization", 1, true);  
  
  adda_lidar_detection::Detection detection(lidar_detection_cloud, lidar_detection_percepts, lidar_detection_visualization);

  ros::Subscriber sub = nh.subscribe("/kitti/velo/pointcloud", 1, &adda_lidar_detection::Detection::Callback, &detection);

  ros::spin();

  return 0;
}
