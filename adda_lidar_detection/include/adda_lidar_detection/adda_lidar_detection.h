#ifndef adda_lidar_detection_H
#define adda_lidar_detection_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <adda_worldmodel_msgs/Percepts.h>
#include <visualization_msgs/MarkerArray.h>

namespace adda_lidar_detection
{
class Detection
{
public:
  Detection(const ros::Publisher& cloud_pub, const ros::Publisher& percept_pub, const ros::Publisher& vis_pub);
  void Callback(const sensor_msgs::PointCloud2& msg);

private:
  int n_sequence_ = 0;
  ros::Publisher cloud_pub_;
  ros::Publisher percept_pub_;
  ros::Publisher vis_pub_;
};

}  // end of namespace adda_lidar_detection

#endif  // adda_lidar_detection_H
