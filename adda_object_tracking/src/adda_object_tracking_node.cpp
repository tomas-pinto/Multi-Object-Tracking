#include <ros/ros.h>
#include <adda_object_tracking/adda_object_tracking.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "adda_object_tracking_node");

    ros::NodeHandle nh;
    
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("worldmodel_visualization", 1, true);
    ros::Publisher objects_pub = nh.advertise<adda_worldmodel_msgs::Objects>("objects", 1, true);
    
    adda_object_tracking::GM_PHD_Tracker tracker(vis_pub, objects_pub);

    ros::Subscriber percept_sub = nh.subscribe("lidar_percepts", 1, &adda_object_tracking::GM_PHD_Tracker::Callback, &tracker);
    
    ros::spin();
    
    return 0;
}
