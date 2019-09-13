# kitti2bag
# Adda Worldmodel Server

The Worldmodel server includes 3 packages, namely adda_object_tracking, adda_worldmodel_msgs and adda_lidar_detection.

<img src="/tracker.png"/>

## How to run it?

```  
rosbag play <name of rosbag file from kitti2bag>
roslaunch adda_object_tracking adda_object_tracking.launch
