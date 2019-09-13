# aDDa Object Tracking package

This ROS has the implementation of the GM-PHD filter in C++.

<img src="/tracker.png"/>

## How to run it?

```  
rosbag play <name of rosbag file from kitti2bag>
roslaunch adda_object_tracking adda_object_tracking.launch
```

## How to configure the parameters?
The list of parameters can be found in the config/GM_PHD_config.yaml
The parameters are loaded into the node when its launched and they can be modified during execution using the rosparam command-line tool.
