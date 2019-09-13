string object_type # type of object e.g. car, pedestrian, traffic sign
string sensor_type # e.g. lidar, camera, radar
geometry_msgs/PoseStamped pose # detected pose
geometry_msgs/Vector3 bounding_box # detected shape
float64 detection_certainty # non-occlusion probability [0,1]

# geometry_msgs/Twist velocity # detected velocity 
