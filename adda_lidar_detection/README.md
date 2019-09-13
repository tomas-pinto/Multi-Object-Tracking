# kitti2bag
Adaptation of kitti2bag (https://github.com/tomas789/kitti2bag) to include tracklets visualization in RViz.

<img src="example.png"/>

## How to install it?

1.Create a Conda Environment with Python 2 

```bash
$ conda create -n <env_name> python=2
$ conda activate <env_name>
```

2.Install kitti2bag

```bash
$ git clone <url>
$ cd kitti2bag
$ pip install -e .
$ cd .. 
```

## How to run it?

One example is better then thousand words so here it is:

```bash
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0002/2011_09_26_drive_0002_sync.zip
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0002/2011_09_26_drive_0002_tracklets.zip
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip
$ unzip 2011_09_26_drive_0002_sync.zip
$ unzip 2011_09_26_drive_0002_tracklets.zip
$ unzip 2011_09_26_calib.zip
$ kitti2bag -t 2011_09_26 -r 0002 raw_synced .
Exporting static transformations
Exporting time dependent transformations
Exporting IMU
Exporting camera 0
100% (77 of 77) |############################################| Elapsed Time: 0:00:00 Time:  0:00:00
Exporting camera 1
100% (77 of 77) |############################################| Elapsed Time: 0:00:00 Time:  0:00:00
Exporting camera 2
100% (77 of 77) |############################################| Elapsed Time: 0:00:01 Time:  0:00:01
Exporting camera 3
100% (77 of 77) |############################################| Elapsed Time: 0:00:02 Time:  0:00:02
Exporting velodyne data
100% (77 of 77) |############################################| Elapsed Time: 0:00:28 Time:  0:00:28
parsing tracklet file ./2011_09_26/2011_09_26_drive_0002_sync/tracklet_labels.xml
file contains 3 tracklets
loaded 3 tracklets
Exporting tracklet data
## OVERVIEW ##
path:        kitti_2011_09_26_drive_0002_synced.bag
version:     2.0
duration:    7.8s
start:       Sep 26 2011 13:02:44.33 (1317034964.33)
end:         Sep 26 2011 13:02:52.18 (1317034972.18)
size:        417.2 MB
messages:    1155
compression: none [309/309 chunks]
types:       geometry_msgs/TwistStamped     [98d34b0043a2093cf9d9345ab6eef12e]
             sensor_msgs/CameraInfo         [c9a58c1b0b154e0e6da7578cb991d214]
             sensor_msgs/Image              [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/Imu                [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/NavSatFix          [2d3a8cd499b9b4a0249fb98fd05cfa48]
             sensor_msgs/PointCloud2        [1158d486dd51d683ce2f1be655c3c181]
             tf2_msgs/TFMessage             [94810edda583a504dfda3829e70d7eec]
             visualization_msgs/MarkerArray [d155b9ce5188fbaf89745847fd5882d7]
topics:      /kitti/camera_color_left/camera_info    77 msgs    : sensor_msgs/CameraInfo        
             /kitti/camera_color_left/image_raw      77 msgs    : sensor_msgs/Image             
             /kitti/camera_color_right/camera_info   77 msgs    : sensor_msgs/CameraInfo        
             /kitti/camera_color_right/image_raw     77 msgs    : sensor_msgs/Image             
             /kitti/camera_gray_left/camera_info     77 msgs    : sensor_msgs/CameraInfo        
             /kitti/camera_gray_left/image_raw       77 msgs    : sensor_msgs/Image             
             /kitti/camera_gray_right/camera_info    77 msgs    : sensor_msgs/CameraInfo        
             /kitti/camera_gray_right/image_raw      77 msgs    : sensor_msgs/Image             
             /kitti/oxts/gps/fix                     77 msgs    : sensor_msgs/NavSatFix         
             /kitti/oxts/gps/vel                     77 msgs    : geometry_msgs/TwistStamped    
             /kitti/oxts/imu                         77 msgs    : sensor_msgs/Imu               
             /kitti/tracklet                         77 msgs    : visualization_msgs/MarkerArray
             /kitti/velo/pointcloud                  77 msgs    : sensor_msgs/PointCloud2       
             /tf                                     77 msgs    : tf2_msgs/TFMessage            
             /tf_static                              77 msgs    : tf2_msgs/TFMessage
