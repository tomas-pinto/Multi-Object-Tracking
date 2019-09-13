#include <adda_lidar_detection/adda_lidar_detection.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/filters/voxel_grid.h"
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <pcl/filters/extract_indices.h>
#include "pcl/common/common.h"
#include "pcl/PointIndices.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <vector>
#include "geometry_msgs/Pose.h"
#include <adda_worldmodel_msgs/Percepts.h>
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include "simple_grasping/shape_extraction.h"
#include "shape_msgs/SolidPrimitive.h"
#include <chrono>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

void downsample(const PointCloudC::Ptr cloud){

	// Read the leaf size parameter
	double leaf_size;
	ros::param::get("/lidar_detection_node/downsample_leaf_size", leaf_size);
	
	// Apply Voxel Grid filter
	pcl::VoxelGrid<PointC> vox;
	vox.setInputCloud(cloud);
 	vox.setLeafSize(leaf_size, leaf_size, leaf_size);
 	vox.filter(*cloud);
 	ROS_INFO("Downsampled to %ld points", cloud->size());

}

void clip(const PointCloudC::Ptr cloud){
	
	// Initialize the indices that need to be clipped
	pcl::PointIndices::Ptr clip_indices(new pcl::PointIndices);

	// Get the minimum and maximum values along the z axis
	double min_z, max_z;
	ros::param::get("/lidar_detection_node/clip_cloud/min_z", min_z);
	ros::param::get("/lidar_detection_node/clip_cloud/max_z", max_z);
	
	// Store indices that are inside the boundary
	for (unsigned int i = 0; i < cloud->points.size(); i++){
		if (cloud->points[i].z >= min_z && cloud->points[i].z <= max_z){
			clip_indices->indices.push_back(i);
		}
	}
	
	// Remove indices that are not inside the boundary
	pcl::ExtractIndices<PointC> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(clip_indices);
	extract.setNegative(false);  // true removes the indices, false leaves only the indices
	extract.filter(*cloud);

}

void ground_removal(const PointCloudC::Ptr cloud){
	
	// Initialize RANSAC parameters
	double max_iter, dist_thresh, deg_tol;
	ros::param::get("/lidar_detection_node/ground_removal/max_iter", max_iter);
	ros::param::get("/lidar_detection_node/ground_removal/dist_thresh", dist_thresh);
	ros::param::get("/lidar_detection_node/ground_removal/deg_tol", deg_tol);
	
	// Initialize segmentation indices
	pcl::SACSegmentation<PointC> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	seg.setOptimizeCoefficients(true);

	// Search for a plane perpendicular to some axis (specified below).
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(max_iter);

	// Set the distance to the plane for a point to be an inlier.
	seg.setDistanceThreshold(dist_thresh);
	seg.setInputCloud(cloud);

	// Make sure that the plane is perpendicular to Z-axis with some degree tolerance.
	seg.setAxis(Eigen::Vector3f(0, 0, 1));
	seg.setEpsAngle(pcl::deg2rad(deg_tol));

	// coeff contains the coefficients of the plane:
	// ax + by + cz + d = 0
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0){
		std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	}

	// Remove the floor from the cloud
	pcl::ExtractIndices<PointC> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);  // true removes the indices, false leaves only the indices
	extract.filter(*cloud);

	ROS_INFO("Removed floor to %ld points", cloud->size());

}

std::vector<pcl::PointIndices> euclidian_clustering(const PointCloudC::Ptr cloud){
	
	// initialize Euclidean clustering parameters
  	double cluster_tolerance;
  	int min_cluster_size, max_cluster_size;
  	ros::param::get("/lidar_detection_node/clustering/tol", cluster_tolerance);
  	ros::param::get("/lidar_detection_node/clustering/min_size", min_cluster_size);
  	ros::param::get("/lidar_detection_node/clustering/max_size", max_cluster_size);
	
	// create 2d pointcloud
	PointCloudC::Ptr cloud_2d(new PointCloudC());
  	pcl::copyPointCloud(*cloud, *cloud_2d);
  	
	// make it flat
  	for (size_t i = 0; i < cloud_2d->points.size(); i++){
    		cloud_2d->points[i].z = 0;
  	}


  	// Creating the KdTree object for the search method of the extraction
  	pcl::search::KdTree<PointC>::Ptr tree (new pcl::search::KdTree<PointC>);
  	tree->setInputCloud(cloud_2d);

  	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<PointC> euclid;
  	euclid.setInputCloud(cloud_2d);
  	euclid.setClusterTolerance(cluster_tolerance);
  	euclid.setMinClusterSize(min_cluster_size);
  	euclid.setMaxClusterSize(max_cluster_size);
  	euclid.setSearchMethod (tree);
  	euclid.extract(cluster_indices);

  	ROS_INFO("Found %ld objects", cluster_indices.size());
	
	return cluster_indices;
}

void publish_percepts(const std::vector<pcl::PointIndices> cluster_indices,
		const PointCloudC::Ptr cloud,
		const ros::Publisher& percept_pub_,
		const ros::Publisher& vis_pub_,
		int sequence_id){

	// Initialize percepts msg and marker array
        adda_worldmodel_msgs::Percepts percepts_msg;
	visualization_msgs::MarkerArray lidar_detection_markers;

	for (size_t i = 0; i < cluster_indices.size(); ++i) {
    		
		// Retify indices into a point cloud of the object.
    		pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    		*indices = cluster_indices[i];
    		PointCloudC::Ptr object_cloud(new PointCloudC());

    		// Fill in object_cloud using indices
    		pcl::ExtractIndices<PointC> fill;
    		fill.setInputCloud(cloud);
    		fill.setIndices(indices);
    		fill.setNegative(false);  // true removes the indices, false leaves only the indices
    		fill.filter(*object_cloud);
                
		// Initialize percept msg
		adda_worldmodel_msgs::Percept percept_msg;

		percept_msg.object_type = "unknown";
		percept_msg.sensor_type = "lidar";
		percept_msg.pose.header.seq = sequence_id;
		percept_msg.pose.header.frame_id = "velo_link";
		percept_msg.pose.header.stamp = ros::Time::now();

		double certainty;
		ros::param::get("/lidar_detection_node/detection_certainty", certainty);
		percept_msg.detection_certainty = certainty;
		
		// Estimate pose and bounding box
    		shape_msgs::SolidPrimitive shape;
    		geometry_msgs::Pose pose;
		
    		PointCloudC::Ptr projected_cloud(new PointCloudC());
    		
		simple_grasping::extractShape(*object_cloud, *projected_cloud, shape, percept_msg.pose.pose);

    		if (shape.type == shape_msgs::SolidPrimitive::BOX) {
			percept_msg.bounding_box.x = shape.dimensions[0];
                        percept_msg.bounding_box.y = shape.dimensions[1];
                        percept_msg.bounding_box.z = shape.dimensions[2];

    		} else {
      			ROS_INFO("Error: Found another shape");
    		}
		
		percepts_msg.percepts.push_back(percept_msg);

                // Initialize centroid marker
                visualization_msgs::Marker centroid_marker;
                centroid_marker.ns = "centroids";
                centroid_marker.id = i;
                centroid_marker.header.frame_id = "velo_link";
                centroid_marker.type = visualization_msgs::Marker::CUBE;
                centroid_marker.lifetime = ros::Duration(0.1);
                centroid_marker.color.b = 1;
                centroid_marker.color.a = 1.0;

                centroid_marker.scale.x = 0.2;
                centroid_marker.scale.y = 0.2;
                centroid_marker.scale.z = 0.2;

                centroid_marker.pose = percept_msg.pose.pose;

                // Update centroid markers
                lidar_detection_markers.markers.push_back(centroid_marker);

		// Initialize bounding box marker
                visualization_msgs::Marker box_marker;
                box_marker.ns = "bounding_box";
                box_marker.id = i + cluster_indices.size() + 1;
                box_marker.header.frame_id = "velo_link";
                box_marker.type = visualization_msgs::Marker::CUBE;
                box_marker.lifetime = ros::Duration(0.1);
                box_marker.color.g = 1;
                box_marker.color.a = 0.3;

                box_marker.pose = percept_msg.pose.pose;
                box_marker.scale.x = percept_msg.bounding_box.x;
                box_marker.scale.y = percept_msg.bounding_box.y;
                box_marker.scale.z = percept_msg.bounding_box.z;

                // Update bounding box markers
		lidar_detection_markers.markers.push_back(box_marker);

  	}
	
	// Publish percepts
	percept_pub_.publish(percepts_msg);
        vis_pub_.publish(lidar_detection_markers);

}

namespace adda_lidar_detection
{
Detection::Detection(const ros::Publisher& cloud_pub, 
		const ros::Publisher& percept_pub, 
		const ros::Publisher& vis_pub) : cloud_pub_(cloud_pub), percept_pub_(percept_pub), vis_pub_(vis_pub) {}

void Detection::Callback(const sensor_msgs::PointCloud2& msg) {
  
  auto t0 = std::chrono::high_resolution_clock::now();

  // Read the pointcloud from the ROS message
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  ROS_INFO("Got point cloud with %ld points", cloud->size());
  
  // Downsample Point Cloud with Voxel Grid filter
  downsample(cloud);

  // Ground Removal with RANSAC
  ground_removal(cloud);

  // Clip Point Cloud
  clip(cloud);

  // Publish cloud
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_pub_.publish(cloud_msg);

  // Get Objects indices with 2D Euclidian Clustering
  std::vector<pcl::PointIndices> cluster_indices;
  cluster_indices = euclidian_clustering(cloud);
  
  // Publish Objects Marker Array
  publish_percepts(cluster_indices, cloud, percept_pub_, vis_pub_, n_sequence_);
  
  n_sequence_ += 1;

  auto diff=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-t0); 
  ROS_INFO("Duration of %ld ms.", diff.count());  
}

}
