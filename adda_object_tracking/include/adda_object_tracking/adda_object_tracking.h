#ifndef adda_object_tracking_H
#define adda_object_tracking_H

#include <ros/ros.h>
#include <adda_worldmodel_msgs/Percepts.h>
#include <adda_worldmodel_msgs/Objects.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>

using namespace Eigen;

struct percept_info {
    
    percept_info(float prob, float z, geometry_msgs::Vector3 bb){
        det_prob = prob;
	z_position = z;
        box = bb;
    }

    float det_prob; // probability of non-occlusion
    float z_position; // z position of a percept
    geometry_msgs::Vector3 box; // estimated bounding box
};

struct state {

    state(int n_id, float life, Vector4f sX, Matrix4f sP, float sw, float prob, int mI){
        
        id = n_id;
        lifetime = life;
        x = sX;
        P = sP;
        w = sw;
        det_prob = prob;
        meas_index = mI;
	
    }

    int id;
    float lifetime;
    Vector4f x; //state
    Matrix4f P; // covariance
    float w; // state hypotheses weight
    float det_prob; // probability of non-occlusion
    int meas_index; // measurement index that originated the given state hypotesis
                    // -1 means that no measurement has been associated

};

namespace adda_object_tracking
{

class GM_PHD_Tracker 
{
public:
    GM_PHD_Tracker(const ros::Publisher& vis_pub, const ros::Publisher& objects_pub);
    void Callback(const adda_worldmodel_msgs::Percepts::ConstPtr& percept_msg);
	
private:
    int total_ids_ = 0;
    std::vector<state> states_;
    std::vector<percept_info> percepts_;
    ros::Publisher vis_pub_;
    ros::Publisher objects_pub_;
};

} // end of namespace adda_object_tracking

#endif // adda_object_tracking_H
