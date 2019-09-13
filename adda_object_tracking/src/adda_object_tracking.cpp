#include <adda_object_tracking/adda_object_tracking.h>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>

using namespace std;
using namespace Eigen;

void add_birth_model(vector<state>& states){
	
	// Birth id and lifetime
	int id = 0;
	float lifetime = 0.0;

	// Birth State
        Vector4f x0;
        x0 << 0.0,0.0,0.0,0.0;
        
	// Birth Covariance
        float birth_P;
        ros::param::get("/adda_object_tracking_node/birth_model/covariance", birth_P);
        
	Matrix4f P;
        P = Matrix4f::Identity() * birth_P;
        
	// Birth Weight
        float birth_w;
        ros::param::get("/adda_object_tracking_node/birth_model/weight", birth_w);
        
	// Birth Weight
        float det_prob;
        ros::param::get("/adda_object_tracking_node/birth_model/detection_prob", det_prob);

	// Update objects state hypotheses
        states.push_back(state(id, lifetime, x0, P,log(birth_w), det_prob, -1));
}

void read_percepts(const adda_worldmodel_msgs::Percepts::ConstPtr& percepts_msg, 
		int n_meas, MatrixXf& z, std::vector<percept_info>& percepts_){
	
	percepts_.clear();
	percepts_.reserve(n_meas);

	for (int i=0; i<n_meas; ++i){
                z(0,i) = percepts_msg->percepts[i].pose.pose.position.x;
                z(1,i) = percepts_msg->percepts[i].pose.pose.position.y;
		
		percepts_.emplace_back(percepts_msg->percepts[i].detection_certainty,
				percepts_msg->percepts[i].pose.pose.position.z,
				percepts_msg->percepts[i].bounding_box);
        }

}

void make_meas_model(MatrixXf& h, Matrix2f& R){
        
	// Sensor covariance
	float sigma_r;
        ros::param::get("/adda_object_tracking_node/measurement_model/covariance", sigma_r);

        R = Matrix2f::Identity() * pow(sigma_r,2);
        
	// Sensor model
	h << 1, 0, 0, 0,
             0, 1, 0, 0;

}

void update_mean_cov(state hyp, MatrixXf& h, Matrix2f& R, 
		Vector2f& z_pred, Matrix2f& S_p){

	z_pred = h * hyp.x;
        Matrix2f S = h * hyp.P * h.transpose() + R;

        //Make sure that covariance is positive definite
        S_p = 0.5*S + 0.5*S.transpose();

}

void ellipsoidal_gating(MatrixXf diff, Matrix2f S_upd, float gating_size,
		int n_meas, VectorXf& d, std::vector<int>& indexes){
	
	// Calculate Mahalanobis distance
        d = (diff.transpose() * S_upd.inverse() * diff).diagonal();

        // Get indexes that are inside the gate
        for(int i=0; i<n_meas; ++i){
		if (d(i) <= gating_size){
                        indexes.emplace_back(i);
                }
        }
}

void update_step(std::vector<state>& upd_states, ArrayXf& sum_weights, state hyp, 
		float gating_size, MatrixXf h, Matrix2f R, int n_meas, MatrixXf z, 
		std::vector<percept_info>& percepts_, int& total_ids){
        
	// Update mean and covariance
        Vector2f z_upd;
        Matrix2f S_upd;
        update_mean_cov(hyp,h,R,
                        z_upd,S_upd);

        // Ellipsoidal Gating
        std::vector<int> ind;
        VectorXf d(n_meas);
	ind.reserve(n_meas);
        ellipsoidal_gating(z.colwise()-z_upd,S_upd,gating_size,n_meas,d,ind);

        // Compute Kalman gain
        MatrixXf K(4,2);
        K = hyp.P * h.transpose() * S_upd.inverse();

        // Compute the state covariance matrix
        Matrix4f new_P = (Matrix4f::Identity() - K*h)*hyp.P;

        // Compute weight terms
        float term = log(hyp.det_prob) + hyp.w - 0.5*log(S_upd.determinant()) - log(2*M_PI);
        
	int id = hyp.id;

	for(int i=0; i<ind.size(); ++i){

		// Assign id to newborn hypothesis
		if (hyp.id == 0){
		   total_ids++;
		   id = total_ids;
		}

                // Compute weights
                float weight = term - 0.5*d(ind[i]);
		sum_weights(ind[i]) += exp(weight); 

                // Compute updated state
                Vector4f new_x = hyp.x + K*(z.col(ind[i]) - z_upd);

                // Update objects state hypotheses
		upd_states.emplace_back(id,hyp.lifetime,new_x,new_P,weight,percepts_[ind[i]].det_prob,ind[i]);

        }

}

void update_weights(std::vector<state>& upd_states, ArrayXf sum_weights){
	
	// Calculate Poisson Point Process intensity
	float min, max, clutter_rate, clutter_intensity;
	ros::param::get("/adda_object_tracking_node/sensor_model/min_range", min);
	ros::param::get("/adda_object_tracking_node/sensor_model/max_range", max);
	ros::param::get("/adda_object_tracking_node/sensor_model/clutter_rate", clutter_rate);

	clutter_intensity = clutter_rate/(pow(min-max,2));

	// Update weights
	for(int i=0; i<upd_states.size(); ++i){
		upd_states[i].w = upd_states[i].w - log(sum_weights(upd_states[i].meas_index) + clutter_intensity);
	}

}

struct weight_sorter
{
    inline bool operator() (state& hyp1, state& hyp2)
    {
        return (hyp1.w > hyp2.w);
    }
};

void reduction(std::vector<state>& states, std::vector<state>& upd_states){
	
	// Clear all states
	states.clear();
        
	// Get Capping and Pruning parameters
	int max_h; 
	float w_min;
        ros::param::get("/adda_object_tracking_node/GM_PHD_tracker/max_h", max_h);
        ros::param::get("/adda_object_tracking_node/GM_PHD_tracker/w_min", w_min);
        
        // Compute minimum number of states
	int n_states = upd_states.size();
        int n = min(n_states,max_h);
	
	states.reserve(n);

        // Sort the updated states by weight
	std::sort(upd_states.begin(), upd_states.end(), weight_sorter());
	
	// Push max_h hypothesis to states if weights are above threshold
	for(int i=0; i<n; ++i){
		if (upd_states[i].w >= log(w_min)){
			states.emplace_back(upd_states[i]);
		}
	}
}

void publish_estimates(std::vector<state>& states, std::vector<percept_info>& percepts_,
		const ros::Publisher& objects_pub_, const ros::Publisher& vis_pub_){

	// Calculate estimated number of objects
	int n_obj, n_phd_obj, n_states, n_markers;
	float sum_exp_w = {0.0};
        
        n_states = states.size();

	for(int i=0; i<n_states; ++i){
		sum_exp_w += exp(states[i].w);
	}

	n_phd_obj = round(sum_exp_w);
	n_obj = min(n_states, n_phd_obj);

	ROS_INFO("Number of objects: %d", n_obj);

	// Initialize msgs
        adda_worldmodel_msgs::Objects objects_msg;
	visualization_msgs::MarkerArray object_markers;
	
	n_markers = 1;

        for (size_t i = 0; i < n_obj; ++i) {

		// Calculate orientation from velocity estimation
                float vx, vy, angle, qz, qw, norm_v;
                vx = states[i].x(2);
                vy = states[i].x(3);
		norm_v = sqrt(pow(vx,2)+pow(vy,2));
                
		angle = atan2(vy,vx);
                qz = -sin(angle/2);
                qw = cos(angle/2);

                // Make object msg
                adda_worldmodel_msgs::Object object_msg;
		
		char obj_type[10] = "unknown";
                object_msg.object_type = obj_type;
                
		object_msg.object_id = states[i].id;
                object_msg.pose.header.frame_id = "velo_link";
                object_msg.pose.header.stamp = ros::Time::now();
                
                object_msg.pose.pose.position.x = states[i].x(0);
		object_msg.pose.pose.position.y = states[i].x(1);

		if (states[i].meas_index != -1){
		   object_msg.pose.pose.position.z = percepts_[states[i].meas_index].z_position; 
                   object_msg.bounding_box = percepts_[states[i].meas_index].box;
		}
		
		object_msg.velocity.linear.x = vx;
		object_msg.velocity.linear.y = vy;

		object_msg.pose.pose.orientation.z = qz;
		object_msg.pose.pose.orientation.w = qw;
		
		object_msg.certainty = exp(states[i].w);
		object_msg.lifetime = states[i].lifetime;
                
		objects_msg.objects.push_back(object_msg);

		// Make bounding box msg
		visualization_msgs::Marker box_marker;
                
		box_marker.ns = "bounding_box";
                box_marker.id = n_markers;
                box_marker.header.frame_id = "velo_link";
                box_marker.type = visualization_msgs::Marker::CUBE;
                box_marker.lifetime = ros::Duration(0.1);
                box_marker.color.g = 1.0;
                box_marker.color.a = 0.5;

                box_marker.pose = object_msg.pose.pose;
                box_marker.scale = object_msg.bounding_box;

                // Update bounding box markers
                object_markers.markers.push_back(box_marker);

		// Make text msg
		visualization_msgs::Marker text_marker;

		text_marker.ns = "text";
                text_marker.id = n_markers+1;
                text_marker.header.frame_id = "velo_link";
                text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text_marker.lifetime = ros::Duration(0.1);
                text_marker.color.g = 1.0;
                text_marker.color.a = 1.0;
		text_marker.pose = box_marker.pose;
		text_marker.pose.position.z = text_marker.pose.position.z + (box_marker.scale.z)/2 + 1.2;
		text_marker.scale.z = 0.7;

		char txt[100];
		sprintf(txt,"%s_%d\nv = %.2f m/s\np = %.2f\nt = %.2f s", obj_type, object_msg.object_id, norm_v, object_msg.certainty, object_msg.lifetime);
		text_marker.text = txt;
		
		// Update text markers
                object_markers.markers.push_back(text_marker);

		// Make arrow markers
		visualization_msgs::Marker arrow_marker;
		
		arrow_marker.ns = "arrows";
                arrow_marker.id = n_markers+2;
                arrow_marker.header.frame_id = "velo_link";
                arrow_marker.type = visualization_msgs::Marker::ARROW;
                arrow_marker.lifetime = ros::Duration(0.1);

		arrow_marker.scale.x = 0.4*norm_v;
  		arrow_marker.scale.y = 0.2;
  		arrow_marker.scale.z = 0.2;

                arrow_marker.color.r = 1.0;
                arrow_marker.color.a = 1.0;
                
		arrow_marker.pose = box_marker.pose;

		// Update arrow markers
                object_markers.markers.push_back(arrow_marker);

		n_markers += 3;

        }

        // Publish objects
        objects_pub_.publish(objects_msg);
	vis_pub_.publish(object_markers);

}

void prediction(std::vector<state>& states){

	// Get Sampling Time, Motion covariance and Survival probability parameters
        float T, cov, P_S;
        ros::param::get("/adda_object_tracking_node/motion_model/sampling_time", T);
        ros::param::get("/adda_object_tracking_node/motion_model/covariance", cov);
	ros::param::get("/adda_object_tracking_node/survival_probability", P_S);

	// Make constant velocity motion model
	Matrix4f F, Q;

	F = Matrix4f::Identity();
	F.topRightCorner(2,2) = Matrix2f::Identity() * T;
        
	Q.topLeftCorner(2,2)     = pow(cov,2)*0.25*pow(T,4)*Matrix2f::Identity();
	Q.topRightCorner(2,2)    = pow(cov,2)*0.50*pow(T,3)*Matrix2f::Identity();
	Q.bottomLeftCorner(2,2)  = Q.topRightCorner(2,2);
	Q.bottomRightCorner(2,2) = pow(cov,2)*pow(T,2)*Matrix2f::Identity();

	for(int n=0; n<states.size(); ++n){
		states[n].x = F * states[n].x;
		states[n].P = F * states[n].P * F.transpose() + Q;
		states[n].w += log(P_S);
		states[n].lifetime += T;
	}

}

namespace adda_object_tracking{
	
GM_PHD_Tracker::GM_PHD_Tracker(const ros::Publisher& vis_pub,
                const ros::Publisher& objects_pub) : vis_pub_(vis_pub), objects_pub_(objects_pub) {}


void GM_PHD_Tracker::Callback(const adda_worldmodel_msgs::Percepts::ConstPtr& percepts_msg){
    
        auto t0 = std::chrono::high_resolution_clock::now();

	// Add birth model to states
        add_birth_model(states_);

        // Get number of states and measurements
	int n_hyp = states_.size();
	ROS_INFO("n_hyp: %d", n_hyp);

        int n_meas = percepts_msg->percepts.size();
        ROS_INFO("Got %d percepts", n_meas);
        
	// Read percepts
        MatrixXf z(2,n_meas);
	VectorXf meas_prob(n_meas);
        read_percepts(percepts_msg,n_meas,z,percepts_);
        
	// Make linear measurement model
	MatrixXf h(2,4);
        Matrix2f R;
        make_meas_model(h,R);
	
	// Kalman Update Step
	float gating_size;
        ros::param::get("/adda_object_tracking_node/GM_PHD_tracker/gating_size", gating_size);

	std::vector<state> upd_states;
	upd_states.reserve(n_hyp*(n_meas+1));
        ArrayXf sum_weights = ArrayXf::Zero(n_meas);
	
	for(int n=0; n<n_hyp; ++n){
		// Update states hypotheses given the measurements
		update_step(upd_states,sum_weights,states_[n],gating_size,
				h,R,n_meas,z,percepts_,total_ids_);
	}

	ROS_INFO("n_hyp before pruning: %d", total_ids_);
        
	// Update weights taking into account sensor noise
        update_weights(upd_states,sum_weights);

	for(int n=0; n<n_hyp; ++n){
		// Include misdetection of states hypotheses
                upd_states.emplace_back(states_[n].id,
					states_[n].lifetime,
					states_[n].x,
                                        states_[n].P,
                                        log(1-states_[n].det_prob) + states_[n].w,
                                        states_[n].det_prob,-1);
	}

        // Get number of states and measurements
        n_hyp = upd_states.size();
        ROS_INFO("n_hyp before pruning: %d", n_hyp);
        
	// Hypothesis Reduction (Pruning + Capping)
        reduction(states_, upd_states);
	
        n_hyp = states_.size();
        ROS_INFO("n_hyp after pruning: %d", n_hyp);

	for(int n=0; n<n_hyp; ++n){
                ROS_INFO("id = %d - life = %f - x = [%f %f %f %f] - P = [%f %f %f %f] - w = %f - p = %f - z = %f", 
                                states_[n].id, states_[n].lifetime,
				states_[n].x(0), states_[n].x(1), states_[n].x(2), states_[n].x(3),
                                states_[n].P(0,0), states_[n].P(1,1), states_[n].P(2,2), states_[n].P(3,3),
                                states_[n].w, states_[n].det_prob,percepts_[n].z_position);
        }

	// Publish Estimates 
        publish_estimates(states_, percepts_, objects_pub_,vis_pub_);

	// Prediction Step
	prediction(states_);

	auto diff=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-t0);
	ROS_INFO("Total tracker Duration of %ld ms.", diff.count());
	ROS_INFO(" ");
}

}
