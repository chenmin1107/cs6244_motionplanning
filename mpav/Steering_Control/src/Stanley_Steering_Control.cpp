/*
 * Stanley_Steering_Control.cpp
 *
 *  Created on: 1 Aug, 2014
 *      Author: liuwlz
 */


#include <Steering_Control/Stanley_Steering_Control.hpp>

namespace Control {
	StanleySteeringCtr::
	StanleySteeringCtr():
	SteeringControl(),
	direction_flag_(1){
		pri_nh_.param("tracking_gain",tracking_gain_, 5.0);
		pri_nh_.param("track_error_tolerance",track_error_tolerance_,1.0);
	}

	StanleySteeringCtr::
	~StanleySteeringCtr(){

	}

	void
	StanleySteeringCtr::
	getTrackingPoint(){
		double curr_dist = 0.0, next_dist = 0.0;
		for (vector<geometry_msgs::PoseStamped>::iterator iter = tracking_path_->poses.begin()+path_sequence_;
				iter != tracking_path_->poses.end()-1; iter ++){
			curr_dist = distBetweenPose(vehicle_pose_, *iter);
			next_dist = distBetweenPose(vehicle_pose_, *(iter+1));
			if (next_dist >= curr_dist){
				if (((iter+1)->pose.position.z != iter->pose.position.z) ||
						fabs(next_dist - curr_dist) < 1e-8){
					tracking_pose_ = *(iter + 1);
					path_sequence_ = iter - tracking_path_->poses.begin() + 1;
					break;
				}
				else{
					tracking_pose_ = *iter;
					path_sequence_ = iter - tracking_path_->poses.begin();
					break;
				}
			}
		}
		tracking_pose_.header.frame_id = global_frame_;
		tracking_pose_.header.stamp = ros::Time::now();
		tracking_pose_pub_.publish(tracking_pose_);
	}

	void
	StanleySteeringCtr::
	calculateSteeringAngle(){
		getTrackingPoint();
		geometry_msgs::PoseStamped track_pose_base;
		tracking_pose_.header.stamp = ros::Time::now();
		transformToBaseFrame(tracking_pose_, track_pose_base);
		double track_err = distBetweenPose(tracking_pose_, vehicle_pose_);

#if true
		if (track_err > track_error_tolerance_){
			ROS_WARN("Stanley tracking error is over tolerance, triggering for replanning");
			//TODO:Optimize the speed control instead of emergency stop
			cmd_steer_.twist.linear.x = 0.0;
			steering_angle_ = 0.0;
			over_tracking_error_ = true;
			return;
		}
#endif

		int error_flag = track_pose_base.pose.position.y > 0 ? 1 : -1;
		double yaw_diff = getYawOfPose(track_pose_base);
		double cross_track_err = track_pose_base.pose.position.y;
		//Forward Driving
		if (tracking_pose_.pose.position.z == 1 ){
			direction_flag_ = 1;
			cmd_steer_.twist.linear.x = vehicle_vel_;
			steering_angle_ = yaw_diff + atan(tracking_gain_*cross_track_err/(abs(odom_vel_)+1e-5));
		}
		//Backward Driving
		else{
			direction_flag_ = 0;
			cmd_steer_.twist.linear.x = - vehicle_vel_;
			steering_angle_ = -yaw_diff + atan(tracking_gain_*cross_track_err/(abs(odom_vel_)+1e-5));
		}
		boundSteeringAngle(steering_angle_);
		//ROS_INFO("Steering angle: %f deg, yaw_diff: %f, cross_check_error: %f", steering_angle_*180.0/M_PI,yaw_diff,cross_track_err);
	}

}  // namespace Control
