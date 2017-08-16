/*
 * StanleySteeringCtr.cpp
 *
 *  Created on: Apr 6, 2014
 *      Author: liuwlz
 */


#include <Steering_Control/StanleySteeringCtr.h>

namespace MPAV{
	StanleySteeringCtr::
	StanleySteeringCtr():
	private_nh_("~"),
	control_status_(Stop),
	path_callback_counter_(0),
	path_sequence_(0),
	dist_to_goal_(DBL_MAX),
	turning_angle_(0.0),
	vehicle_vel_(1.0),
	path_length_(0.0){

		private_nh_.param("goal_region_threshold", goal_region_threshold_, 0.5);
		private_nh_.param("tracking_ratio", tracking_gain_, 5.0);
		private_nh_.param("global_frame", global_frame_, string("map"));
		private_nh_.param("base_frame", base_frame_, string("base_link"));

		front_axis_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("/front_axis",1);
		tracking_pose_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("/tracking_pose",1);

		/**
		 * For testing only
		 */
		vel_cmd_pub_ = private_nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1);

		path_sub_ = private_nh_.subscribe("/trajectory_control",1,&StanleySteeringCtr::pathCallBack, this);
		control_timer_ = private_nh_.createTimer(ros::Duration(0.05), &StanleySteeringCtr::controlLoop, this);
	}

	StanleySteeringCtr::
	~StanleySteeringCtr(){

	}

	void
	StanleySteeringCtr::
	controlLoop(const ros::TimerEvent& e){
		analyzeStatus();
		switch (control_status_) {
			case Stop:
				ROS_INFO_ONCE("Stop");
				this->idleDriving();
				break;
			case Moving:
				ROS_INFO_ONCE("Moving");
				this->getVehiclePose();
				break;
			case GoalReach:
				ROS_INFO_ONCE("GoalReach");
				this->idleDriving();
				break;
			default:
				break;
		}
		vel_cmd_pub_.publish(cmd_vel_);
	}

	int
	StanleySteeringCtr::
	idleDriving(){
		cmd_vel_.linear.x = 0.0;
		cmd_vel_.angular.z = 0.0;
		return 1;
	}

	int
	StanleySteeringCtr::
	analyzeStatus(){
		if (dist_to_goal_ < goal_region_threshold_){
			control_status_ = GoalReach;
			return 1;
		}
		return 1;
	}

	void
	StanleySteeringCtr::
	pathCallBack(const path_ptr& pathIn){
		path_callback_counter_ ++;
		if(path_callback_counter_<3)
			return;
		if (path_ == NULL){
			path_ = pathIn;
			this->resetControlLoop();
			ROS_INFO("Update NULL path, path_size");
			return;
		}
		if (fabs(path_length_-getPathLength(pathIn) )>1e-6){
			this->resetControlLoop();
			ROS_INFO("Update new path, previous_length: %f, curr_length: %f", path_length_, getPathLength(pathIn));
		}
		path_ = pathIn;
	}

	int
	StanleySteeringCtr::
	resetControlLoop(){
		dist_to_goal_ = DBL_MAX;
		path_sequence_ = 0;
		control_status_ = Moving;
		path_callback_counter_ = 0;
		return 1;
	}

	double
	StanleySteeringCtr::
	getPathLength(const path_ptr& pathIn){
		double path_length= 0.0;
		for (vector<pose_t>::iterator pose_it = pathIn->poses.begin(); pose_it != pathIn->poses.end()-1; pose_it++){
			path_length += distBetweenPose(*pose_it, *(pose_it + 1));
		}
		path_length_ = path_length;
		return path_length;
	}

	int
	StanleySteeringCtr::
	calculateSteeringAngle(){
		pose_t track_pose_tmp;
		tracking_pose_.header.stamp = ros::Time::now();
		transformToBaseFrame(tracking_pose_, track_pose_tmp);
		int error_flag = track_pose_tmp.pose.position.y > 0 ? 1 : -1;
		double yaw_diff = getYawOfPose(track_pose_tmp);
		double cross_track_err = distBetweenPose(tracking_pose_, vehicle_pose_)* error_flag;
		double steering_angle;
		//Forward Driving
		if (tracking_pose_.pose.position.z == 1 ){
			cmd_vel_.linear.x = vehicle_vel_;
			steering_angle = yaw_diff + atan(tracking_gain_*cross_track_err/(abs(vehicle_vel_)+1e-5));
		}
		//Backward Driving
		else{
			cmd_vel_.linear.x = -vehicle_vel_;
			steering_angle = -yaw_diff + atan(tracking_gain_*cross_track_err/(abs(vehicle_vel_)+1e-5));
		}

		turning_angle_ = steering_angle;
		boundSteeringAngle(turning_angle_);
		ROS_INFO("Steering angle: %f deg, yaw_diff: %f, cross_check_error: %f", turning_angle_*180.0/M_PI,yaw_diff,cross_track_err);

		cmd_vel_.angular.z = turning_angle_;
		return 1;
	}

	int
	StanleySteeringCtr::
	boundSteeringAngle(double& steeringIn){
		while (steeringIn > M_PI)
			steeringIn -= 2.0*M_PI;
		while (steeringIn < -M_PI)
			steeringIn += 2.0*M_PI;

		steeringIn = steeringIn > M_PI/4.0 ? M_PI/4.0 : steeringIn;
		steeringIn = steeringIn < -M_PI/4.0 ? -M_PI/4.0 : steeringIn;

		return 0;
	}

	/*Approximate the distance*/
	int
	StanleySteeringCtr::
	getDistToGoal(){
		dist_to_goal_ = 0.0;
		for (vector<pose_t>::iterator pose_it = path_->poses.begin() + path_sequence_; pose_it != path_->poses.end()-1; pose_it++){
			dist_to_goal_ += distBetweenPose(*pose_it, *(pose_it + 1));
		}
		dist_to_goal_ += distBetweenPose(vehicle_pose_, tracking_pose_);
		ROS_INFO("Distance to Goal: %f", dist_to_goal_);
		return 1.0;
	}

	/*Check the distance change from the last tracking path point*/
	int
	StanleySteeringCtr::
	getTrackingPoint(){
		double curr_dist = 0.0, next_dist = 0.0;
		for (vector<pose_t>::iterator pose_it = path_->poses.begin()+path_sequence_; pose_it != path_->poses.end()-1; pose_it ++){
			curr_dist = distBetweenPose(vehicle_pose_, *pose_it);
			next_dist = distBetweenPose(vehicle_pose_, *(pose_it+1));
			if (next_dist >= curr_dist){
				if (((*(pose_it+1)).pose.position.z != (*(pose_it)).pose.position.z) ||
						fabs(next_dist - curr_dist) < 1e-8){
					tracking_pose_ = *(pose_it + 1);
					path_sequence_ = pose_it - path_->poses.begin() + 1;
					break;
				}
				else{
					tracking_pose_ = *pose_it;
					path_sequence_ = pose_it - path_->poses.begin();
					break;
				}
			}
		}
		this->getDistToGoal();
		this->calculateSteeringAngle();
		tracking_pose_.header.frame_id = global_frame_;
		tracking_pose_.header.stamp = ros::Time::now();
		tracking_pose_pub_.publish(tracking_pose_);
		return 1;
	}

	double
	StanleySteeringCtr::
	yawDiffBetweenPose(const pose_t& posePath, const pose_t& poseVehicle){
		double yaw_path = getYawOfPose(posePath), yaw_vehicle = getYawOfPose(poseVehicle);
		return (yaw_vehicle-yaw_path);
	}

	inline double
	StanleySteeringCtr::
	getYawOfPose(const pose_t& poseIn){
		double roll=0.0, pitch=0.0, yaw=0.0;
		tf::Quaternion quaternion;
		tf::quaternionMsgToTF(poseIn.pose.orientation, quaternion);
		tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
		return(yaw);
	}

	inline double
	StanleySteeringCtr::
	distBetweenPose(const pose_t& poseA, const pose_t& poseB){
		double x_A = poseA.pose.position.x, y_A = poseA.pose.position.y;
		double x_B = poseB.pose.position.x, y_B = poseB.pose.position.y;
		return sqrt((x_A-x_B)*(x_A-x_B)+(y_A-y_B)*(y_A-y_B));
	}

	int
	StanleySteeringCtr::
	getVehiclePose(){
		tf::Stamped<tf::Pose> vehicle_pose, base_pose;
		vehicle_pose.setIdentity();
		base_pose.setIdentity();
		base_pose.frame_id_ = base_frame_;
		base_pose.stamp_ = ros::Time();

		try {
			tf_.waitForTransform(global_frame_, base_frame_,ros::Time::now(),ros::Duration(0.01));
			tf_.transformPose(global_frame_, base_pose, vehicle_pose);
		}
		catch(tf::LookupException& ex) {
			ROS_ERROR("No Transform available Error: %s", ex.what());
			return 0;
		}
		catch(tf::ConnectivityException& ex) {
			ROS_ERROR("Connectivity Error: %s", ex.what());
			return 0;
		}
		catch(tf::ExtrapolationException& ex) {
			ROS_ERROR("Extrapolation Error: %s", ex.what());
			return 0;
		}

		tf::poseStampedTFToMsg(vehicle_pose, vehicle_pose_);
		front_axis_pub_.publish(vehicle_pose_);
		this->getTrackingPoint();
		return 1;
	}

	int
	StanleySteeringCtr::
	transformToBaseFrame(const pose_t& poseIn, pose_t& poseOut){
		poseOut.header.stamp = ros::Time::now();
		poseOut.header.frame_id = base_frame_;
		try {
			tf_.waitForTransform(base_frame_, global_frame_,ros::Time::now(),ros::Duration(0.01));
			tf_.transformPose(base_frame_, poseIn, poseOut);
		}
		catch(tf::LookupException& ex) {
			ROS_ERROR("No Transform available Error: %s", ex.what());
			return 0;
		}
		catch(tf::ConnectivityException& ex) {
			ROS_ERROR("Connectivity Error: %s", ex.what());
			return 0;
		}
		catch(tf::ExtrapolationException& ex) {
			ROS_ERROR("Extrapolation Error: %s", ex.what());
			return 0;
		}
		return 1;
	}
}
