/*
 * Steering_Control.cpp
 *
 *  Created on: 1 Aug, 2014
 *      Author: liuwlz
 */

#include <Steering_Control/Steering_Control.hpp>

namespace Control {
	SteeringControl::
	SteeringControl():
	pri_nh_("~"),
	odom_filter_time_(ros::Time::now()),
	steering_angle_(0.0),
	dist_to_goal_(DBL_MAX),
	vehicle_vel_(1.0),
	odom_vel_(0.0),
	tracking_path_got_(false),
	cmd_vel_got_(false),
	over_tracking_error_(false),
	path_sequence_(0){
		tracking_path_ = boost::shared_ptr<nav_msgs::Path>(new nav_msgs::Path());

		speed_sub_ = nh_.subscribe("cmd_sm",1,&SteeringControl::speedCallBack,this);
		odom_sub_ = nh_.subscribe("odom",1,&SteeringControl::odomCallBack,this);
		tracking_path_sub_ = nh_.subscribe("tracking_path",1,&SteeringControl::trackingPathCallBack,this);
		tracking_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("tracking_pose",1);
		cmd_steer_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_steer",1);
		steer_control_timer_ = nh_.createTimer(ros::Duration(0.02),&SteeringControl::steerControlTimer, this);
		simulate_pose_pub_ = nh_.advertise<nav_msgs::Path>("simulate_path",1);

		pri_nh_.param("global_frame",global_frame_,string("map"));
		pri_nh_.param("base_frame",base_frame_,string("base_link"));

		vel_filter_ = boost::shared_ptr<Util::LowPassFilter>(new Util::LowPassFilter(0.1));
	}

	SteeringControl::
	~SteeringControl(){

	}

	void
	SteeringControl::
	initTrackingPath(const nav_msgs::Path& pathIn){
		*tracking_path_ = pathIn;
		resetControlLoop();
		tracking_path_got_ = true;
	}

	void
	SteeringControl::
	resetControlLoop(){
		ROS_INFO("Reset Control Loop");
		dist_to_goal_ = DBL_MAX;
		path_sequence_ = 0;
		over_tracking_error_ = false;
	}

	void
	SteeringControl::
	getTrackingPoint(){

	}

	void
	SteeringControl::
	calculateSteeringAngle(){

	}

	void
	SteeringControl::
	boundSteeringAngle(double& steeringIn){
		while (steeringIn > M_PI)
			steeringIn -= 2.0*M_PI;
		while (steeringIn < -M_PI)
			steeringIn += 2.0*M_PI;

		steeringIn = steeringIn > M_PI/4.0 ? M_PI/4.0 : steeringIn;
		steeringIn = steeringIn < -M_PI/4.0 ? -M_PI/4.0 : steeringIn;
	}

	void
	SteeringControl::
	speedCallBack(const geometry_msgs::TwistStampedConstPtr speedIn){
		vehicle_vel_ = speedIn->twist.linear.x;
		cmd_steer_ = *speedIn;
		cmd_vel_got_ = true;
	}

	void
	SteeringControl::
	odomCallBack(const nav_msgs::OdometryConstPtr odomIn){
		ros::Time time_now = ros::Time::now();
		odom_vel_ = fabs(vel_filter_->filter_dt((time_now - odom_filter_time_).toSec(), odomIn->twist.twist.linear.x));
		odom_filter_time_ = time_now;
	}

	void
	SteeringControl::
	trackingPathCallBack(const nav_msgs::PathConstPtr pathIn){
		initTrackingPath(*pathIn);
	}

	void
	SteeringControl::
	steerControlTimer(const ros::TimerEvent& e){
		if (true)
			cmd_vel_got_ = true;
		if (!tracking_path_got_ || !cmd_vel_got_){
			return;
		}
		getVehiclePose();
		calculateSteeringAngle();
		steerAnglePub();
		getDistToGoal();
		ROS_INFO_ONCE("Steering Control Spinning");
	}

	void
	SteeringControl::
	steerAnglePub(){
		cmd_steer_.twist.angular.z = steering_angle_;
		cmd_steer_.header.stamp = ros::Time::now();
		cmd_steer_pub_.publish(cmd_steer_.twist);
	}

	void
	SteeringControl::
	getVehiclePose(){
		vehicle_pose_.header.stamp = ros::Time::now();
		tf::Stamped<tf::Pose> global_pose, base_pose;
		global_pose.setIdentity();
		base_pose.setIdentity();
		base_pose.frame_id_ = base_frame_;
		base_pose.stamp_ = ros::Time();

		try {
			tf_.waitForTransform(global_frame_, base_frame_,ros::Time(0),ros::Duration(0.01));
			tf_.transformPose(global_frame_, base_pose, global_pose);
		}
		catch(tf::LookupException& ex) {
			ROS_ERROR("No Transform available Error: %s", ex.what());
		}
		catch(tf::ConnectivityException& ex) {
			ROS_ERROR("Connectivity Error: %s", ex.what());
		}
		catch(tf::ExtrapolationException& ex) {
			ROS_ERROR("Extrapolation Error: %s", ex.what());
		}
		tf::poseStampedTFToMsg(global_pose, vehicle_pose_);
	}

	void
	SteeringControl::
	transformToBaseFrame(geometry_msgs::PoseStamped& poseIn, geometry_msgs::PoseStamped& poseOut){
		poseIn.header.stamp = ros::Time();
		poseOut.header.stamp = ros::Time();
		poseOut.header.frame_id = base_frame_;
		try {
			tf_.waitForTransform(base_frame_, global_frame_,ros::Time(0),ros::Duration(0.01));
			tf_.transformPose(base_frame_, poseIn, poseOut);
		}
		catch(tf::LookupException& ex) {
			ROS_ERROR("No Transform available Error: %s", ex.what());
		}
		catch(tf::ConnectivityException& ex) {
			ROS_ERROR("Connectivity Error: %s", ex.what());
		}
		catch(tf::ExtrapolationException& ex) {
			ROS_ERROR("Extrapolation Error: %s", ex.what());
		}
	}

	void
	SteeringControl::
	getDistToGoal(){
		dist_to_goal_ = 0.0;
		for (vector<geometry_msgs::PoseStamped>::iterator iter = tracking_path_->poses.begin() + path_sequence_ + 1;
				iter != tracking_path_->poses.end()-1;
				iter ++){
			dist_to_goal_ += distBetweenPose(*iter, *(iter+1));
		}
		dist_to_goal_ += distBetweenPose(vehicle_pose_, tracking_path_->poses[path_sequence_+1]);
	}

	double
	SteeringControl::
	distBetweenPose(const geometry_msgs::PoseStamped& poseA, const geometry_msgs::PoseStamped& poseB){
		double x_A = poseA.pose.position.x, y_A = poseA.pose.position.y;
		double x_B = poseB.pose.position.x, y_B = poseB.pose.position.y;
		return sqrt((x_A-x_B)*(x_A-x_B)+(y_A-y_B)*(y_A-y_B));
	}

	double
	SteeringControl::
	getYawOfPose(const geometry_msgs::PoseStamped& poseIn){
		double roll=0.0, pitch=0.0, yaw=0.0;
		tf::Quaternion quaternion;
		tf::quaternionMsgToTF(poseIn.pose.orientation, quaternion);
		tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
		return(yaw);
	}

}  // namespace Control
