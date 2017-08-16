/*
 * Steering_Control.hpp
 *
 *  Created on: 1 Aug, 2014
 *      Author: liuwlz
 */

#ifndef STEERING_CONTROL_HPP_
#define STEERING_CONTROL_HPP_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <MPAVUtil/Filter.hpp>

#include <vector>

using namespace std;
using namespace boost;

namespace Control {

	class SteeringControl{

	public:
		SteeringControl();
		virtual ~SteeringControl();

		virtual void initTrackingPath(const nav_msgs::Path& pathIn);
		virtual void calculateSteeringAngle();
		virtual void getTrackingPoint();

		void odomCallBack(const nav_msgs::OdometryConstPtr odomIn);
		void speedCallBack(const geometry_msgs::TwistStampedConstPtr speedIn);
		void trackingPathCallBack(const nav_msgs::PathConstPtr speedIn);

		void resetControlLoop();
		void getVehiclePose();
		void getDistToGoal();
		void steerControlTimer(const ros::TimerEvent& e);
		void boundSteeringAngle(double& steeringIn);
		void steerAnglePub();

		void transformToBaseFrame(geometry_msgs::PoseStamped& poseIn, geometry_msgs::PoseStamped& poseOut);
		double distBetweenPose(const geometry_msgs::PoseStamped& poseA, const geometry_msgs::PoseStamped& poseB);
		double getYawOfPose(const geometry_msgs::PoseStamped& poseIn);

		ros::NodeHandle nh_, pri_nh_;
		ros::Timer steer_control_timer_;
		ros::Publisher tracking_pose_pub_, cmd_steer_pub_, simulate_pose_pub_;
		ros::Subscriber speed_sub_, odom_sub_, tracking_path_sub_;

		tf::TransformListener tf_;
		geometry_msgs::PoseStamped vehicle_pose_, tracking_pose_;

		string base_frame_, global_frame_;

		boost::shared_ptr<Util::LowPassFilter> vel_filter_;
		ros::Time odom_filter_time_;

		nav_msgs::Path simulate_path_;
		nav_msgs::PathPtr tracking_path_;
		geometry_msgs::TwistStamped cmd_steer_;
		double steering_angle_, dist_to_goal_, vehicle_vel_, odom_vel_;
		bool tracking_path_got_, cmd_vel_got_, over_tracking_error_;
		int path_sequence_;

	};
}  // namespace Control


#endif /* STEERING_CONTROL_HPP_ */
