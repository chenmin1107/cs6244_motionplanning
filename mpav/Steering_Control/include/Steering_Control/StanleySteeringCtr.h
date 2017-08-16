/*
 * StanleySteeringCtr.h
 *
 *  Created on: Apr 6, 2014
 *      Author: liuwlz
 */

#ifndef STANLEYSTEERINGCTR_H_
#define STANLEYSTEERINGCTR_H_

#include <math.h>

#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace boost;

#define REAR_DIST 1.2

namespace MPAV{

	typedef geometry_msgs::PoseStamped pose_t;
	typedef geometry_msgs::PoseStampedPtr pose_ptr;
	typedef geometry_msgs::Point32 point_t;
	typedef geometry_msgs::Point32Ptr point_ptr;
	typedef nav_msgs::Path path_t;
	typedef nav_msgs::PathPtr path_ptr;

	class StanleySteeringCtr{

		ros::NodeHandle nh_, private_nh_;
		ros::Publisher tracking_pose_pub_, front_axis_pub_;
		ros::Publisher vel_cmd_pub_;
		ros::Subscriber path_sub_;
		ros::Timer control_timer_;

		tf::TransformListener tf_;

		typedef enum{Moving, Stop, GoalReach}CONTROL_STATUS;
		CONTROL_STATUS control_status_;

		int path_callback_counter_;
		int path_sequence_;
		double dist_to_goal_;
		double turning_angle_;
		double vehicle_vel_;
		double path_length_;
		pose_t vehicle_pose_, tracking_pose_;
		path_ptr path_;

		double tracking_gain_, goal_region_threshold_;

		string global_frame_, base_frame_;

		int analyzeStatus();
		int idleDriving();
		int forwardDriving();
		int backwardDriving();

		int getVehiclePose();
		int getDistToGoal();
		int getTrackingPoint();
		int calculateSteeringAngle();
		int boundSteeringAngle(double& steeringIn);

		/**
		 * For testing only
		 */
		geometry_msgs::Twist cmd_vel_;
		int analyzeCmdVel();

		void pathCallBack(const path_ptr& pathIn);
		void controlLoop(const ros::TimerEvent& e);

		double getPathLength(const path_ptr& pathIn);

		double yawDiffBetweenPose(const pose_t& poseA, const pose_t& poseB);
		inline double getYawOfPose(const pose_t& poseIn);
		inline double distBetweenPose(const pose_t& poseA, const pose_t& poseB);

		int resetControlLoop();
		int transformToBaseFrame(const pose_t& poseIn, pose_t& poseOut);

	public:
		StanleySteeringCtr();
		virtual ~StanleySteeringCtr();
	};
}



#endif /* STANLEYSTEERINGCTR_H_ */
