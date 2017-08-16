/*
 * PP_Steering_Control.hpp
 *
 *  Created on: 1 Aug, 2014
 *      Author: liuwlz
 */

#ifndef PP_STEERING_CONTROL_HPP_
#define PP_STEERING_CONTROL_HPP_

#include <Steering_Control/Steering_Control.hpp>

#define Debug false

namespace Control {
	typedef pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> LineSegment;

	class PPSteeringCtr:public SteeringControl{
	public:
		PPSteeringCtr();
		virtual ~PPSteeringCtr();

		void simulateVehiclePose();
		void initTrackingPath(const nav_msgs::Path& pathIn);
		nav_msgs::Path combineModifiedPath(nav_msgs::Path insertPath, const nav_msgs::Path& remainPath);
		void combineTrackingPath(nav_msgs::Path insertPath, const nav_msgs::Path& remainPath);
		void setLookAheadDist(double lookAheadDistIn){look_ahead_dist_ = lookAheadDistIn;};

	private:

		ros::Publisher purepuisuit_path_pub_, modified_path_pub_;

		double look_ahead_dist_, car_length_, simulate_dist_, simulate_increment_;
		int neighbour_waypoint_size_;
		LineSegment tracking_segment_;
		bool tracking_pts_got_, simulate_pose_init_;

		void getTrackingPoint();
		void calculateSteeringAngle();
		bool getLineCircleIntersection();
		void densePathSegment();
		double boundAnglePNPI(double angleIn);

		void kinematicPredict();
		void calculateSteeringAngleSimulate();
	};
}  // namespace Control


#endif /* PP_STEERING_CONTROL_HPP_ */
