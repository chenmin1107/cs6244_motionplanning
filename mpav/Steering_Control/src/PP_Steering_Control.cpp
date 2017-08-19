/*
 * PP_Steering_Control.cpp
 *
 *  Created on: 1 Aug, 2014
 *      Author: liuwlz
 */


#include <Steering_Control/PP_Steering_Control.hpp>

namespace Control {
	PPSteeringCtr::
	PPSteeringCtr():
	SteeringControl(),
	tracking_pts_got_(false),
	simulate_pose_init_(false){
		pri_nh_.param("look_ahead_dist", look_ahead_dist_,2.0);
		pri_nh_.param("car_length", car_length_,2.2);
		pri_nh_.param("simulate_dist",simulate_dist_,10.0);
		pri_nh_.param("simulate_increment",simulate_increment_,0.2);
		pri_nh_.param("neighbour_waypoint_size",neighbour_waypoint_size_,5);

		purepuisuit_path_pub_ = nh_.advertise<nav_msgs::Path>("pp_path",1);
		modified_path_pub_ = nh_.advertise<nav_msgs::Path>("modified_path",1);
	}

	PPSteeringCtr::
	~PPSteeringCtr(){
	}

	void
	PPSteeringCtr::
	initTrackingPath(const nav_msgs::Path& pathIn){
		if (Debug)
			cout <<"Init Tracking Path with size: "<< pathIn.poses.size() <<endl;
		*tracking_path_= pathIn;
		densePathSegment();
		resetControlLoop();
		if (Debug)
			cout <<"Dense Tracking Path with size: "<< tracking_path_->poses.size() <<endl;
		tracking_path_got_ = true;
	}

	void
	PPSteeringCtr::
	densePathSegment(){
		vector<geometry_msgs::PoseStamped> path_dense_tmp = tracking_path_->poses;
		tracking_path_->poses.clear();
		for (vector<geometry_msgs::PoseStamped>::iterator iter = path_dense_tmp.begin();
				iter != path_dense_tmp.end()-1;
				iter ++){
			tracking_path_->poses.push_back(*iter);
			int path_seg_size = (int)(distBetweenPose(*iter,*(iter+1)) / look_ahead_dist_) + 1;
			if (path_seg_size > 1){
				vector<geometry_msgs::PoseStamped> pose_set_insert(path_seg_size - 1);
				for (vector<geometry_msgs::PoseStamped>::iterator iter_insert = pose_set_insert.begin();
						iter_insert != pose_set_insert.end();
						iter_insert ++){
					int curr_index = iter_insert - pose_set_insert.begin() + 1;
					iter_insert->pose.position.x = iter->pose.position.x +
							(double)curr_index/(double)path_seg_size *
							((iter+1)->pose.position.x - iter->pose.position.x);
					iter_insert->pose.position.y = iter->pose.position.y +
							(double)curr_index/(double)path_seg_size *
							((iter+1)->pose.position.y - iter->pose.position.y);
				}
				tracking_path_->poses.insert(tracking_path_->poses.end() ,pose_set_insert.begin(),pose_set_insert.end());
			}
		}
		tracking_path_->poses.push_back(path_dense_tmp.back());
		purepuisuit_path_pub_.publish(*tracking_path_);
	}

	void
	PPSteeringCtr::
	combineTrackingPath(nav_msgs::Path insertPath, const nav_msgs::Path& remainPath){
		insertPath.poses.insert(insertPath.poses.end(),remainPath.poses.begin(), remainPath.poses.end());
		*tracking_path_ = insertPath;
		purepuisuit_path_pub_.publish(*tracking_path_);
		resetControlLoop();
	}

	nav_msgs::Path
	PPSteeringCtr::
	combineModifiedPath(nav_msgs::Path insertPath, const nav_msgs::Path& remainPath){
		insertPath.poses.insert(insertPath.poses.end(),remainPath.poses.begin(), remainPath.poses.end());
		modified_path_pub_.publish(insertPath);
		return insertPath;
	}

	double
	PPSteeringCtr::
	boundAnglePNPI(double angleIn){
		while (angleIn < -M_PI)
			angleIn += 2 * M_PI;
		while (angleIn > M_PI)
			angleIn -= 2*M_PI;
		return angleIn;
	}

	void
	PPSteeringCtr::
	getTrackingPoint(){
		tracking_pts_got_ = false;
		//If the lookahead dist is larger than the distance to goal, then set the last waypoint as tracking pose.
		if (distBetweenPose(vehicle_pose_, tracking_path_->poses.back())<look_ahead_dist_){
			tracking_pose_ = tracking_path_->poses.back();
			if (!simulate_pose_init_)
				tracking_pose_pub_.publish(tracking_pose_);
			tracking_pts_got_= true;
			return;
		}
		//This is to handle the issue of localization drift, the solution is searching the tracking poits start from the neighboor instead.
		int initial_check_index = path_sequence_ >= neighbour_waypoint_size_ ? path_sequence_ - neighbour_waypoint_size_ : path_sequence_;
		for (vector<geometry_msgs::PoseStamped>::iterator iter = tracking_path_->poses.begin() + initial_check_index;
				iter != tracking_path_->poses.end()-1;
				iter ++){
			double curr_dist_diff = distBetweenPose(vehicle_pose_, *iter) - look_ahead_dist_;
			double next_dist_diff = distBetweenPose(vehicle_pose_, *(iter+1)) - look_ahead_dist_;
			double yaw_diff = boundAnglePNPI(getYawOfPose(vehicle_pose_)) -
					boundAnglePNPI(atan2((iter+1)->pose.position.y - iter->pose.position.y, (iter+1)->pose.position.x-iter->pose.position.x));
			if (Debug && fabs(curr_dist_diff)<0)
				cout <<"Curr_dist: "<< curr_dist_diff << " Next dist: "<< next_dist_diff<< " Angle dist: "<< yaw_diff<<endl;
			if (curr_dist_diff <= 0 && next_dist_diff > 0 && fabs(boundAnglePNPI(yaw_diff)) < M_PI/1.5 ){
				tracking_segment_= make_pair(*iter, *(iter+1));
				path_sequence_ = iter - tracking_path_->poses.begin();
				break;
			}
		}

		if (getLineCircleIntersection()){
			tracking_pose_.header.frame_id = global_frame_;
			tracking_pose_.header.stamp = ros::Time::now();
			if (!simulate_pose_init_)
				tracking_pose_pub_.publish(tracking_pose_);
			tracking_pts_got_= true;
		}
		else{
			tracking_pts_got_= false; 
			if (!simulate_pose_init_){
				cmd_steer_.twist.linear.x = 0.00;
				ROS_WARN("No tracking pose found, stop for safety");
			}
			tracking_pose_.pose.orientation.w = 1.0;
			tracking_pose_.header.frame_id = global_frame_;
			tracking_pose_.header.stamp = ros::Time::now();
		}
	}

	bool
	PPSteeringCtr::
	getLineCircleIntersection(){
	    //http://stackoverflow.com/questions/1073336/circle-line-collision-detection
	    double Ex = tracking_segment_.first.pose.position.x, Ey = tracking_segment_.first.pose.position.y;
	    double Lx = tracking_segment_.second.pose.position.x, Ly = tracking_segment_.second.pose.position.y;
	    double Cx = vehicle_pose_.pose.position.x, Cy = vehicle_pose_.pose.position.y;
	    double r = look_ahead_dist_;
	    double dx = Lx - Ex, dy = Ly - Ey;
	    double fx = Ex - Cx, fy = Ey - Cy;

	    double a = dx * dx + dy * dy;
	    double b = 2 * (fx * dx + fy * dy);
	    double c = (fx * fx + fy * fy) - (r * r);

	    double discriminant = b*b-4*a*c;
	    if (discriminant<0)
	    	return false;

	    discriminant = sqrt(discriminant);
	    double t1 = (-b + discriminant)/(2*a);

	    if(t1 >=0 && t1 <=1){
	        tracking_pose_.pose.position.x = Ex+t1*dx;
	        tracking_pose_.pose.position.y = Ey+t1*dy;
			double tracking_pose_yaw = atan2(tracking_path_->poses[path_sequence_+1].pose.position.y - tracking_path_->poses[path_sequence_].pose.position.y,
				        		tracking_path_->poses[path_sequence_+1].pose.position.x-tracking_path_->poses[path_sequence_].pose.position.x);
			tf::Quaternion q;
			q.setRPY(0, 0, tracking_pose_yaw);
			tf::quaternionTFToMsg(q, tracking_pose_.pose.orientation);
	        return true;
	    }
	    else
	    	return false;
	}

	void
	PPSteeringCtr::
	calculateSteeringAngle(){
		getTrackingPoint();
		if (!tracking_pts_got_){
			steering_angle_ =0.0;
			return;
		}
		double theta = atan2(tracking_pose_.pose.position.y - vehicle_pose_.pose.position.y,
				tracking_pose_.pose.position.x - vehicle_pose_.pose.position.x) - getYawOfPose(vehicle_pose_);
		steering_angle_ = atan(2*car_length_*sin(theta)/look_ahead_dist_);
		boundSteeringAngle(steering_angle_);
		if (!simulate_pose_init_)
			simulateVehiclePose();
	}

	//TODO: Improve the simulate path when vehicle approach stations or reach subgoal.
	void
	PPSteeringCtr::
	simulateVehiclePose(){
		simulate_pose_init_ = true;
		int simulate_seg_size = ( (dist_to_goal_ + car_length_) > simulate_dist_) ? (simulate_dist_/simulate_increment_) : ((dist_to_goal_ + car_length_)/simulate_increment_);
		int path_seq_copy = path_sequence_;
		double steering_anle_copy = steering_angle_;
		geometry_msgs::PoseStamped vehicle_pose_copy = vehicle_pose_;
		geometry_msgs::PoseStamped trakcing_pose_copy = tracking_pose_;

		simulate_path_.poses.clear();
		simulate_path_.poses.resize(simulate_seg_size);

		if (Debug)
			cout << "Start Prediction"<<endl;
		for (vector<geometry_msgs::PoseStamped>::iterator iter = simulate_path_.poses.begin();
				iter != simulate_path_.poses.end();
				iter++){
			*iter = vehicle_pose_;
			getTrackingPoint();
			calculateSteeringAngle();
			kinematicPredict();
		}
		simulate_path_.header.frame_id = global_frame_;
		simulate_path_.header.stamp = ros::Time::now();
		simulate_pose_pub_.publish(simulate_path_);

		path_sequence_ = path_seq_copy;
		steering_angle_= steering_anle_copy;
		vehicle_pose_ = vehicle_pose_copy;
		tracking_pose_ = trakcing_pose_copy;
		simulate_pose_init_ = false;
	}

	void
	PPSteeringCtr::
	kinematicPredict(){
		if (Debug)
			cout <<"Turning Radius" << steering_angle_<<endl;
		double radius = fabs(car_length_/tan(steering_angle_));
		double alpha = steering_angle_/fabs(steering_angle_)*simulate_increment_/radius;
		double base_x = fabs(radius*sin(alpha));
		double base_y = alpha/fabs(alpha)*radius*(1-cos(alpha));
		double vehicle_yaw = getYawOfPose(vehicle_pose_);

		vehicle_pose_.pose.position.x = vehicle_pose_.pose.position.x + base_x*cos(vehicle_yaw) - base_y*sin(vehicle_yaw);
		vehicle_pose_.pose.position.y = vehicle_pose_.pose.position.y + base_x*sin(vehicle_yaw) + base_y*cos(vehicle_yaw);
		double new_yaw = vehicle_yaw + alpha;

		tf::Quaternion q;
		q.setRPY(0, 0, new_yaw);
		tf::quaternionTFToMsg(q, vehicle_pose_.pose.orientation);

		if (Debug)
			ROS_INFO("Base_x, %f, Base_y. %f, yaw, %f, new, %f, new, %f",base_x, base_y, vehicle_yaw, vehicle_pose_.pose.position.x, vehicle_pose_.pose.position.y);
	}

}  // namespace Control
