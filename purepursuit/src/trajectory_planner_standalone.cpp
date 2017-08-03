#include <math.h>

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseArray.h>
#include <purepursuit/purepursuit.h>
#include <pnc_msgs/move_status.h>
#include <pnc_msgs/poi.h>
#include <fmutil/fm_math.h>
#include <StationPath.h>
#include <pnc_msgs/poi.h>

using namespace std;

class TrajectoryPlannerStandalone
{
  bool forward_, stopped_, goalreached_;
  int waypointPassed_;
  double frequency_, delay_;
  ros::Publisher move_status_pub_, cmd_steer_pub_, global_plan_pub_;
  ros::Publisher slowZone_pub_, poi_pub_, intersections_pub_;
  ros::Subscriber origin_destination_pt_sub_, g_plan_repub_sub_, cmd_vel_sub_, route_planner_sub_, rrt_path_sub_;
  golfcar_purepursuit::PurePursuit* pp_;
  string global_frame_, robot_frame_;
  tf::TransformListener* tf_;
  nav_msgs::Path rrt_path_;
  StationPaths* sp_;
  pnc_msgs::poi poi_;
  double dist_to_goal_;
public:
  TrajectoryPlannerStandalone(){
    forward_ = true;
    stopped_ = false;
    goalreached_ = false;
    waypointPassed_ = -1;
    tf_ = new tf::TransformListener();
    ros::NodeHandle nh, priv_nh("~");
    move_status_pub_ = nh.advertise<pnc_msgs::move_status>("move_status",1);
    cmd_steer_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_steer", 1);
    global_plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
    slowZone_pub_ = nh.advertise<geometry_msgs::PoseArray>("slowZone", 1, true);
    intersections_pub_ = nh.advertise<sensor_msgs::PointCloud>("intersection_pts", 1, true);
    poi_pub_ = nh.advertise<pnc_msgs::poi>("poi", 1, true);
    g_plan_repub_sub_ = nh.subscribe("global_plan_repub", 1, &TrajectoryPlannerStandalone::repubCallback, this);
    origin_destination_pt_sub_ = nh.subscribe("move_base_simple/goal", 1, &TrajectoryPlannerStandalone::originDestinationCallback, this);
    route_planner_sub_ = nh.subscribe("route_plan", 1, &TrajectoryPlannerStandalone::routePlaneCallback, this);
    rrt_path_sub_ = nh.subscribe("rrts_path", 1, &TrajectoryPlannerStandalone::rrtPathCallback, this);
    cmd_vel_sub_ = nh.subscribe("cmd_vel", 1, &TrajectoryPlannerStandalone::cmdVelCallback, this);
    priv_nh.param("global_frame", global_frame_, string("/map")); 
    priv_nh.param("robot_frame", robot_frame_, string("/base_link"));
    priv_nh.param("frequency", frequency_, 100.0);
    priv_nh.param("max_pose_delay", delay_, 0.015);
    double min_lookahead, anchor_pt_dist, car_length;
    priv_nh.param("min_lookahead", min_lookahead, 3.0);
    priv_nh.param("anchor_pt_dist", anchor_pt_dist, 1.0);
    priv_nh.param("car_length", car_length, 1.632);
    pp_ = new golfcar_purepursuit::PurePursuit(global_frame_.c_str(), min_lookahead, anchor_pt_dist, car_length);
    
    //without a variable to hold the timer, the timer won't start!!!
    ros::Timer timer = nh.createTimer(ros::Duration(1.0/frequency_), &TrajectoryPlannerStandalone::controlLoop, this);
    dist_to_goal_ = 1e99;
    ros::spin();
  }
  
private:
  void cmdVelCallback(geometry_msgs::Twist twist){
    pp_->updateCommandedSpeed(fabs(twist.linear.x));
  }
  
  void repubCallback(nav_msgs::PathConstPtr p){
    ROS_INFO("New path received from rrt with size of: %d", (int)p->poses.size());
    rrt_path_ = *p;
  }
  
  void rrtPathCallback(nav_msgs::PathConstPtr p){
    //rrt_path_ above is for use with part of the v1 rrt
    pp_->setPath(*p);
    pp_-> initialized_ = false;
    pp_-> dist_to_final_point = 100;
    pp_-> path_n_ =0;
//     cout<<"New rrts path received, following this new path"<<endl;
  }
  void add_pts(vector<int>& pts, vector<pnc_msgs::sig_pt>* poi, int type)
  {
    pnc_msgs::sig_pt sig_pt;
    for(unsigned int i=0; i<pts.size(); i++)
    {
	sig_pt.points = pts[i];
	sig_pt.type = type;
	poi->push_back(sig_pt);
    }
  }
  
  void routePlaneCallback(nav_msgs::PathConstPtr p){
    // note: slow zones and signals not working with simple path
    pp_->setPath(*p);
    pp_-> initialized_ = false;
    pp_-> dist_to_final_point = 100;
    pp_-> path_n_ =0;
  }
  
  void originDestinationCallback(geometry_msgs::PoseStampedConstPtr origin_dest){
    ROS_INFO("In make plan");
    std::vector<geometry_msgs::PoseStamped> plan;
    if(rrt_path_.poses.size() == 0){
      //all in according to the pixels from global map
      ROS_INFO("Old Station Path used");
      sp_ = new StationPaths();
      int start_number = (int)origin_dest->pose.position.x;
      int end_number = (int)origin_dest->pose.position.y;
      StationPath station_path = sp_->getPath(sp_->knownStations()(start_number),
					    sp_->knownStations()(end_number));
      geometry_msgs::PoseArray slowZones;
      slowZones.header.frame_id = global_frame_;
      slowZones.header.stamp = ros::Time::now();

      for(unsigned int i=0;i<sp_->slowZones_.size();i++)
      {
	  geometry_msgs::Pose pose;
	  pose.orientation.w = 1.0;
	  pose.position.x = sp_->slowZones_[i].x_;
	  pose.position.y = sp_->slowZones_[i].y_;
	  pose.position.z = sp_->slowZones_[i].r_;
	  slowZones.poses.push_back(pose);
      }

      slowZone_pub_.publish(slowZones);

      //publish point of interest
      pnc_msgs::poi poi;
      add_pts(station_path.leftsig_pts_, &poi.sig_pts, 0);
      add_pts(station_path.rightsig_pts_, &poi.sig_pts, 1);
      add_pts(station_path.offsig_pts_, &poi.sig_pts, 2);

      //poi.rightsig_pts.insert(poi.rightsig_pts.begin(),station_path.rightsig_pts_.begin(),station_path.rightsig_pts_.end());
      //poi.offsig_pts.insert(poi.offsig_pts.begin(),station_path.offsig_pts_.begin(),station_path.offsig_pts_.end());// = station_path.leftsig_pts_;
      poi.int_pts.insert(poi.int_pts.begin(),station_path.ints_pts_.begin(),station_path.ints_pts_.end());// = station_path.leftsig_pts_;
      poi_ = poi;
      poi_pub_.publish(poi);

      vector<geometry_msgs::PoseStamped> final_targets;
      geometry_msgs::PoseStamped ps;

      nav_msgs::Path p;
      p.poses.resize(station_path.size());
      for(unsigned int i=0; i<station_path.size(); i++)
      {
	  p.poses[i].header.frame_id = global_frame_;
	  p.poses[i].header.stamp = ros::Time::now();
	  p.poses[i].pose.position.x = station_path[i].x_;
	  p.poses[i].pose.position.y = station_path[i].y_;
	  p.poses[i].pose.orientation.w = 1.0;
	  geometry_msgs::PoseStamped pl;
	  pl.header.frame_id = global_frame_;
	  pl.header.stamp = ros::Time::now();
	  pl.pose.position.x = station_path[i].x_;
	  pl.pose.position.y = station_path[i].y_;
	  pl.pose.orientation.w = 1.0;
	  plan.push_back(pl);
      }
      ROS_INFO("Plan with %d points sent.", (int)plan.size());
      p.header.stamp = ros::Time();
      p.header.frame_id = global_frame_;

      //p.poses.push_back(start);
      //p.poses.push_back(goal);
      global_plan_pub_.publish(p);
      //plan.push_back(targets);
      sensor_msgs::PointCloud int_pc;
      int_pc.header.frame_id = global_frame_;
      int_pc.header.stamp = ros::Time::now();
      
      for(size_t j=0; j<poi_.int_pts.size(); j++){
	geometry_msgs::Point32 p;
	p.x = plan[poi_.int_pts[j]].pose.position.x;
	p.y = plan[poi_.int_pts[j]].pose.position.y;
	int_pc.points.push_back(p);
      }
      intersections_pub_.publish(int_pc);
    }
    //send the rrt path to the local planner
    else {
      for(size_t i=0; i<rrt_path_.poses.size(); i++){
	geometry_msgs::PoseStamped ps_temp;
	ps_temp.header.frame_id = global_frame_;
	ps_temp.header.stamp = ros::Time::now();
	ps_temp.pose.position.x = rrt_path_.poses[i].pose.position.x;
	ps_temp.pose.position.y = rrt_path_.poses[i].pose.position.y;
	ps_temp.pose.orientation.w = 1.0;
	plan.push_back(ps_temp);
      }
     ROS_INFO("Plan with rrt %d points sent.",(int)plan.size());
    }
    
    pp_-> initialized_ = false;
    pp_-> dist_to_final_point = 100;
    pp_-> path_n_ =0;
    pp_-> setPoses(plan);
  }
  
  double getIntDist(geometry_msgs::Point* int_point){
	
    //the intersections points is strictly increasing, getting the distance is easier
    for( unsigned i=0; i<poi_.int_pts.size(); i++ )
    {
      if( poi_.int_pts[i] >= pp_->path_n_ )
      {
	double dist;
	*int_point = pp_->getPath().poses[poi_.int_pts[i]].pose.position;
	if( pp_->current_pos_to_point_dist_simple(poi_.int_pts[i], &dist) )
	{
	  ROS_DEBUG_STREAM("int_p "<<poi_.int_pts[i]<<" path_n "<< pp_->path_n_);
	  return dist;
	}
	else return -1;
      }
    }
    return -1;
  }
  
  void controlLoop(const ros::TimerEvent& event){
    
    geometry_msgs::Twist cmd_vel;
    tf::Stamped<tf::Pose> robot_pose;
    pnc_msgs::move_status move_status;
    move_status.path_exist = false;
    poi_.node_tracking_dist = 0.0;
    double steer_angle;
    geometry_msgs::PoseStamped robot_pose_msg;
    if(pp_->getPath().poses.size()>0 && getRobotPose(robot_pose)){
      tf::poseStampedTFToMsg(robot_pose, robot_pose_msg);
      pp_->vehicle_base_ = robot_pose_msg.pose;
      double dist_to_goal_temp;
      bool path_exist = pp_->steering_control(&steer_angle, &dist_to_goal_temp);
      geometry_msgs::Point int_point;
      move_status.dist_to_ints = getIntDist(&int_point);
      move_status.int_point = int_point;
      move_status.sig_type = -1;
      if(path_exist){
	dist_to_goal_ = dist_to_goal_temp;
	move_status.steer_angle = steer_angle;
	move_status.obstacles_dist = 123;
	move_status.path_exist = true;
	if( waypointPassed_ != pp_->path_n_ )
	{
	    ROS_INFO("Path %d/%d", pp_->path_n_, (int)pp_->getPath().poses.size()-1);
	    waypointPassed_ = pp_->path_n_;
	}
	poi_.node_tracking_dist = fmutil::distance(pp_->getPath().poses[pp_->path_n_].pose.position, pp_->getCollidedPt());
      }
    }
    else
    {
        move_status.path_exist = false;
        if( pp_->path_n_ < (int) pp_->getPath().poses.size()-1 )
        {
            //move_status.emergency = -1;
            move_status.steer_angle = steer_angle;
            ROS_WARN("Steering control at unknown state");
        }
        else
        {

            move_status.steer_angle = 0;
            if(!goalreached_)
            {
                ROS_INFO("No path found");
                goalreached_=true;
            }
        }


    }
    poi_.robot_pose = robot_pose_msg.pose;
    poi_.cur_node = pp_->path_n_;
    poi_pub_.publish(poi_);
    move_status.dist_to_goal = dist_to_goal_;
    move_status_pub_.publish(move_status);
    cmd_vel.angular.z = move_status.steer_angle;
    cmd_steer_pub_.publish(cmd_vel);
  }
  
    
  bool getRobotPose(tf::Stamped<tf::Pose> &robot_pose) {
    robot_pose.setIdentity();
    tf::Stamped<tf::Pose> i_pose;
    i_pose.setIdentity();
    i_pose.frame_id_ = robot_frame_;
    i_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

    try {
        tf_->transformPose(global_frame_, i_pose, robot_pose);
    }
    catch(tf::LookupException& ex) {
        ROS_ERROR("No Transform available Error: %s", ex.what());
        return false;
    }
    catch(tf::ConnectivityException& ex) {
        ROS_ERROR("Connectivity Error: %s", ex.what());
        return false;
    }
    catch(tf::ExtrapolationException& ex) {
        ROS_ERROR("Extrapolation Error: %s", ex.what());
        return false;
    }
    // check robot_pose timeout
    if (current_time.toSec() - robot_pose.stamp_.toSec() > delay_) {
        ROS_WARN("PurePursuit transform timeout. Current time: %.4f, pose(%s) stamp: %.4f, tolerance: %.4f",
                 current_time.toSec(), global_frame_.c_str(), robot_pose.stamp_.toSec(), delay_);
        return false;
    }
    return true;
 }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "purepursuit_controller");
  TrajectoryPlannerStandalone tps;
  return 0;
}
