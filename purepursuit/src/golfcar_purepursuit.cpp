#include <purepursuit/purepursuit.h>

#include <fmutil/fm_math.h>
#define DEBUG 0
namespace golfcar_purepursuit
{

PurePursuit::PurePursuit(string global_frameID, double min_look_ahead_dist, double forward_achor_pt_dist, double car_length, bool publish)
{
    global_frameID_ = global_frameID;
    Lfw_ = min_look_ahead_dist;
    min_lookahead_ = min_look_ahead_dist;
    // min_lookahead_ = 0.5;
    lfw_ = forward_achor_pt_dist;
    car_length_ = car_length;
    publish_ = publish;
    nextPathThres_ = 5;
    dist_to_final_point = 100;
    initialized_ = false;
    lookahead_ini_ =false;
    forward_reverse_ini_ = false;
    reverse_ = false;
    path_n_ = 0;
    nextPathCount_ = 0;
    ros::NodeHandle n;
    //added to make sure the new steering wasn't too far from current track point
    tracking_point_ = -1;
    if(publish){
      pp_vis_pub_ = n.advertise<geometry_msgs::PolygonStamped>("pp_vis", 1);
      button_reverse_pub_ = n.advertise<std_msgs::Bool>("button_state_reverse", 1);
    }
    smoothed_path_pub_ = n.advertise<nav_msgs::Path>("smoothed_path", 1, true);
}

nav_msgs::Path PurePursuit::getPath(){
  return path_;
}

geometry_msgs::Point PurePursuit::getCollidedPt(){
  return collided_pt_;
}

void PurePursuit::setPoses(vector<geometry_msgs::PoseStamped> poses){
  if(poses.size()>0){
    forward_reverse_ini_ = false;
    tracking_point_ = -1;
    nav_msgs::Path smoothed_path, smoothed_path_densified;
    smoothed_path.header.frame_id = global_frameID_;
    smoothed_path.header.stamp = ros::Time::now();
    
    smoothed_path.poses.push_back(poses[0]);
    smoothed_path.poses.push_back(poses[1]);
    for (size_t i=2; i<poses.size(); i++){
      geometry_msgs::Point p1, p2, p3;
      //assuming first point is sane
      int smoothed_size = smoothed_path.poses.size();
      p1 = smoothed_path.poses[smoothed_size-2].pose.position;
      p2 = smoothed_path.poses[smoothed_size-1].pose.position;
      p3 = poses[i].pose.position;
      double ang_p12 = atan2(p1.y-p2.y, p1.x-p2.x);
      double ang_p23 = atan2(p2.y-p3.y, p2.x-p3.x);
      if(fmutil::angDist(ang_p12, ang_p23) < M_PI/2)
	        smoothed_path.poses.push_back(poses[i]);
    }
    //point density is to ensure a point's controlability in a path can be quickly determined
    //two points that is longer than Lfw_ will be eliminated by function heading_lookahead,
    //casuing trouble where path not exist reported
    double point_density = min_lookahead_/2.0;
    smoothed_path_densified.header = smoothed_path.header;
    smoothed_path_densified.poses.push_back(smoothed_path.poses[0]);
    for(size_t i=1; i<smoothed_path.poses.size(); i++){
      geometry_msgs::Point p1, p2;
      p1 = smoothed_path.poses[i].pose.position;
      p2 = smoothed_path.poses[i-1].pose.position;
      double dist = fmutil::distance(p1.x, p1.y, p2.x, p2.y);
      geometry_msgs::PoseStamped pose = smoothed_path.poses[i];
      if(dist>point_density) {
	int additional_points = dist/point_density;
	cout<<"Adding additional "<<additional_points<<" points."<<endl;
	double t_inc = 1.0/additional_points;
	cout<<p2.x<<","<<p2.y<<" t_inc:"<<t_inc<<endl;
	for(double t=0.0; t<=1.0; t+=t_inc){
	  geometry_msgs::PoseStamped new_pose=pose;
	  new_pose.pose.position.x = p2.x + (p1.x-p2.x)*t;
	  new_pose.pose.position.y = p2.y + (p1.y-p2.y)*t;
	  smoothed_path_densified.poses.push_back(new_pose);
	  cout<<new_pose.pose.position.x<<","<<new_pose.pose.position.y<<endl;
	}
      }
      else
	smoothed_path_densified.poses.push_back(pose);
    }
    if(publish_)
      smoothed_path_pub_.publish(smoothed_path_densified);
    path_ = smoothed_path_densified;
  }
}
void PurePursuit::setPath(nav_msgs::Path path){
  setPoses(path.poses);
}
    
void PurePursuit::updateCommandedSpeed(double speed_now){
  // previous version
  //    speed_now = fabs(speed_now);
  //    double Lfw = 2.24*speed_now;
  //    if(Lfw < min_lookahead_)
  //      Lfw_ = min_lookahead_;
  //    else if(Lfw > 12.0)
  //      Lfw_ = 12.0;
  //    else
  //      Lfw_ = Lfw;

  speed_now = fabs(speed_now);
  double Lfw = 1.0*speed_now;
  // double Lfw = 0.4*speed_now;
  if(Lfw < min_lookahead_)
    Lfw_ = min_lookahead_;
  else if(Lfw > 4.0)
    Lfw_ = 4.0;
  else
    Lfw_ = Lfw;
}

double PurePursuit::getMinLookahead(){
  return min_lookahead_;
}

bool PurePursuit::steering_control(double *wheel_angle, double *dist_to_goal)
{
  std_msgs::Bool reverse_button;
  reverse_button.data = reverse_;
  if(publish_)
    button_reverse_pub_.publish(reverse_button);
  //http://acl.mit.edu/papers/KuwataTCST09.pdf
    geometry_msgs::Point pt;
    if( ! initialized_ )
    {
        current_point_ = vehicle_base_.position;
        next_point_ = path_.poses[0].pose.position;
        initialized_ = true;
    }

    double heading_lh = 0;
    //disabling the forward reverse for now
    bool disable_f_r = false;
    if(disable_f_r){
      forward_reverse_ini_ = true;
      reverse_ = false;
    }
    if(!forward_reverse_ini_){
      bool forward_found=false, reverse_found=false;
      int forward_path_n, reverse_path_n;
      //check for forward orientation
      reverse_ = false;
      if(heading_lookahead(&heading_lh, dist_to_goal)){
	forward_found = true;
	forward_path_n = path_n_;
      }
      //now check for reverse orientation
      reverse_ = true;
      if(heading_lookahead(&heading_lh, dist_to_goal)){
	reverse_found = true;
	reverse_path_n = path_n_;
      }
      //make decision
      if(forward_found && !reverse_found){
	if(DEBUG) cout<<"Trivial solution, going forward"<<endl;
	reverse_ = false;
      }
      else if(!forward_found && reverse_found){
	if(DEBUG) cout<<"Trivial solution, going reverse"<<endl;
	reverse_ = true;
      }
      else if(forward_found && reverse_found){
	if(DEBUG) cout<<"Non-trivial solution, checking..."<<endl;
	//got to compare 2 different solution
	if(forward_path_n == path_.poses.size()-1) forward_path_n--;
	if(reverse_path_n == path_.poses.size()-1) reverse_path_n--;
	geometry_msgs::Point fp1 = path_.poses[forward_path_n].pose.position;
	geometry_msgs::Point fp2 = path_.poses[forward_path_n+1].pose.position;
	geometry_msgs::Point rp1 = path_.poses[reverse_path_n].pose.position;
	geometry_msgs::Point rp2 = path_.poses[reverse_path_n+1].pose.position;
	double forward_path_orientation = atan2(fp2.y-fp1.y, fp2.x-fp1.x);
	double reverse_path_orientation = atan2(rp1.y-rp2.y, rp1.x-rp2.x);
	double yaw =  tf::getYaw(vehicle_base_.orientation);
	double forward_orientation_diff = fmutil::angDist(yaw, forward_path_orientation);
	double reverse_orientation_diff = fmutil::angDist(yaw, reverse_path_orientation);
	if(forward_orientation_diff>reverse_orientation_diff) reverse_ = true;
	else reverse_ = false;
	if(DEBUG)
	  cout<<(reverse_?"Reverse":"Forward")<<", yaw: "<<yaw<<", diff_forward: "<<forward_orientation_diff<<", diff_reverse: "<<reverse_orientation_diff<<endl;
      }
      else if(!forward_found && !reverse_found){
	if(DEBUG) cout<<"No solution found, returning false"<<endl;
	return false;
      }
      forward_reverse_ini_ = true;
    }
    if( heading_lookahead(&heading_lh, dist_to_goal) )
    {
      
        double a = atan((car_length_ * sin(heading_lh))/(Lfw_/2+lfw_*cos(heading_lh)));
        *wheel_angle = fmutil::symbound<double>(a, 0.65);
        ROS_DEBUG("pursuing point: current=(%lf,%lf), next=(%lf,%lf)",
                    current_point_.x, current_point_.y,
                    next_point_.x, next_point_.y);
        return true;
    }

    return false;
}


bool PurePursuit::heading_lookahead(double *heading_la, double *dist_to_goal)
{
    double vehicle_heading = tf::getYaw(vehicle_base_.orientation);
    geometry_msgs::Point anchor_pt;
    if(reverse_){
      anchor_pt.x = vehicle_base_.position.x - lfw_ * cos(vehicle_heading);
      anchor_pt.y = vehicle_base_.position.y - lfw_ * sin(vehicle_heading);
    }
    else {
      anchor_pt.x = vehicle_base_.position.x + lfw_ * cos(vehicle_heading);
      anchor_pt.y = vehicle_base_.position.y + lfw_ * sin(vehicle_heading);
    }
      

    //search through all the path segments
    //reverse search to ensure that the pursuing point will always
    //be the one in front of the vehicle
    *dist_to_goal = 0;
    //if( !circle_line_collision(anchor_pt, &collided_pt_) || !lookahead_ini_)
    {
      lookahead_ini_ = true;
      vector<int> within_circle_pt_idices_;
      int tracked_point = -1;
      for( path_n_=(int)path_.poses.size()-2; path_n_>=0; path_n_-- )
      {
	//first check for path that is within the circle line
	//then select the path that has circle line collision with smallest angle diff
	//this is to extra evaluation to avoid two path close to each other having different orientation
	  geometry_msgs::Point current_point = path_.poses[path_n_].pose.position;
	  geometry_msgs::Point next_point = path_.poses[path_n_+1].pose.position;
	  current_point_ = current_point;
	  next_point_ = next_point;
	  double dist_1 = fmutil::distance(current_point.x, current_point.y, anchor_pt.x, anchor_pt.y);
	  double dist_2 = fmutil::distance(next_point.x, next_point.y, anchor_pt.x, anchor_pt.y);
	  if(DEBUG){
	    if(dist_1 < 3.0)
	      cout<<"dist_1:"<<dist_1<<" dist_2:"<<dist_2<<" dist1>dist2:"<<(dist_1>dist_2)<<" Lfw_:"<<Lfw_<<endl;
	  }
	  if( dist_1 < Lfw_ && dist_2 > Lfw_){
// 	    cout<<"**PASS**"<<dist_1<<" "<<dist_2<<endl;
	    if(tracking_point_ == -1)
	      within_circle_pt_idices_.push_back(path_n_);
	    else{
	      //make sure the tracking point wasn't too far from the curent point
	      double tracking_dist = 0.0;
	      //check path distance
	      int start_pt =tracking_point_;
	      int end_pt = path_n_;
	      if(tracking_point_ > path_n_){
		start_pt = path_n_;
		end_pt = tracking_point_;
	      }
	      for(int j=start_pt; j<end_pt; j++){
		geometry_msgs::Point p1 = path_.poses[j].pose.position;
		geometry_msgs::Point p2 = path_.poses[j+1].pose.position;
		tracking_dist += fmutil::distance(p1.x, p1.y, p2.x, p2.y);
	      }
	      if(DEBUG)
		cout<<"start_pt: "<<start_pt<<" end_pt: "<<end_pt<<" Tracking dist: "<<tracking_dist<<endl;
	      if(tracking_dist < 40.0) 
		within_circle_pt_idices_.push_back(path_n_);
	      
	    }
	  }
	      
      }
      double smallest_heading_diff = M_PI;
      geometry_msgs::Point selected_current_point;
      geometry_msgs::Point selected_next_point;
      if(DEBUG)
	cout<<"within_circle_pt_idices_.size(): "<<within_circle_pt_idices_.size()<<endl;
      geometry_msgs::Point last_pt = path_.poses[path_.poses.size()-1].pose.position;
      for(size_t i=0; i<within_circle_pt_idices_.size(); i++){
	geometry_msgs::Point pt1, pt2;
	if(reverse_){
	  pt1 = path_.poses[within_circle_pt_idices_[i]].pose.position;
	  pt2 = path_.poses[within_circle_pt_idices_[i]+1].pose.position;
	}
	else {
	  pt1 = path_.poses[within_circle_pt_idices_[i]+1].pose.position;
	  pt2 = path_.poses[within_circle_pt_idices_[i]].pose.position;
	}
	double proposed_path_heading = atan2(pt1.y-pt2.y, pt1.x-pt2.x);
	double heading_diff = fmutil::angDist(vehicle_heading, proposed_path_heading);
	
	current_point_ = pt2;
	next_point_ = pt1;
	//make sure when approaching station the vehicle won't continue looking for other points and turn around!
	if(heading_diff > M_PI/2 && fmutil::distance(anchor_pt.x, anchor_pt.y, last_pt.x, last_pt.y)<10.0)
	  continue;
	
	if(DEBUG)
	  cout<<i<<": idx "<<within_circle_pt_idices_[i]<<" proposed_path_heading: "<<proposed_path_heading/M_PI*180.0<<" vehicle_heading: "<<vehicle_heading/M_PI*180.0<<" heading_diff: "<<heading_diff/M_PI*180<<endl;
	
	if(heading_diff < smallest_heading_diff && circle_line_collision(anchor_pt, &collided_pt_)){
	  selected_current_point = current_point_;
	  selected_next_point = next_point_;
	  smallest_heading_diff = heading_diff;
	  path_n_ = within_circle_pt_idices_[i];
	  tracked_point = within_circle_pt_idices_[i];
	  
	}
      }
      tracking_point_ = tracked_point;
      current_point_ = selected_current_point;
      next_point_ = selected_next_point;
      if(DEBUG)
	cout<<"Selected idx: "<<path_n_<<endl;
    }
    //calculate distance to goal
    if( ! current_pos_to_point_dist(path_.poses.size()-1, dist_to_goal) )
    	return false;

    if(reverse_){
      *heading_la = -(atan2(anchor_pt.y-collided_pt_.y,
    		anchor_pt.x-collided_pt_.x) - vehicle_heading);
    }
    else {
      *heading_la = atan2(collided_pt_.y-anchor_pt.y,
		  collided_pt_.x-anchor_pt.x) - vehicle_heading;
    }
// 		cout<<"Result: "<<*heading_la<<endl;
//     cout<<"vehicle heading: "<<vehicle_heading<<" line heading: "<<line_heading<<" angDist: "<<fmutil::angDist(vehicle_heading, line_heading)<<endl;
    return true;
}

bool PurePursuit::current_pos_to_point_dist_simple(int end_point, double* path_dist)
{
	*path_dist=0;

	*path_dist = fmutil::distance(vehicle_base_.position,
				path_.poses[end_point].pose.position);

	if(path_n_<0)
		return false;
	else
		return true;

}

bool PurePursuit::current_pos_to_point_dist(int end_point, double* path_dist)
{
    *path_dist=0;
    if(path_n_<0)
    {
        *path_dist = fmutil::distance(vehicle_base_.position,
                                        path_.poses[end_point].pose.position);
        return false;
    }

    for( int i=path_n_+1; i<end_point; i++ )
    {
        *path_dist += fmutil::distance(path_.poses[i].pose.position,
                                        path_.poses[i+1].pose.position);
    }

    *path_dist +=
        fmutil::distance(collided_pt_, path_.poses[path_n_+1].pose.position)
        + fmutil::distance(collided_pt_, vehicle_base_.position);

    return true;
}

double PurePursuit::getVehicleLength(){
  return car_length_;
}

//add to search for the latest collision path as the colliding point to avoid the path
//behind being followed
bool PurePursuit::circle_line_collision(geometry_msgs::Point anchor_point,
                                        geometry_msgs::Point *intersect_point)
{
    //http://stackoverflow.com/questions/1073336/circle-line-collision-detection
    double Ex = current_point_.x;
    double Ey = current_point_.y;
    double Lx = next_point_.x;
    double Ly = next_point_.y;
    double Cx = anchor_point.x;
    double Cy = anchor_point.y;
    double r = Lfw_;
    double dx = Lx - Ex; double dy = Ly - Ey;
    double fx = Ex - Cx; double fy = Ey - Cy;

    float a = dx * dx + dy * dy;
    float b = 2 * (fx * dx + fy * dy);
    float c = (fx * fx + fy * fy) - (r * r);

    float discriminant = b*b-4*a*c;

    geometry_msgs::PolygonStamped polyStamped;
    polyStamped.header.frame_id = global_frameID_;
    polyStamped.header.stamp = ros::Time::now();
    ROS_DEBUG("cur:x=%lf y%lf, next:x=%lf y=%lf, anchor_point:x=%lf y=%lf r=%lf", Ex,Ey,Lx,Ly,Cx,Cy,r);
    if(discriminant < 0)
    {
        ROS_DEBUG("No intersection, cur:x=%lf y%lf, next:x=%lf y=%lf, anchor_point:x=%lf y=%lf", Ex,Ey,Lx,Ly,Cx,Cy);
        return false;
    }


    discriminant = sqrt(discriminant);
    float t1 = (-b - discriminant)/(2*a);
    float t2 = (-b + discriminant)/(2*a);

    geometry_msgs::Point32 p;
    
    //quick change to test t2
//     t1 = t2;
//     cout<<t1<<" "<<t2<<endl;
    if(t1 >=0 && t1 <=1)
    {
      //2 solutions available
      // the first solution is on the segment
      p.x = intersect_point->x = Ex+t1*dx;
      p.y = intersect_point->y = Ey+t1*dy;
      polyStamped.polygon.points.push_back(p);
      p.x = Cx; p.y = Cy;
      polyStamped.polygon.points.push_back(p);
      if(publish_)
	pp_vis_pub_.publish(polyStamped);
      ROS_DEBUG("Intersection found, cur:x=%lf y%lf, next:x=%lf y=%lf, anchor_point:x=%lf y=%lf", Ex,Ey,Lx,Ly,Cx,Cy);
      return true;
    }
    if(t2 >= 0 && t2 <=1){
      p.x = intersect_point->x = Ex+t2*dx;
      p.y = intersect_point->y = Ey+t2*dy;
      polyStamped.polygon.points.push_back(p);
      p.x = Cx; p.y = Cy;
      polyStamped.polygon.points.push_back(p);
      if(publish_)
	pp_vis_pub_.publish(polyStamped);
      return true;
    }
    // What about the second solution?

    ROS_DEBUG("No solution, cur:x=%lf y=%lf, next:x=%lf y=%lf", Ex, Ey, Lx, Ly);
    return false;
}

} //namespace golfcar_purepursuit
