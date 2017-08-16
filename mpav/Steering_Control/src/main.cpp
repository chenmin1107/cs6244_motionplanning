/*
 * main.cpp
 *
 *  Created on: 7 Apr, 2014
 *      Author: liuwlz
 */

#include <Steering_Control/Steering_Control.hpp>
#include <Steering_Control/PP_Steering_Control.hpp>
#include <Steering_Control/Stanley_Steering_Control.hpp>

#include <Steering_Control/StanleySteeringCtr.h>

int main(int argc, char** argv){
	ros::init( argc, argv, "steering_ctd");
	//Control::SteeringControl steer_ctr;
	Control::PPSteeringCtr steer_ctr;
	//MPAV::StanleySteeringCtr steer_ctr;
	ros::spin();
	return 1;
}
