/*
 * Stanley_Steering_Control.hpp
 *
 *  Created on: 1 Aug, 2014
 *      Author: liuwlz
 */

#ifndef STANLEY_STEERING_CONTROL_HPP_
#define STANLEY_STEERING_CONTROL_HPP_

#include <Steering_Control/Steering_Control.hpp>
#include <math.h>

namespace Control {
	class StanleySteeringCtr :public SteeringControl{
	public:
		StanleySteeringCtr();
		virtual ~StanleySteeringCtr();
	private:

		int direction_flag_;
		double tracking_gain_, track_error_tolerance_;

		void getTrackingPoint();
		void calculateSteeringAngle();
	};

}  // namespace Control


#endif /* STANLEY_STEERING_CONTROL_HPP_ */
