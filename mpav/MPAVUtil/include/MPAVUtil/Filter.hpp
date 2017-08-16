/*
 * Filter.hpp
 *
 *  Created on: 3 Aug, 2014
 *      Author: liuwlz
 */

#ifndef FILTER_HPP_
#define FILTER_HPP_

namespace Util {

	class LowPassFilter{

	    double y;
	    double t;
	    double tau;
	    bool initialized;

	public:

	    LowPassFilter();
	    LowPassFilter(double tau);
	    virtual ~LowPassFilter();
	    double value() const;
	    void value(double x);
	    void reset();
	    double filter(double t, double x);
	    double filter_dt(double dt, double x);
	};
}  // namespace Util


#endif /* FILTER_HPP_ */
