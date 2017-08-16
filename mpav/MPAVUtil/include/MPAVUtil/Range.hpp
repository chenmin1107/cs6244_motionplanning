/*
 * Range.hpp
 *
 *  Created on: Nov 23, 2013
 *      Author: liuwlz
 */

#ifndef RANGE_HPP_
#define RANGE_HPP_

#include <stdlib.h>
#include <math.h>
#include <assert.h>

using namespace std;

namespace Util{
	template <class T>
	class Range{
	public:
		Range();
		virtual ~Range();

		bool WithIn(T variableIn, T center, T range);
		T Saturate(T variableIn, T bound);
		double EucludeanDist(T x_1, T y_1, T x_2, T y_2);
	};
}


#endif /* RANGE_HPP_ */
