/*
 * Range.cpp
 *
 *  Created on: 2 Aug, 2014
 *      Author: liuwlz
 */


#include <MPAVUtil/Range.hpp>

namespace Util {
	template <class T>
	bool
	Range<T>::
	WithIn(T variableIn, T center, T range){
		assert(range > 0);
		return (variableIn < (center+range) && variableIn > (center-range));
	}

	template <class T>
	T
	Range<T>::
	Saturate(T variableIn, T bound){
		assert(bound >0);
		T variableOut, tempBound;
		tempBound = variableIn > 0 ? bound: -bound;
		variableOut = fabs((double)variableIn) < bound ? variableIn : tempBound;
		return variableOut;
	}

	template <class T>
	double
	Range<T>::
	EucludeanDist(T x_1, T y_1, T x_2, T y_2){
		return sqrt((x_2-x_1)*(x_2-x_1) + (y_2 - y_1)*(y_2 - y_1));
}


}  // namespace Util
