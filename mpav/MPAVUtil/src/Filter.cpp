/*
 * Filter.cpp
 *
 *  Created on: 3 Aug, 2014
 *      Author: liuwlz
 */


#include <MPAVUtil/Filter.hpp>

namespace Util{

	LowPassFilter::
	LowPassFilter(){
		this->tau = 0;
		reset();
	}

	LowPassFilter::
	LowPassFilter(double tau){
		if( tau<=0 )
			throw ("Filter's time constant (tau) must be > 0");
		this->tau = tau;
		reset();
	}

	LowPassFilter::
	~LowPassFilter(){

	}

	double
	LowPassFilter::
	value() const{
		if( ! this->initialized )
			throw ("Trying to get the value from an uninitialized filter");
		return this->y;
	}


	void
	LowPassFilter::
	value(double x){
		this->initialized = true;
		this->t = 0;
		this->y = x;
	}


	void
	LowPassFilter::
	reset(){
		this->initialized = false;
	}


	double
	LowPassFilter::
	filter(double t, double x){
		if( this->tau==0 )
			throw ("Filter's time constant (tau) was not set");

		if( ! this->initialized ){
			this->initialized = true;
			this->y = x;
			this->t = t;
			return x;
		}

		double dt = t - this->t;
		this->t = t;

		if( dt > this->tau ){
			this->y = x;
			return x;
		}

		this->y += (x - this->y) * dt / this->tau;
		return this->y;
	}

	double
	LowPassFilter::
	filter_dt(double dt, double x){
		if( this->tau==0 )
			throw ("Filter's time constant (tau) was not set");
		if( ! this->initialized ){
     	  this->initialized = true;
     	  this->t = 0;
     	  this->y = x;
     	  return x;
		}
		return filter(this->t+dt, x);
	}
}

