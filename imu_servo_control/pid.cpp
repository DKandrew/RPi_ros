#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;

/*///////////////////////////////////////////////////////////
 * This .cpp file contains functions required for PID control
/*///////////////////////////////////////////////////////////

//Constructor
pid::pid(double dt_, double max_, double min_, double Kp_, double Kd_, double Ki_)
{
	dt = dt_;
	max = max_;
	min = min_;
	Kp = Kp_;
	Kd = Kd_;
	Ki = Ki_;
	pre_error = 0;
	integral = 0;
}

//Destructor
pid::~pid(void)
{
	//Nothing
}

//This function calculates and returns a value to be added for control
double pid::calc(double setpoint, double pv)
{
	double error = setpoint - pv;
	double p = Kp*error;
	double i = Ki*integral;
	double derivative = (error - pre_error) / dt;
	double d = Kd*derivative;
	double rtn = p + i + d;

	if (rtn > max)
		rtn = max;
	if (rtn < min)
		rtn = min;
	pre_error = error;

	return rtn;
}
