#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;


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

pid::~pid(void)
{
	//Nothing
}	
		
double pid::calc( double setpoint, double pv )
{
	double error = setpoint - pv;
	//cout<<"================"<<endl;
	//cout<<"Error: "<<error<<endl;
	//cout<<"Prev Error: "<<pre_error<<endl;
	//cout<<"Kp: "<<Kp<<endl;
	//cout<<"Ki: "<<Kp<<endl;
	//cout<<"Kd: "<<Kp<<endl;
	double p = Kp*error;
	double i = Ki*integral;
	double derivative = (error - pre_error)/dt;
	//cout<<"dt: "<<dt<<endl;
	//cout<<"derivative: "<<derivative<<endl;
	double d = Kd*derivative;
	double rtn = p + i +d;
	//cout<<"P: "<<p<<endl;
	//cout<<"I: "<<i<<endl;
	//cout<<"D: "<<d<<endl;
	//cout<<"================"<<endl;
	
	if(rtn > max)
		rtn = max;
	if(rtn < min)
		rtn = min;
	pre_error = error;
	
	return rtn;
}
