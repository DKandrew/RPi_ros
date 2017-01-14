#ifndef _PID_H_
#define _PID_H_

#include <iostream>
#include <cmath>

class pid
{
public:
	pid(double dt, double max, double min, double Kp, double Kd, double Ki);
	~pid(void);
	double calc(double setpoint, double pv);

private:
	double dt;
	double max;
	double min;
	double Kp;
	double Kd;
	double Ki;
	double pre_error;
	double integral;

};
#endif