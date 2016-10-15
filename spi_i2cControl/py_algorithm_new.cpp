#include "py_algorithm.h"

#define PI 3.14159265

// The length of each robot stick
double l1 = 14.31; 
double l2 = 2.84;
double l3 = 2.72;
double l4 = 11.4;
double l5 = 13.08+l3;
double lf = 1.97;
double qf = (PI / 180) * 30;

// Forward Kinematics algorithm. Receive input of two angles 
// and output an array containing the x,y position of endeffector.
double* forward_kinematics(double theta1, double theta2){
	// Calculate length1 (distance from origin to d) 
	// and length2 (distance from b to d)
	double Yd = l2*cos(theta2); // Yd is the y value of point d.
	double Xd = l2*sin(theta2); // Xd is the x value of point d. 

	double length1 = sqrt(Yd*Yd + (Xd+l6)*(Xd+l6)); // length1 is the distance from origin to d. 

	double Xb = -l4*cos(theta1); // Xb is the x value of point b.
	double Yb = l4*sin(theta1); // Yb is the y value of point b.

	double length2_x = Xd+l6-Xb;
	double length2_y = Yd-Yb;
	double length2 = sqrt(length2_y*length2_y+length2_x*length2_x);
	// Calculate angle abo
	double temp = (length2*length2 + l4*l4 - length1*length1)/(2*length2*l4);
	double dbo = acos(temp);
	temp = (l3*l3 + length2*length2 - l1*l1)/(2*l3*length2);
	double abd = acos(temp);

	double abo = dbo+abd;
	// Calculate length3 (distance from c to a)
	double length3 = sqrt(l3*l3 + l4*l4 - 2*l3*l4*cos(abo));
	// Calculate boa
	temp = (length3*length3 + l4*l4 - l3*l3)/(2*length3*l4);
	double boa = acos(temp);
	// Calculate (x,y) at point a
	double koa = boa + theta1;
	double Xa = -length3*cos(koa);
	double Ya = length3*sin(koa);
	// Calculate (x,y) at point p
	double slope_ab = (Ya-Yb)/(Xa-Xb);
	double constant_line_ab = Yb - slope_ab*Xb;
	double slope_angle;
	if(slope_ab < 0) // The angle of slope_ab
		slope_angle = PI-atan(slope_ab);
	else
		slope_angle = atan(slope_ab);
	double Xp = Xa-l5*cos(slope_angle);
	double Yp = slope_ab*Xp + constant_line_ab;
	// Return result;
	static double p[2];
	p[0]=Xp; p[1]=Yp;
	return p;
}

// Inverse Kinematics algorithm. Receive input of x, y position of endeffector
// and output an array of two angles: theta1, theta2
double* inverse_kinematics(double x, double y){
	// Return value
	static double angle[2];
	// Calculate theta5
	double Lp = sqrt(x*x+y*y); // The distance from origin to p
	double q5 = acos_calc(l1, l5-l4, Lp);

	// Calculate theta1
	double alpha = PI - q5;
	double P0P3 = legnth_clac(l1, l4, alpha);
	double beta = acos_calc(l1, Lp, l5-l4);
	double gamma = atan(y/x);
	double q1 = PI + gamma - beta;

 	// Calculate theta2
	double psi = acos_calc(P0P3, l5, Lp);
	double xi = q1 - q5;
	double a = l5*cos(xi);
	double b = l5*sin(xi);
	double P3x = a + x;
	double P3y = b + y;
	double P02x = lf*cos(qf);
	double P02y = lf*sin(qf);
	double P02P3 = sqrt((P3x - P02x)*(P3x - P02x) + (P3y - P02y)*(P3y - P02y));
	double delta1 = acos_calc(P02P3, lf, P0P3);
	double delta2 = acos_calc(P02P3, l2, l3);
	double q2 = PI - ((delta1 - qf) + delta2);

	// Calculate theta4
	double P2x = P02x + l2*cos(q2);
	double P2y = P02y + l2*sin(q2);
	double P0P2 = sqrt(P2x*P2x + P2y*P2y);
	double psi0 = acos_calc(P0P3, l3, P0P2);
	double q4 = PI - (psi + psi0);

	//Calculate theta3
	double q3 = PI - acos_calculator(l3, l2, P02P3);

	angle[0] = q1;
	angle[1] = q2;
	
	return angle;
}

double radiansToServoPosition(double theta){
    return (theta+1.3962634)*161.14437988+150;
}

// Convert the angle theta1, theta2 to PWM signal
int* angle2PWM(double theta1, double theta2){
	static int angle[2];
	// Convert all angle from radian to degree
	double t1 = theta1/PI*180;
	double t2 = theta2/PI*180;
	// Translate theta1
	int max_theta1 = 1000; // The maximum position theta1 can reach
	int min_theta1 = 450; // The minimum position theta1 can reach
	angle[0] = 2.629*t1+665;
	if(angle[0] < min_theta1)
		angle[0] = min_theta1;
	else if (angle[0] > max_theta1)
		angle[0] = max_theta1;
	// Translate theta2
	int max_theta2 = 1000; // The maximum position theta2 can reach
	int min_theta2 = 320; // The minimum position theta2 can reach
	angle[1] = 2.629*t2+665;
	if(angle[1] < min_theta2)
		angle[1] = min_theta2;
	else if (angle[1] > max_theta2)
		angle[1] = max_theta2;
	// Return angle
	return angle;
}

double acos_calc(double L1, double L2, double L3)
{
	double rtn = acos(( L1*L1 + L2*L2 - L3*L3) / (2*L1*l2));
	return rtn;
}

double length_calc(double L1, double L2, double angle)
{
	double rtn = sqrt( L1*L1 + L2*L2 - 2*L1*L2*cos(angle) );
	return rtn;
}


