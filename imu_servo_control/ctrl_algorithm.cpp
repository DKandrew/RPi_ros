#include "ctrl_algorithm.h"

#define PI 3.14159265

// The length of each robot stick
double l1 = 14.31; 
double l2 = 2.84;
double l3 = 2.72;
double l4 = 11.4;
double l5 = 13.08+l3;
double l6 = 1.97;

double mult = 180/PI;
double L1 = 9;
double L2 = 2.8;
double L3 = 8;
double L4 = 1.7;
double L5 = 15;
double Lf = 2;
double offset = 120;	//qf in degree
double qf = offset*(PI/180);

// func: forward_kinematics
double* forward_kinematics(double q1, double q2)
{
	//Get positions of P4, P02, and P2
	double P4x = L1*cos(q1);
	double P4y = L1*sin(q1);
	double P02x = Lf*cos(qf);
	double P02y = Lf*sin(qf);
	
	double P2x = P02x + L2*cos(q2);
	double P2y = P02y + L2*sin(q2);
	
	//Theta5
	double P0P2 = sqrt( P2x*P2x + P2y*P2y );
	double P2P4 = sqrt( (P4x - P2x)*(P4x - P2x) + (P4y - P2y)*(P4y - P2y) );
	double alpha = acos_calc(L1, P2P4, P0P2);
	double alpha0 = acos_calc(L4, P2P4, L3);
	double q5 = PI - (alpha + alpha0);
	
	//Theta4
	double beta0 = acos_calc(L3, P2P4, L4);
	double q4 = alpha0 + beta0;
	
	//Theta3
	double P0P3 = length_calc(L1, L4, alpha + alpha0);
	double gamma = acos_calc(L3, P0P3, P0P2);
	double gamma0 = PI - q4 - gamma;
	double gamma1 = q5 - gamma0;
	double gamma2 = q1 - gamma1;
	double P3x = P0P3*cos(gamma2);
	double P3y = P0P3*sin(gamma2);
	double P02P3 = sqrt( (P3x - P02x)*(P3x - P02x) + (P3y - P02y)*(P3y - P02y)  );
	double gamma3 = acos_calc(L3, L2, P02P3);
	double q3 = PI - gamma3;
	
	//Position
	double q4abs = q2 + q3 + q4;
	double p1x = L1*cos(q1);
	double p1y = L1*sin(q1);
	double p5x = p1x + (L5-L4)*cos(q4abs);
	double p5y = p1y + (L5-L4)*sin(q4abs);	
	
	//Return Results
	static double rtn[5];
	rtn[0]=p5x; 
	rtn[1]=p5y;
	rtn[2]=q3; 
	rtn[3]=q4;
	rtn[4]=q5; 

	return rtn;
}

double* inverse_kinematics(double x, double y, double z)
{
	// Return value
	static double angle[3];
	y = -sqrt(x*x + y*y);
	double qs = atan(x/y);
	if(qs < 0)
		qs = -qs;
	
	if(x < 0)
		qs = -qs;
	
	// Calculate theta5
	double Lp = sqrt(y*y+z*z); // The distance from origin to p
	double q5 = acos_calc(L1, L5-L4, Lp);

	// Calculate theta1
	double alpha = PI - q5;
	double P0P3 = length_calc(L1, L4, alpha);
	double beta = acos_calc(L1, Lp, L5-L4);
	double gamma = atan(z/y);
	double q1 = PI + gamma - beta;	

 	// Calculate theta2
	double psi = acos_calc(P0P3, L5, Lp);
	double xi = q1 - q5;
	double a = L5*cos(xi);
	double b = L5*sin(xi);
	double P3y = a + y;
	double P3z = b + z;
	double P02y = Lf*cos(qf);
	double P02z = Lf*sin(qf);
	double P02P3 = sqrt((P3y - P02y)*(P3y - P02y) + (P3z - P02z)*(P3z - P02z));
	double delta1 = acos_calc(P02P3, Lf, P0P3);
	double delta2 = acos_calc(P02P3, L2, L3);
	double q2 = PI - ((delta1 - qf) + delta2);	
		
	// Calculate theta4
	double P2y = P02y + L2*cos(q2);
	double P2z = P02z + L2*sin(q2);
	double P0P2 = sqrt(P2y*P2y + P2z*P2z);
	double psi0 = acos_calc(P0P3, L3, P0P2);
	double q4 = PI - (psi + psi0);

	//Calculate theta3
	double q3 = PI - acos_calc(L3, L2, P02P3);

	angle[0] = q1;
	angle[1] = q2;
	angle[2] = qs;
	
	return angle;	
}

double* glide_stretch(double dihedral, double tension, double q5)
{
	// Return value
	static double rtn[3];

	// Calculate theta5
	dihedral = dihedral/mult;
	q5 = q5/mult;
	double qs = tension/mult;
		
	double Lp = sqrt( (L5-L4)*(L5-L4) + L1*L1 - 2*(L5-L4)*L1*cos(q5) );
	double angle = -acos_calc((L5-L4), Lp, L1);
	double y = -Lp*cos(dihedral - angle);
	double z = Lp*sin(dihedral - angle);	

	// Calculate theta1
	double alpha = PI - q5;
	double P0P3 = length_calc(L1, L4, alpha);
	double beta = acos_calc(L1, Lp, L5-L4);
	double gamma = atan(z/y);
	double q1 = PI + gamma - beta;	

 	// Calculate theta2
	double psi = acos_calc(P0P3, L5, Lp);
	double xi = q1 - q5;
	double a = L5*cos(xi);
	double b = L5*sin(xi);
	double P3y = a + y;
	double P3z = b + z;
	double P02y = Lf*cos(qf);
	double P02z = Lf*sin(qf);
	double P02P3 = sqrt((P3y - P02y)*(P3y - P02y) + (P3z - P02z)*(P3z - P02z));
	double delta1 = acos_calc(P02P3, Lf, P0P3);
	double delta2 = acos_calc(P02P3, L2, L3);
	double q2 = PI - ((delta1 - qf) + delta2);	
		
	// Calculate theta4
	double P2y = P02y + L2*cos(q2);
	double P2z = P02z + L2*sin(q2);
	double P0P2 = sqrt(P2y*P2y + P2z*P2z);
	double psi0 = acos_calc(P0P3, L3, P0P2);
	double q4 = PI - (psi + psi0);

	//Calculate theta3
	double q3 = PI - acos_calc(L3, L2, P02P3);

	rtn[0] = q1;
	rtn[1] = q2;
	rtn[2] = qs;
	
	return rtn;	
	
}
/*
//2D Version
double* inverse_kinematics(double x, double y)
{
	// Return value
	static double angle[2];
	//static double angle[15];  //Debug Purpose
	
	// Calculate theta5
	double Lp = sqrt(x*x+y*y); // The distance from origin to p
	double q5 = acos_calc(L1, L5-L4, Lp);

	// Calculate theta1
	double alpha = PI - q5;
	double P0P3 = length_calc(L1, L4, alpha);
	double beta = acos_calc(L1, Lp, L5-L4);
	double gamma = atan(y/x);
	double q1 = PI + gamma - beta;	

 	// Calculate theta2
	double psi = acos_calc(P0P3, L5, Lp);
	double xi = q1 - q5;
	double a = L5*cos(xi);
	double b = L5*sin(xi);
	double P3x = a + x;
	double P3y = b + y;
	double P02x = Lf*cos(qf);
	double P02y = Lf*sin(qf);
	double P02P3 = sqrt((P3x - P02x)*(P3x - P02x) + (P3y - P02y)*(P3y - P02y));
	double delta1;// = acos_calc(P02P3, Lf, P0P3);
	if(b >= 0)
		delta1 = acos_calc(P02P3, Lf, P0P3);
	if(b < 0)
		delta1 = 2*PI - acos_calc(P02P3, Lf, P0P3);
	double delta2 = acos_calc(P02P3, L2, L3);
	double q2 = PI - ((delta1 - qf) + delta2);	
		
	// Calculate theta4
	double P2x = P02x + L2*cos(q2);
	double P2y = P02y + L2*sin(q2);
	double P0P2 = sqrt(P2x*P2x + P2y*P2y);
	double psi0 = acos_calc(P0P3, L3, P0P2);
	double q4 = PI - (psi + psi0);

	//Calculate theta3
	double q3 = PI - acos_calc(L3, L2, P02P3);

	//Convert Unit from rad to degree
	/*	
	alpha *= mult;	// Debug Purpose
	beta *= mult;	// Debug Purpose
	gamma *= mult;	// Debug Purpose
	delta1 *= mult;	// Debug Purpose
	delta2 *= mult;	// Debug Purpose
	psi *= mult;	// Debug Purpose
	psi0 *= mult;	// Debug Purpose
	xi *= mult;		// Debug Purpose
	
	q1 *= mult;		// Debug Purpose
	q2 *= mult;		// Debug Purpose
	q3 *= mult;		// Debug Purpose
	q4 *= mult;		// Debug Purpose
	q5 *= mult;		// Debug Purpose
	
	
	angle[0] = q5;
	angle[1] = alpha;
	angle[2] = beta;
	angle[3] = gamma;
	angle[4] = q1;
	angle[5] = psi;
	angle[6] = xi;
	angle[7] = a;
	angle[8] = b;
	angle[9] = delta1;
	angle[10] = delta2;
	angle[11] = q2;
	angle[12] = psi0;
	angle[13] = q4;
	angle[14] = q5;
	*/
	
	//q1 -= qf;	//Delete this line when Servo2 is placed horizontally 
	//q2 -= qf;	//Delete this line when Servo1 is placed horizontally
	/*
	angle[0] = q1;
	angle[1] = q2;
	
	return angle;
	
}
*/

double acos_calc(double len1, double len2, double len3)
{
	double rtn = acos(( len1*len1 + len2*len2 - len3*len3) / (2*len1*len2));
	return rtn;
}

double length_calc(double len1, double len2, double angle)
{
	double rtn = sqrt( len1*len1 + len2*len2 - 2*len1*len2*cos(angle) );
	return rtn;
}

double* angle_temp_correction_RF(double knee, double hip, double shoulder)
{
	static double rtn[3];	
	rtn[0] = (PI/2) - knee;
	rtn[1] = (PI/2) - hip;
	rtn[2] = shoulder;
	
	return rtn;
}

double* angle_temp_correction_LF(double knee, double hip, double shoulder)
{
	static double rtn[3];	
	rtn[0] = (PI/2) - knee;
	rtn[1] = (PI/2) - hip;
	rtn[2] = shoulder - PI;
	
	return rtn;
}

double** trot_RF(double* x, double* y, double* z, int resolution)
{	
	double x_range[5] = {x[0], 0, x[1], 0, x[0]};
	double y_range[5] = {y[0], y[0], y[0], y[0], y[0]};
	double z_avg = (z[0] + z[1])/2.0;
	double z_range[5] = {z_avg, z[1], z_avg, z[0], z_avg};	
			
	double incX = x[1]/(double)resolution;
	double incY = 0.0/(double)resolution;
	double incZ = ((z[1]-z[0])/2)/(double)resolution;
	
	double* xInput = new double[4*resolution];
	double* yInput = new double[4*resolution];
	double* zInput = new double[4*resolution];
	for(int jj = 0; jj < 4; ++jj)
	{
		for(int ii = 0; ii < resolution; ++ii)
		{
			if(jj == 0 || jj == 1)
				xInput[jj*resolution + ii] = x_range[jj] + incX*ii;
			else
				xInput[jj*resolution + ii] = x_range[jj] - incX*ii;
				
			if(jj == 0 || jj == 3)
				zInput[jj*resolution + ii] = z_range[jj] + incZ*ii;
			else
				zInput[jj*resolution + ii] = z_range[jj] - incZ*ii;
				
			yInput[jj*resolution + ii] = y_range[jj] + incY*ii;					
		}
	}
	
	double** rtn;
	rtn = new double*[3];
	rtn[0] = xInput;
	rtn[1] = yInput;
	rtn[2] = zInput;
	
	return rtn;		
}

double** trot_LF(double* x, double* y, double* z, int resolution)
{	
	double x_range[5] = {x[1], 0, x[0], 0, x[1]};
	double y_range[5] = {y[0], y[0], y[0], y[0], y[0]};
	double z_avg = (z[0] + z[1])/2.0;
	double z_range[5] = {z_avg, z[0], z_avg, z[1], z_avg};	
			
	double incX = x[1]/(double)resolution;
	double incY = 0.0/(double)resolution;
	double incZ = ((z[1]-z[0])/2)/(double)resolution;
	
	double* xInput = new double[4*resolution];
	double* yInput = new double[4*resolution];
	double* zInput = new double[4*resolution];
	for(int jj = 0; jj < 4; ++jj)
	{
		for(int ii = 0; ii < resolution; ++ii)
		{
			if(jj == 0 || jj == 1)
				xInput[jj*resolution + ii] = x_range[jj] - incX*ii;
			else
				xInput[jj*resolution + ii] = x_range[jj] + incX*ii;
				
			if(jj == 0 || jj == 3)
				zInput[jj*resolution + ii] = z_range[jj] - incZ*ii;
			else
				zInput[jj*resolution + ii] = z_range[jj] + incZ*ii;
				
			yInput[jj*resolution + ii] = y_range[jj] + incY*ii;					
		}
	}
	
	double** rtn;
	rtn = new double*[3];
	rtn[0] = xInput;
	rtn[1] = yInput;
	rtn[2] = zInput;
	
	return rtn;		
}

int* angle2PWM_IMU(double LF, double RF, double LH, double RH)
{
	static int angle[4];

	// LF
	int max_LF = 1200; // The maximum position theta1 can reach	
	int min_LF = 300; // The minimum position theta1 can reach
	angle[0] = 2.629*LF+700;
	if(angle[0] < min_LF)
		angle[0] = min_LF;
	else if (angle[0] > max_LF)
		angle[0] = max_LF;
		
	// RF
	int max_RF = 1000; // The maximum position theta2 can reach
	int min_RF = 300; // The minimum position theta2 can reach
	angle[1] = 2.629*RF+670;
	if(angle[1] < min_RF)
		angle[1] = min_RF;
	else if (angle[1] > max_RF)
		angle[1] = max_RF;
	
	// LH
	int max_LH = 1000; // The maximum position theta3 can reach
	int min_LH = 320; // The minimum position theta3 can reach
	angle[2] = 2.629*LH+680;
	if(angle[2] < min_LH)
		angle[2] = min_LH;
	else if (angle[2] > max_LH)
		angle[2] = max_LH;
		
	// RH
	int max_RH = 1000; // The maximum position theta3 can reach
	int min_RH = 320; // The minimum position theta3 can reach
	angle[3] = 2.629*RH+660;
	if(angle[3] < min_RH)
		angle[3] = min_RH;
	else if (angle[3] > max_RH)
		angle[3] = max_RH;
		
	// Return angle
	return angle;
}

// Convert the angle theta1, theta2 to PWM signal
int* angle2PWM(double knee, double hip, double shoulder)
{
	static int angle[3];
	// Convert all angle from radian to degree
	double t1 = knee/PI*180;
	double t2 = hip/PI*180;
	double t3 = shoulder/PI*180;
	
	// Translate Knee (q1)
	int max_knee = 1200; // The maximum position theta1 can reach
	//int min_theta1 = 450; // The minimum position theta1 can reach
	int min_knee = 300; // The minimum position theta1 can reach
	angle[0] = 2.629*t1+690;  //1000-450
	//angle[0] = 2.629*t1+700;  //For IMU
	if(angle[0] < min_knee)
		angle[0] = min_knee;
	else if (angle[0] > max_knee)
		angle[0] = max_knee;
		
	// Translate Hip (q2)
	int max_hip = 1000; // The maximum position theta2 can reach
	int min_hip = 300; // The minimum position theta2 can reach
	angle[1] = 2.629*t2+665;
	if(angle[1] < min_hip)
		angle[1] = min_hip;
	else if (angle[1] > max_hip)
		angle[1] = max_hip;
	
	// Translate Shoulder (qs)
	int max_shoulder = 1000; // The maximum position theta3 can reach
	int min_shoulder = 320; // The minimum position theta3 can reach
	angle[2] = 2.629*t3+720;
	//angle[2] = 2.629*t3+660; //For IMU
	if(angle[2] < min_shoulder)
		angle[2] = min_shoulder;
	else if (angle[2] > max_shoulder)
		angle[2] = max_shoulder;
		
	// Return angle
	return angle;
}
