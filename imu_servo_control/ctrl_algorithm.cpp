#include "ctrl_algorithm.h"
#define PI 3.14159265

/*/////////////////////////////////////////////////////////////////////////
 * This .cpp file contains functions required for mathematical calculations
/*/////////////////////////////////////////////////////////////////////////


// The length of each robot link
double mult = 180/PI;
double L1 = 9;
double L2 = 2.8;
double L3 = 8;
double L4 = 1.7;
double L5 = 15;
double Lf = 2;
double offset = 120;	//qf in degree
double qf = offset*(PI/180);

//This function calculates forward kinematics with given q1 and q2
double* forward_kinematics(double q1, double q2)
{
	//Get positions of P4, P02, and P2
	double P4x = L1*cos(q1);
	double P4y = L1*sin(q1);
	double P02x = Lf*cos(qf);
	double P02y = Lf*sin(qf);

	double P2x = P02x + L2*cos(q2);
	double P2y = P02y + L2*sin(q2);

	//Calculate Theta5
	double P0P2 = sqrt(P2x*P2x + P2y*P2y);
	double P2P4 = sqrt((P4x - P2x)*(P4x - P2x) + (P4y - P2y)*(P4y - P2y));
	double alpha = acos_calc(L1, P2P4, P0P2);
	double alpha0 = acos_calc(L4, P2P4, L3);
	double q5 = PI - (alpha + alpha0);

	//Calculate Theta4
	double beta0 = acos_calc(L3, P2P4, L4);
	double q4 = alpha0 + beta0;

	//Calculate Theta3
	double P0P3 = length_calc(L1, L4, alpha + alpha0);
	double gamma = acos_calc(L3, P0P3, P0P2);
	double gamma0 = PI - q4 - gamma;
	double gamma1 = q5 - gamma0;
	double gamma2 = q1 - gamma1;
	double P3x = P0P3*cos(gamma2);
	double P3y = P0P3*sin(gamma2);
	double P02P3 = sqrt((P3x - P02x)*(P3x - P02x) + (P3y - P02y)*(P3y - P02y));
	double gamma3 = acos_calc(L3, L2, P02P3);
	double q3 = PI - gamma3;

	//Calculate Position
	double q4abs = q2 + q3 + q4;
	double p1x = L1*cos(q1);
	double p1y = L1*sin(q1);
	double p5x = p1x + (L5 - L4)*cos(q4abs);
	double p5y = p1y + (L5 - L4)*sin(q4abs);

	//Return Results
	static double rtn[5];
	rtn[0] = p5x;
	rtn[1] = p5y;
	rtn[2] = q3;
	rtn[3] = q4;
	rtn[4] = q5;

	return rtn;
}

//This function calculates motor angles with given x,y, & z values of foot
double* inverse_kinematics(double x, double y, double z)
{
	// Return value
	static double angle[3];
	
	//Calculate y in 2D
	y = -sqrt(x*x + y*y);
	
	//Calculate shoulder angle
	double qs = atan(x / y);
	if (qs < 0)
		qs = -qs;

	if (x < 0)
		qs = -qs;

	// Calculate Theta5
	double Lp = sqrt(y*y + z*z); // The distance from origin to p
	double q5 = acos_calc(L1, L5 - L4, Lp);

	// Calculate Theta1
	double alpha = PI - q5;
	double P0P3 = length_calc(L1, L4, alpha);
	double beta = acos_calc(L1, Lp, L5 - L4);
	double gamma = atan(z / y);
	double q1 = PI + gamma - beta;

	//Calculate Theta2
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

	//Calculate Theta4
	double P2y = P02y + L2*cos(q2);
	double P2z = P02z + L2*sin(q2);
	double P0P2 = sqrt(P2y*P2y + P2z*P2z);
	double psi0 = acos_calc(P0P3, L3, P0P2);
	double q4 = PI - (psi + psi0);

	//Calculate Theta3
	double q3 = PI - acos_calc(L3, L2, P02P3);

	//Return results
	angle[0] = q1;
	angle[1] = q2;
	angle[2] = qs;

	return angle;
}

//This function calculates motor angles with given dihedral, shoulder & q5 angles 
double* glide(double dihedral, double tension, double q5)
{
	//Return value
	static double rtn[3];

	//Calculate Theta5
	dihedral = dihedral / mult;
	q5 = q5 / mult;
	double qs = tension / mult;

	double Lp = sqrt((L5 - L4)*(L5 - L4) + L1*L1 - 2 * (L5 - L4)*L1*cos(q5));
	double angle = -acos_calc((L5 - L4), Lp, L1);
	double y = -Lp*cos(dihedral - angle);
	double z = Lp*sin(dihedral - angle);

	//Calculate Theta1
	double alpha = PI - q5;
	double P0P3 = length_calc(L1, L4, alpha);
	double beta = acos_calc(L1, Lp, L5 - L4);
	double gamma = atan(z / y);
	double q1 = PI + gamma - beta;

	//Calculate Theta2
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

	//Calculate Theta4
	double P2y = P02y + L2*cos(q2);
	double P2z = P02z + L2*sin(q2);
	double P0P2 = sqrt(P2y*P2y + P2z*P2z);
	double psi0 = acos_calc(P0P3, L3, P0P2);
	double q4 = PI - (psi + psi0);

	//Calculate Theta3
	double q3 = PI - acos_calc(L3, L2, P02P3);

	//Return results
	rtn[0] = q1;
	rtn[1] = q2;
	rtn[2] = qs;

	return rtn;
}

//This function calculates angle opposite to len3 with given three lengths of a triangle
double acos_calc(double len1, double len2, double len3)
{
	double rtn = acos((len1*len1 + len2*len2 - len3*len3) / (2 * len1*len2));
	return rtn;
}

//This function calculates length of a link opposite to angle using law of cosine
double length_calc(double len1, double len2, double angle)
{
	double rtn = sqrt(len1*len1 + len2*len2 - 2 * len1*len2*cos(angle));
	return rtn;
}

//This function converts joint angles of LF & RH in robot frame to motor frame
double* angle_temp_correction_LF(double knee, double hip, double shoulder)
{
	static double rtn[3];
	rtn[0] = knee - (PI / 2);
	rtn[1] = hip - (PI / 2);
	rtn[2] = -shoulder;

	return rtn;
}

//This function converts joint angles of RF & LH in robot frame to motor frame
double* angle_temp_correction_RF(double knee, double hip, double shoulder)
{
	static double rtn[3];
	rtn[0] = (PI / 2) - knee;
	rtn[1] = (PI / 2) - hip;
	rtn[2] = shoulder;

	return rtn;
}

//This function calculates foot trajectory using ellipse
double** gait(double x, double y, double z, double a, double b, double angle, int res, double ratio, int fore)
{	
	double** rtn = new double*[3];
	rtn[0] = new double[res];
	rtn[1] = new double[res];
	rtn[2] = new double[res];

	double** XZ = ellipse(x, z, a, b, angle, res, ratio, fore);
	double* Y = linspace(y, y, res);
	
	deepCopy(rtn[0], XZ[0], res);
	deepCopy(rtn[1], Y, res);
	deepCopy(rtn[2], XZ[1], res);

	return rtn;
}

//This function calculates foot trejectory using half ellipse
double** gait_ver2(double x, double y, double z, double a, double b, double angle, int res, double ratio, int fore)
{	
	double** rtn = new double*[3];
	rtn[0] = new double[res];
	rtn[1] = new double[res];
	rtn[2] = new double[res];

	double** XZ = halfEllipse(x, z, a, b, angle, res, ratio, fore);
	double* Y = linspace(y, y, res);

	deepCopy(rtn[0], XZ[0], res);
	deepCopy(rtn[1], Y, res);
	deepCopy(rtn[2], XZ[1], res);

	return rtn;
}

//This function calculates foot trajectory for stationary mode
double** fixedHeight(double x, double y, double z, double a, double b, double angle, int res, double ratio, int fore)
{	
	double** rtn = new double*[3];
	rtn[0] = new double[res];
	rtn[1] = new double[res];
	rtn[2] = new double[res];

	double** XY = ellipse(x, y, a, -b, angle, res, ratio, fore);
	double* Z = linspace(z, z, res);
	
	deepCopy(rtn[0], XY[0], res);
	deepCopy(rtn[1], XY[1], res);
	deepCopy(rtn[2], Z, res);

	return rtn;
}

//This function calculates ellipse data points
double** ellipse(double x, double z, double a, double b, double angle, int res, double ratio, int fore)
{
	double** rtn = new double*[2];
	rtn[0] = new double[res];
	rtn[1] = new double[res];

	if(ratio > 1)
	{
		ratio = 1;
		printf("Ratio Cannot be larger than 100%\n");
	}
	
	if(ratio < 0)
	{
		ratio = 0;
		printf("Ratio Cannot be smaller than 0%\n");
	}
		
	int res_G = 2*(res*ratio); 
	int res_A = 2*((1 - ratio)*res + 1);
	while (res_G/2 + (res_A/2 - 1) != res)
		++res_A;

	double sinbeta = sin(-angle/mult);
	double cosbeta = cos(-angle/mult);

	double alpha_G[res_G];
	double* temp_G = linspace(0.0/mult, 360.0/mult, res_G);

	deepCopy(alpha_G, temp_G, res_G);
	double* sinalpha_G = new double[res_G/2];
	double* cosalpha_G = new double[res_G/2];

	double alpha_A[res_A];
	double* temp_A = linspace(0.0/mult, 360.0/mult, res_A);

	deepCopy(alpha_A, temp_A, res_A);
	double sinalpha_A[res_A/2 - 1];
	double cosalpha_A[res_A/2 - 1];

	for (int i = 0; i < res_G/2; ++i)
	{
		sinalpha_G[i] = sin(alpha_G[i]);
		cosalpha_G[i] = cos(alpha_G[i]);
		if(fore == 0)
			rtn[0][i] = x + (a*cosalpha_G[i]*cosbeta - b*sinalpha_G[i]*sinbeta);
		else
			rtn[0][i] = x + (-a*cosalpha_G[i]*cosbeta - b*sinalpha_G[i]*sinbeta);

		rtn[1][i] = z + (a*cosalpha_G[i]*sinbeta - b*sinalpha_G[i]*cosbeta);
	}

	for (int i = 0; i < res_A/2 - 1; ++i)
	{
		sinalpha_A[i] = sin(alpha_A[res_A/2 + i]);
		cosalpha_A[i] = cos(alpha_A[res_A/2 + i]);
		if(fore == 0)
			rtn[0][i + res_G/2] = x + (a*cosalpha_A[i]*cosbeta + b*sinalpha_A[i]*sinbeta);
		else
			rtn[0][i + res_G / 2] = x + (-a*cosalpha_A[i]*cosbeta - b*sinalpha_A[i]*sinbeta);

		rtn[1][i + res_G/2] = z + (a*cosalpha_A[i]*sinbeta - b*sinalpha_A[i]*cosbeta);
	}


	return rtn;
}

//This function calculates half ellipse data points
double** halfEllipse(double x, double z, double a, double b, double angle, int res, double ratio, int fore)
{
	double** rtn = new double*[2];
	rtn[0] = new double[res];
	rtn[1] = new double[res];

	if(ratio > 1)
	{
		ratio = 1;
		printf("Ratio Cannot be larger than 100%\n");
	}
	
	if(ratio < 0)
	{
		ratio = 0;
		printf("Ratio Cannot be smaller than 0%\n");
	}
		
	int res_G = 2*(res*ratio); 
	int res_A = 2*((1 - ratio)*res + 1);
	while (res_G/2 + (res_A/2 - 1) != res)
		++res_A;

	double sinbeta = sin(-angle/mult);
	double cosbeta = cos(-angle/mult);

	double alpha_G[res_G];
	double* temp_G = linspace(0.0/mult, 360.0/mult, res_G);

	deepCopy(alpha_G, temp_G, res_G);
	double* sinalpha_G = new double[res_G/2];
	double* cosalpha_G = new double[res_G/2];

	double alpha_A[res_A];
	double* temp_A = linspace(0.0/mult, 360.0/mult, res_A);

	deepCopy(alpha_A, temp_A, res_A);
	double sinalpha_A[res_A/2 - 1];
	double cosalpha_A[res_A/2 - 1];

	for (int i = 0; i < res_G/2; ++i)
	{
		sinalpha_G[i] = sin(alpha_G[i]);
		cosalpha_G[i] = cos(alpha_G[i]);
		if(fore == 0)
			rtn[0][i] = x + (a*cosalpha_G[i]*cosbeta - b*sinalpha_G[i]*sinbeta);
		else
			rtn[0][i] = x + (-a*cosalpha_G[i]*cosbeta - b*sinalpha_G[i]*sinbeta);
			
		rtn[1][i] = z;		
	}

	for (int i = 0; i < res_A/2 - 1; ++i)
	{
		sinalpha_A[i] = sin(alpha_A[res_A/2 + i]);
		cosalpha_A[i] = cos(alpha_A[res_A/2 + i]);
		if(fore == 0)
			rtn[0][i + res_G/2] = x + (a*cosalpha_A[i]*cosbeta + b*sinalpha_A[i]*sinbeta);
		else
			rtn[0][i + res_G / 2] = x + (-a*cosalpha_A[i]*cosbeta - b*sinalpha_A[i]*sinbeta);

		rtn[1][i + res_G/2] = z + (a*cosalpha_A[i]*sinbeta - b*sinalpha_A[i]*cosbeta);
	}

	return rtn;
}

//This function is identical to linspcae function in MATLAB
double* linspace(double a, double b, int num)
{
	double* rtn = new double[num];
	double interval = (b - a) / (double)(num - 1);
	for (int i = 0; i < num; ++i)
	{
		rtn[i] = a + i*interval;
	}

	return rtn;
}

//This function does deep copy for pointers and delete the pointer b after finishing copy 
void deepCopy(double* a, double* b, int res)
{
	for (int i = 0; i < res; ++i)
	{
		a[i] = b[i];
	}

	delete[] b;
	return;
}

//This function converts motor angles to PWM values for a simplified model using 4 motors
int* angle2PWM_IMU(double LF, double RF, double LH, double RH)
{
	static int angle[4];

	// LF
	int max_LF = 1200; // The maximum position theta1 can reach	
	int min_LF = 300; // The minimum position theta1 can reach
	angle[0] = 2.629*LF + 700;
	if (angle[0] < min_LF)
		angle[0] = min_LF;
	else if (angle[0] > max_LF)
		angle[0] = max_LF;

	// RF
	int max_RF = 1000; // The maximum position theta2 can reach
	int min_RF = 300; // The minimum position theta2 can reach
	angle[1] = 2.629*RF + 670;
	if (angle[1] < min_RF)
		angle[1] = min_RF;
	else if (angle[1] > max_RF)
		angle[1] = max_RF;

	// LH
	int max_LH = 1000; // The maximum position theta3 can reach
	int min_LH = 320; // The minimum position theta3 can reach
	angle[2] = 2.629*LH + 680;
	if (angle[2] < min_LH)
		angle[2] = min_LH;
	else if (angle[2] > max_LH)
		angle[2] = max_LH;

	// RH
	int max_RH = 1000; // The maximum position theta3 can reach
	int min_RH = 320; // The minimum position theta3 can reach
	angle[3] = 2.629*RH + 660;
	if (angle[3] < min_RH)
		angle[3] = min_RH;
	else if (angle[3] > max_RH)
		angle[3] = max_RH;

	// Return angle
	return angle;
}

//This function converts motor angle to PWM values for LF 
int* angle2PWM_LF(double knee, double hip, double shoulder)
{
	static int angle[3];
	// Convert all angle from radian to degree
	double t1 = knee / PI * 180;
	double t2 = hip / PI * 180;
	double t3 = shoulder / PI * 180;

	// Translate Knee (q1)
	int max_knee = 1200; // The maximum position theta1 can reach
						 //int min_theta1 = 450; // The minimum position theta1 can reach
	int min_knee = 300; // The minimum position theta1 can reach
	angle[0] = 2.629*t1 + 683;  //1000-450
								//angle[0] = 2.629*t1+700;  //For IMU
	if (angle[0] < min_knee)
		angle[0] = min_knee;
	else if (angle[0] > max_knee)
		angle[0] = max_knee;

	// Translate Hip (q2)
	int max_hip = 1200; // The maximum position theta2 can reach
	int min_hip = 300; // The minimum position theta2 can reach
	angle[1] = 2.629*t2 + 620;
	if (angle[1] < min_hip)
		angle[1] = min_hip;
	else if (angle[1] > max_hip)
		angle[1] = max_hip;

	// Translate Shoulder (qs)
	int max_shoulder = 1000; // The maximum position theta3 can reach
	int min_shoulder = 320; // The minimum position theta3 can reach
	angle[2] = 2.629*t3 + 713;
	//angle[2] = 2.629*t3+660; //For IMU
	if (angle[2] < min_shoulder)
		angle[2] = min_shoulder;
	else if (angle[2] > max_shoulder)
		angle[2] = max_shoulder;

	// Return angle
	return angle;
}

//This function converts motor angle to PWM values for RF
int* angle2PWM_RF(double knee, double hip, double shoulder)
{
	static int angle[3];
	// Convert all angle from radian to degree
	double t1 = knee / PI * 180;
	double t2 = hip / PI * 180;
	double t3 = shoulder / PI * 180;

	// Translate Knee (q1)
	int max_knee = 1200; // The maximum position theta1 can reach
						 //int min_theta1 = 450; // The minimum position theta1 can reach
	int min_knee = 300; // The minimum position theta1 can reach
	angle[0] = 2.629*t1 + 680;  //1000-450
								//angle[0] = 2.629*t1+700;  //For IMU
	if (angle[0] < min_knee)
		angle[0] = min_knee;
	else if (angle[0] > max_knee)
		angle[0] = max_knee;

	// Translate Hip (q2)
	int max_hip = 1200; // The maximum position theta2 can reach
	int min_hip = 300; // The minimum position theta2 can reach
	angle[1] = 2.629*t2 + 649;
	if (angle[1] < min_hip)
		angle[1] = min_hip;
	else if (angle[1] > max_hip)
		angle[1] = max_hip;

	// Translate Shoulder (qs)
	int max_shoulder = 1000; // The maximum position theta3 can reach
	int min_shoulder = 320; // The minimum position theta3 can reach
	angle[2] = 2.629*t3 + 697;
	//angle[2] = 2.629*t3+660; //For IMU
	if (angle[2] < min_shoulder)
		angle[2] = min_shoulder;
	else if (angle[2] > max_shoulder)
		angle[2] = max_shoulder;

	// Return angle
	return angle;
}

//This function converts motor angle to PWM values for LH
int* angle2PWM_LH(double knee, double hip, double shoulder)
{
	static int angle[3];
	// Convert all angle from radian to degree
	double t1 = knee / PI * 180;
	double t2 = hip / PI * 180;
	double t3 = shoulder / PI * 180;

	// Translate Knee (q1)
	int max_knee = 1200; // The maximum position theta1 can reach
						 //int min_theta1 = 450; // The minimum position theta1 can reach
	int min_knee = 300; // The minimum position theta1 can reach
	angle[0] = 2.629*t1 + 670;  //1000-450
								//angle[0] = 2.629*t1+700;  //For IMU
	if (angle[0] < min_knee)
		angle[0] = min_knee;
	else if (angle[0] > max_knee)
		angle[0] = max_knee;

	// Translate Hip (q2)
	int max_hip = 1200; // The maximum position theta2 can reach
	int min_hip = 300; // The minimum position theta2 can reach
	angle[1] = 2.629*t2 + 657;
	if (angle[1] < min_hip)
		angle[1] = min_hip;
	else if (angle[1] > max_hip)
		angle[1] = max_hip;

	// Translate Shoulder (qs)
	int max_shoulder = 1000; // The maximum position theta3 can reach
	int min_shoulder = 320; // The minimum position theta3 can reach
	angle[2] = 2.629*t3 + 690;
	//angle[2] = 2.629*t3+660; //For IMU
	if (angle[2] < min_shoulder)
		angle[2] = min_shoulder;
	else if (angle[2] > max_shoulder)
		angle[2] = max_shoulder;

	// Return angle
	return angle;
}

//This function converts motor angle to PWM values for RH
int* angle2PWM_RH(double knee, double hip, double shoulder)
{
	static int angle[3];
	// Convert all angle from radian to degree
	double t1 = knee / PI * 180;
	double t2 = hip / PI * 180;
	double t3 = shoulder / PI * 180;

	// Translate Knee (q1)
	int max_knee = 1200; // The maximum position theta1 can reach
						 //int min_theta1 = 450; // The minimum position theta1 can reach
	int min_knee = 300; // The minimum position theta1 can reach
	angle[0] = 2.629*t1 + 635;  //1000-450
								//angle[0] = 2.629*t1+700;  //For IMU
	if (angle[0] < min_knee)
		angle[0] = min_knee;
	else if (angle[0] > max_knee)
		angle[0] = max_knee;

	// Translate Hip (q2)
	int max_hip = 1200; // The maximum position theta2 can reach
	int min_hip = 300; // The minimum position theta2 can reach
	angle[1] = 2.629*t2 + 685;
	if (angle[1] < min_hip)
		angle[1] = min_hip;
	else if (angle[1] > max_hip)
		angle[1] = max_hip;

	// Translate Shoulder (qs)
	int max_shoulder = 1000; // The maximum position theta3 can reach
	int min_shoulder = 320; // The minimum position theta3 can reach
	angle[2] = 2.629*t3 + 695;
	//angle[2] = 2.629*t3+660; //For IMU
	if (angle[2] < min_shoulder)
		angle[2] = min_shoulder;
	else if (angle[2] > max_shoulder)
		angle[2] = max_shoulder;

	// Return angle
	return angle;
}

int pwmDS6100(double angle)
{
	int max = 2150;
	int min = 850;
	int mid = 1500;
	double a = 3.611;
	angle = mid + a*angle;
	
	return angle;
}
