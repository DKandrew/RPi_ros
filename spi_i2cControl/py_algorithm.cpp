#include "py_algorithm.h"

#define PI 3.14159265

// The length of each robot stick
double l1 = 14.31; 
double l2 = 2.84;
double l3 = 2.72;
double l4 = 11.4;
double l5 = 13.08+l3;
double l6 = 1.97;


double mult = 180/PI;
double L1 = 11.75;
double L2 = 2.9;
double L3 = 14.4;
double L4 = 2.65;
double L5 = 16.15;
double Lf = 1.65;
double offset = 15;
double qf = offset*(PI/180);


/*
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
*/

/*
// Inverse Kinematics algorithm. Receive input of x, y position of endeffector
// and output an array of two angles: theta1, theta2
double* inverse_kinematics(double x, double y){
	// Return value
	static double angle[2];
	// Calculate the first angle: alpha
	double pc = sqrt(x*x+y*y); // The distance from origin to p
	double bcp = acos((l4*l4+x*x+y*y-(l5-l3)*(l5-l3))/(2*pc*l4));
	if(y<0){
		double kcp = PI+atan2(y,x);
		angle[0] = bcp - kcp;
	}
	else{
		double kcp = PI-atan2(y,x);
		angle[0] = bcp + kcp;
	}
 	// Calculate theta3
	double pbc = acos(((l5-l3)*(l5-l3)+l4*l4-pc*pc)/(2*(l5-l3)*l4));
	double theta3 = PI - angle[0] - pbc;
	// Calculate the second angle: theta
	double a = x+l5*cos(theta3)-l6; double b = y+l5*sin(theta3); // a = cos(beta), b = sin(beta)
	double temp3 = -(l1*l1-l2*l2-a*a-b*b)/(2*l2); // The constant sum on the right side of equation
	double beta = asin(b/sqrt(a*a+b*b));
	if(a < 0){
		beta = PI - beta; // if a < 0, means cos(beta) < 0. THen beta should be PI - current beta
	}
	double angle_sum = asin(temp3/sqrt(a*a+b*b));
	double candidate1, candidate2; // Two candidates of theta2
	double finalValue = 0; // Final value of theta2
	candidate1 = angle_sum-beta; candidate2 = PI-angle_sum-beta; // 2 case: Either = (PI - angle_sum) - beta, or = angle_sum-beta
	// Pass into forward kinematic algorithm to find out which candidate is correct 
	/*
	double* result = forward_kinematics(angle[0], candidate1);
	if(result != NULL){
		bool x_diff = result[0]-x > -0.00001 && result[0]-x < 0.00001; bool y_diff = result[1]-y > -0.00001 && result[1]-y < 0.00001;
		if(x_diff && y_diff){
			finalValue = candidate1;
		}
	}
	result = forward_kinematics(angle[0], candidate2);
	if(result != NULL){
		bool x_diff = result[0]-x > -0.00001 && result[0]-x < 0.00001; bool y_diff = result[1]-y > -0.00001 && result[1]-y < 0.00001;
		if(x_diff && y_diff){
			finalValue = candidate2;
		}
	}
	*/
	/*
	finalValue = candidate2;
	angle[1] = finalValue;
	
	return angle;
}

*/


//Written by Won Dong======================================================================
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


double* inverse_kinematics(double x, double y){
	// Return value
	static double angle[2];
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
	double delta1 = acos_calc(P02P3, Lf, P0P3);
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
		
	alpha *= mult;	//
	beta *= mult;	//
	gamma *= mult;	//
	delta1 *= mult;	//
	delta2 *= mult;	//
	psi *= mult;	//
	psi0 *= mult;	//
	xi *= mult;		//
	
	//q1 *= mult;		//
	//q2 *= mult;		//
	q3 *= mult;		//
	q4 *= mult;		//
	q5 *= mult;		//
	
	/*
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
	
	q1 -= qf;	//Delete this line when Servo2 is placed horizontally 
	q2 -= qf;	//Delete this line when Servo1 is placed horizontally 
	angle[0] = q1;
	angle[1] = q2;
	
	return angle;
	
}


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

double* angle_temp_correction(double angle1, double angle2)
{
	static double rtn[2];
	rtn[0] = PI - angle1;
	rtn[1] = (PI/2) - angle2;
	return rtn;

}

//End of Written by Wondong===============================================






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
	angle[0] = 2.629*t1+665;  //1000-450
	if(angle[0] < min_theta1)
		angle[0] = min_theta1;
	else if (angle[0] > max_theta1)
		angle[0] = max_theta1;
	// Translate theta2
	int max_theta2 = 1000; // The maximum position theta1 can reach
	int min_theta2 = 320; // The minimum position theta1 can reach
	angle[1] = 2.629*t2+665;
	if(angle[1] < min_theta2)
		angle[1] = min_theta2;
	else if (angle[1] > max_theta2)
		angle[1] = max_theta2;
	// Return angle
	return angle;
}



/*
double* inverse_kinematics(double* p){
	double result[3];
	PyObject *pModule, *pDict, *pFunc;
    //PyObject *pFunc;
	PyObject *pArgs, *pValue;
	//PyRun_SimpleString("import sys; sys.path.insert(0, '/home/pi/Desktop/Draft_folder/i2c')");
	// Import module
	pModule = PyImport_Import(PyString_FromString("algorithm"));
	pDict = PyModule_GetDict(pModule);
	// Import func
	//pFunc = PyObject_GetAttrString(pModule, "inverse_kinematics");
	pFunc = PyDict_GetItemString(pDict, "inverse_kinematics");
	if(!PyCallable_Check(pFunc)) {
		cout << "Point 7" << endl;
	}
	// Build args
	cout << "Point 1" << endl;
	pArgs = PyList_New(3);
	PyList_SetItem(pArgs, 0, PyFloat_FromDouble(p[0]));
	PyList_SetItem(pArgs, 1, PyFloat_FromDouble(p[1]));
	PyList_SetItem(pArgs, 2, PyFloat_FromDouble(p[2]));
	// Call func
	cout << "Point 2" << endl;
	pValue = PyObject_CallObject(pFunc, pArgs);
	// Clean up
	cout << "Point 3" << endl;
	Py_DECREF(pModule);
	// Build result
	if(pValue == NULL){
		cout << "Point 6" << endl;
	}
	//cout << "Point 4" << endl;
	//Py_ssize_t len = PyTuple_GET_SIZE(pValue);
	//len = len - 3;
	cout << "Point 5" << endl;
	for(int i=0; i<3; i++){
		result[i] = PyFloat_AsDouble(PyList_GetItem(pValue, i));
		cout << "Successfully" << endl;
		//len++;
	}
	return result;
}
* 
* 
* double radiansToServoPosition(double theta){
    return (theta+1.3962634)*161.14437988+150;
//    double result;
//    PyObject *pFunc;
//    PyObject *pArgs, *pValue;
//    // Import func
//    pFunc = PyObject_GetAttrString(pModule, "radiansToServoPosition");
//    // Build args
//    pArgs = PyTuple_New(1);
//    PyTuple_SetItem(pArgs, 0, PyFloat_FromDouble(theta));
//    // Call func
//    pValue = PyObject_CallObject(pFunc, pArgs);
//    result = PyFloat_AsDouble(pValue);
//    return result;
//    
}

*/


