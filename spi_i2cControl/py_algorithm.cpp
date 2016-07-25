#include "py_algorithm.h"

#define PI 3.14159265

// The length of each robot stick
double l1 = 14.31; 
double l2 = 2.84;
double l3 = 2.72;
double l4 = 11.4;
double l5 = 13.08+l3;
double l6 = 1.97;

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
	double ko2a = boa + theta1;
	double Xa = -length3*cos(ko2a);
	double Ya = length3*sin(ko2a);
	// Calculate (x,y) at point p
	double apg = abo - theta1;
	double ag = l5*sin(apg); double pg = l5*cos(apg);
	double Xp = Xa-pg; double Yp = Ya-ag;
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
	double temp1 = x+l5*cos(theta3)-l6; double temp2 = y+l5*sin(theta3);
	double temp3 = -(l1*l1-l2*l2-temp1*temp1-temp2*temp2)/(2*l2);
	double beta = asin(temp2/sqrt(temp1*temp1+temp2*temp2)); // The angle of the beta 
	
	double temp_angle_2 = asin(temp3/sqrt(temp1*temp1+temp2*temp2)); // The angle of the sum: theta + beta
	
	angle[1] = beta-temp_angle_2; 
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


