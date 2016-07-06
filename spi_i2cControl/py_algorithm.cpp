#include "py_algorithm.h"

#define PI 3.14159265
//using namespace std;

void inverse_kinematics(double* p, double *result){
	int a1=6;
	int a2=26;
	int a3=2;
	int L=30;
	double x=p[0];
	double y=p[1];
	double z=p[2];
	// theta1
	double theta1=atan2(y,x) + PI;
	double r=sqrt(x*x+y*y)-a1;
	double s=z;
	double q=sqrt(a3*a3+L*L);
	double D=-(a2*a2+q*q-r*r-s*s)/(2*a2*q);
	// theta3
	double theta3_temp=atan2(sqrt(1-D*D),D);
	double theta3=theta3_temp-atan2(L,a3);
	// theta2
	double theta2=PI/2+(atan2(r,abs(s))-atan2(q*sin(theta3_temp),a2+q*cos(theta3_temp)));
	//build result 
	result[0]=theta1;
	result[1]=theta2;
	result[2]=theta3;
}

double radiansToServoPosition(double theta){
    return (theta+1.3962634)*161.14437988+150;
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


