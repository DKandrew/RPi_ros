/* This file store all the python algorithm that will be used in the 
 * I2C communication between Raspberry Pi and PCA9685 
 */

// Import module
//PyObject *pModule = PyImport_Import(PyString_FromString("algorithm.py"));

#include <stdlib.h>
#include <math.h>

void inverse_kinematics(double* p, double* result);

double radiansToServoPosition(double theta);


