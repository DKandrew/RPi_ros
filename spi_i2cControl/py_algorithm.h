/* This file store all the python algorithm that will be used in the 
 * I2C communication between Raspberry Pi and PCA9685 
 */

#include <stdlib.h>
#include <math.h>

//double* forward_kinematics(double theta1, double theta2);

double* inverse_kinematics(double x, double y);

double radiansToServoPosition(double theta);

int* angle2PWM(double theta1, double theta2);

double acos_calc(double L1, double L2, double L3);

double length_calc(double L1, double L2, double angle);

//Added by Won Dong
double acos_calc(double L1, double L2, double L3);

double length_calc(double L1, double L2, double angle);

double* angle_temp_correction(double angle1, double angle2);

double* forward_kinematics(double q1, double q2);


