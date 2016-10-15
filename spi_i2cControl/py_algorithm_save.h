/* This file store all the python algorithm that will be used in the 
 * I2C communication between Raspberry Pi and PCA9685 
 */

#include <stdlib.h>
#include <math.h>

double* forward_kinematics(double theta1, double theta2);

double* inverse_kinematics(double x, double y);

double radiansToServoPosition(double theta);

int* angle2PWM(double theta1, double theta2);
