#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <stdio.h>

double* forward_kinematics(double q1, double q2);

double* inverse_kinematics(double x, double y, double z);

double acos_calc(double L1, double L2, double L3);

double length_calc(double L1, double L2, double angle);

double acos_calc(double L1, double L2, double L3);

double length_calc(double L1, double L2, double angle);

double* angle_temp_correction_RF(double knee, double hip, double shoulder);

double* angle_temp_correction_LF(double knee, double hip, double shoulder);

double* glide(double dihedral, double tension, double q5);

double** gait(double x, double y, double z, double a, double b, double angle, int res, double ratio, int fore);

double** gait_ver2(double x, double y, double z, double a, double b, double angle, int res, double ratio, int fore);

double** fixedHeight(double x, double y, double z, double a, double b, double angle, int res, double ratio, int fore);

double** ellipse(double x, double z, double a, double b, double angle, int res, double, int fore);

double** halfEllipse(double x, double z, double a, double b, double angle, int res, double ratio, int fore);

double* linspace(double a, double b, int num);

void deepCopy(double* a, double* b, int res);

int* angle2PWM_IMU(double LF, double RF, double LH, double RH);

int* angle2PWM_LF(double knee, double hip, double shoulder);
int* angle2PWM_RF(double knee, double hip, double shoulder);
int* angle2PWM_LH(double knee, double hip, double shoulder);
int* angle2PWM_RH(double knee, double hip, double shoulder);
int pwmDS6100(double angle);
