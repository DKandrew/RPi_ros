#include <stdlib.h>
#include <math.h>

double* forward_kinematics(double q1, double q2);

double* inverse_kinematics(double x, double y, double z);

double acos_calc(double L1, double L2, double L3);

double length_calc(double L1, double L2, double angle);

double acos_calc(double L1, double L2, double L3);

double length_calc(double L1, double L2, double angle);

double* angle_temp_correction_RF(double knee, double hip, double shoulder);

double* angle_temp_correction_LF(double knee, double hip, double shoulder);

double* glide_stretch(double dihedral, double tension, double q5);

double** trot_RF(double* x, double* y, double* z, int resolution);

double** trot_LF(double* x, double* y, double* z, int resolution);

int* angle2PWM_IMU(double LF, double RF, double LH, double RH);

int* angle2PWM(double knee, double hip, double shoulder);


