// Header file of pca9685 class
#include <python2.7/Python.h>
#include <string>
#include <iostream>

using namespace std;

class pca9685 {
	private:	
		PyObject *pModule, *pClass, *pInstance;
	public:
		// consturctor
		pca9685();
		// func
		void set_pwm_freq(int freq_hz);
		void set_pwm(int channel, int on, int off);
		void set_all_pwm(int on, int off);
};
