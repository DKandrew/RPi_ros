// A cpp class than utilize Python Class of pca9685
#include "pca9685_class.h"

pca9685::pca9685(){
	// import module
	pModule = PyImport_Import(PyString_FromString("Adafruit_PCA9685"));
	// Build the pca9685 python class
	pClass = PyObject_GetAttrString(pModule, "PCA9685");
	// Create an instance of the class
	if(PyCallable_Check(pClass)){
		pInstance = PyObject_CallObject(pClass, NULL);
	}
	// Debug
	//cout<< "successfully import the class" << endl;
}

void pca9685::set_pwm_freq(int freq_hz){
	//PyObject *pFunc;
	PyObject *pArgs, *pFunc;
	// call function set_pwm_freq in python
	pFunc = PyObject_GetAttrString(pInstance, "set_pwm_freq");
	if(pFunc == NULL){
		cout << "Failed" << endl;
		return ;
	}
	// build args
	pArgs = PyTuple_New(1);
	PyTuple_SetItem(pArgs, 0, PyInt_FromLong(freq_hz));
	// call set_pwm_freq python function
	PyObject *pValue = PyObject_CallObject(pFunc, pArgs);
	if(pValue == NULL) {
		cout << "Failed" << endl;
		return ;
	}
	// Debug 
	//cout << "Called set_pwm_freq successfully" << endl;
}

void pca9685::set_pwm(int channel, int on, int off){
	PyObject *pArgs, *pFunc;
	// call function set_pwm_freq in python
	pFunc = PyObject_GetAttrString(pInstance, "set_pwm");
	if(pFunc == NULL){
		cout << "Failed" << endl;
		return ;
	}
	// Build args
	pArgs = PyTuple_New(3);
	PyTuple_SetItem(pArgs, 0, PyInt_FromLong(channel));
	PyTuple_SetItem(pArgs, 1, PyInt_FromLong(on));
	PyTuple_SetItem(pArgs, 2, PyInt_FromLong(off));
	// call set_pwm python function
	PyObject *pValue = PyObject_CallObject(pFunc, pArgs);
	if(pValue == NULL) {
		cout << "Failed" << endl;
		return ;
	}
	// Debug
	//cout << "Called set_pwm successfully" << endl;
}

void pca9685::set_all_pwm(int on, int off){
	PyObject *pArgs, *pFunc;
	// Call set_all_pwm python func
	pFunc = PyObject_GetAttrString(pInstance, "set_all_pwm");
	if(pFunc == NULL){
		cout << "Failed" << endl;
		return ;
	}
	// Build args
	pArgs = PyTuple_New(2);
	PyTuple_SetItem(pArgs, 0, PyInt_FromLong(on));
	PyTuple_SetItem(pArgs, 1, PyInt_FromLong(off));
	// call set_all_pwm python function
	PyObject *pValue = PyObject_CallObject(pFunc, pArgs);
	if(pValue == NULL) {
		cout << "Failed" << endl;
		return ;
	}
	// Debug
	//cout << "Called set_all_pwm successfully" << endl;
}
