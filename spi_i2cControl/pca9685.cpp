// The cpp file for pca9685 class
#include "pca9685.h"

// Func: Reset PCA9685 mode to 00
void pca9685::reset(){
	wiringPiI2CWriteReg8(fd, MODE1, 0x00);
	wiringPiI2CWriteReg8(fd, MODE2, 0x04);
}

// Constructor
pca9685::pca9685(int addr){
	_i2caddr = addr;
	fd = wiringPiI2CSetup(addr);
	reset();
}

// Func: set pwm freq
void pca9685::set_pwm_freq(int freq_hz){
	double prescaleval = (CLOCK_FREQ/4096/freq_hz)-1;
  	int prescale = floor(prescaleval + 0.5);
  	int oldmode = wiringPiI2CReadReg8(fd, MODE1);
  	int newmode = (oldmode & 0x7F) | 0x10;
  	wiringPiI2CWriteReg8(fd, MODE1, newmode);		// Go to sleep
  	wiringPiI2CWriteReg8(fd, PRESCALE, prescale);	// Set the prescale 
  	wiringPiI2CWriteReg8(fd, MODE1, oldmode);
  	usleep(5000); 	//Delay 5ms
  	wiringPiI2CWriteReg8(fd, MODE1, oldmode | 0x80);
}

// Func: set pwm
void pca9685::set_pwm(int channel, int on, int off){
	wiringPiI2CWriteReg8(fd, LED0_ON_L+4*channel, on & 0xFF);
	wiringPiI2CWriteReg8(fd, LED0_ON_H+4*channel, on >> 8);
	wiringPiI2CWriteReg8(fd, LED0_OFF_L+4*channel, off & 0xFF);
	wiringPiI2CWriteReg8(fd, LED0_OFF_H+4*channel, off >> 8);
}
