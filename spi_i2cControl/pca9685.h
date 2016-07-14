/* Header file of pca9685 class
 * pca9685 class is a c++ class controlling the pca9685 16-channel LED driver
 * Referred from pca9685 python library and a related c++ code
 */
#include <string>
#include <iostream>
#include "wiringPiI2C.h"
#include <math.h>
#include <unistd.h>		// Time delay library
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>	//i2c-tools library
#include <vector>

using namespace std;

// Registers/etc:
#define MODE1  			0x00
#define MODE2 			0x01
#define SUBADR1 		0x02
#define SUBADR2 		0x03
#define SUBADR3 		0x04
#define ALLCALLADR		0X05
#define LED0_ON_L 		0x06
#define LED0_ON_H 		0x07
#define LED0_OFF_L 		0x08
#define LED0_OFF_H 		0x09
#define ALL_LED_ON_L  	0xFA
#define ALL_LED_ON_H  	0xFB
#define ALL_LED_OFF_L 	0xFC
#define ALL_LED_OFF_H 	0xFD
#define PRESCALE 		0xFE
#define CLOCK_FREQ 		25000000.0	//25MHz default osc clock

class pca9685 {
	private:	
		int _i2caddr;
		int fd;
	public:
		// consturctor
		pca9685(int addr);
		// func
		void reset();
		void set_pwm_freq(int freq_hz);
		int i2cWrite(int fd, int devid, char* data, int data_len);
		void set_pwm(vector<int> & pos);
};
