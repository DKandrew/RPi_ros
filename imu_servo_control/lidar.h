#include <string>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
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

class lidar 
{
	private:
		int _i2caddr;
		int fd;
	public:
		// consturctor
		lidar(int addr);
		// func
		void reset();
		int i2cRead();
};
