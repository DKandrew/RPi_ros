#include "lidar.h"

/*/////////////////////////////////////////////////////////////
 * This .cpp file contains functions required for LIDAR setting
/*/////////////////////////////////////////////////////////////

// Constructor
lidar::lidar(int addr) {
	_i2caddr = addr;
	fd = wiringPiI2CSetup(addr);
}

//This function reads and returns data from LIDAR 
int lidar::i2cRead() 
{
	int count = 0;
	int dist_H = 0;
	int dist_L = 0;
	int rtn;
	wiringPiI2CWriteReg8(fd, 0x00, 0x04);	
	
	//While receives data or count is greater than 100
	while( (wiringPiI2CReadReg8(fd, 0x01) & 1 != 0) && (count <= 100) )
	{
		++count;
		//if(count == 100)
		//	printf("Count: 100");
	}
	//Calculate distance
	dist_H = wiringPiI2CReadReg8(fd, 0x0f);
	dist_L = wiringPiI2CReadReg8(fd, 0x10);
	rtn = 256*dist_H + dist_L;

	return rtn;
}
