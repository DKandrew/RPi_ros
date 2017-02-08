// The cpp file for pca9685 class
#include "pca9685.h"

//i2c_msg structure to avoid error
struct i2c_msg
{
	  __u16 addr;
	  __u16 flags;
	#define I2C_M_TEN		0x0010
	#define I2C_M_RD		0x0001
	#define I2C_M_NOSTART		0x4000
	#define I2C_M_REV_DIR_ADDR	0x2000
	#define I2C_M_IGNORE_NAK	0x1000
	#define I2C_M_NO_RD_ACK		0x0800
	#define I2C_M_RECV_LEN		0x0400
	  __u16 len;
	  char* buf;
};  

// Func: Reset PCA9685 mode to 00
void pca9685::reset() {
	wiringPiI2CWriteReg8(fd, MODE1, 0x20); //enable auto-increment
	wiringPiI2CWriteReg8(fd, MODE2, 0x04);
}

// Constructor
pca9685::pca9685(int addr) {
	_i2caddr = addr;
	fd = wiringPiI2CSetup(addr);
	reset();
}

// Func: set pwm freq
void pca9685::set_pwm_freq(int freq_hz) {
	double prescaleval = (CLOCK_FREQ / 4096 / freq_hz) - 1;
	int prescale = floor(prescaleval + 0.5);
	int oldmode = wiringPiI2CReadReg8(fd, MODE1);
	int newmode = (oldmode & 0x7F) | 0x10;
	wiringPiI2CWriteReg8(fd, MODE1, newmode);		// Go to sleep
	wiringPiI2CWriteReg8(fd, PRESCALE, prescale);	// Set the prescale 
	wiringPiI2CWriteReg8(fd, MODE1, oldmode);
	usleep(5000); 	//Delay 5ms
	wiringPiI2CWriteReg8(fd, MODE1, oldmode | 0x80);
}

// Func: set_pwm for all servo
void pca9685::set_pwm(vector<int> & pos) {

	int on = 0x0;
	int channel = pos.size(); 	//Channel is the number of servo we control.
	int dataSize = 4 * channel;
	char data[dataSize + 1]; 	// dataSize + 1 because we need to include the starting register's address
	data[0] = LED0_ON_L; 	// Start from LED0_ON_L address

	for (int i = 0; i<channel; i++) {
		int off = pos[i];
		data[4 * i + 1] = on & 0xFF;
		data[4 * i + 2] = on >> 8;
		data[4 * i + 3] = off & 0xFF;
		data[4 * i + 4] = off >> 8;
	}
	i2cWrite(fd, _i2caddr, data, sizeof(data));
}

/* Func: Customize I2C write with auto-increment property
* reg: the starting register address
* size: the size of data
* data: the data/command
*
* Return: -1 if fail
*/
int pca9685::i2cWrite(int fd, int devid, char* data, int data_len) {
	struct i2c_rdwr_ioctl_data msgset;
	struct i2c_msg msg[1];

	msg[0].addr = devid;		// slave device address
	msg[0].flags = 0;			// 0 means write
	msg[0].len = data_len;  	// len = data size
	msg[0].buf = data;
	msgset.msgs = msg;
	msgset.nmsgs = 1;

	int result = ioctl(fd, I2C_RDWR, &msgset);
	if (result < 0)
		//cout << "Fail on executing i2cWrite" << endl;
	return result;
}
