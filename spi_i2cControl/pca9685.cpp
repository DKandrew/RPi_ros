// The cpp file for pca9685 class
#include "pca9685.h"

// Func: Reset PCA9685 mode to 00
void pca9685::reset(){
	wiringPiI2CWriteReg8(fd, MODE1, 0x20); //enable auto-increment
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
	
	int dataSize = 4;
	int servo_num = 1;		//servo_num
	int chan = 0; 			//Channel 
	char data[dataSize+1]; 	// dataSize + 1 because we need to include the starting register's address
	data[0] = LED0_ON_L;
	for(int i=0; i<servo_num; i++){
		data[4*i+1] = on & 0xFF;
		data[4*i+2] = on >> 8;
		data[4*i+3] = off & 0xFF;
		data[4*i+4] = off >> 8;
	}
	//i2cWrite(fd, _i2caddr, data);	
}

// Func: set_pwm for all servo
void pca9685::set_pwm(vector<int> & pos){
	
	int on = 0;
	int channel = pos.size(); 	//Channel is the number of servo we control.
	int dataSize = 4*channel;
	//cout << "channel: " << channel << endl;
	char data[dataSize+1]; 	// dataSize + 1 because we need to include the starting register's address
	data[0] = LED0_ON_L; 	// Start from LED0_ON_L address
	//cout << "dataSize: " << dataSize << endl;
	
	for(int i=0; i<channel; i++){
		int off = pos[i];
		data[4*i+1] = on & 0xFF;
		data[4*i+2] = on >> 8;
		data[4*i+3] = off & 0xFF;
		data[4*i+4] = off >> 8;
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
int pca9685::i2cWrite(int fd, int devid, char* data, int data_len){	
	struct i2c_rdwr_ioctl_data msgset;
	struct i2c_msg msg[1];
	
	msg[0].addr = devid;		// slave device address
	msg[0].flags = 0;			// 0 means write
	msg[0].len = data_len+1;  // len = data size + 1 because ioctl will add slave address (0x40) on it.
	msg[0].buf = data;
	
	msgset.msgs = msg;
	msgset.nmsgs = 1;
	
	int result = ioctl(fd, I2C_RDWR, &msgset);
	if(result < 0)
		cout << "Fail on executing i2cWrite" << endl;
	return result;
}
