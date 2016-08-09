#include <ros/ros.h>
#include <wiringPiSPI.h>
#include <iostream>

using namespace std;

unsigned char* reset_imu(int channel){
	int dataLen = 7;
	unsigned char data[dataLen] = {0x03, 0x00, 0x00, 0x00, 0x40, 0x00, 0xc1};
	wiringPiSPIDataRW(channel, data, dataLen);
	return data;
}

void config_imu(int channel){
	int dataLen = 7;
	//'03'opcode,'00''00''00'fillwords,'30'xbus-mid,'00'datalength,'D1'checksum
	//expected response: FA FF FF FF + '31'xbus-mid,'00'datalength,'Checksum'
	uint8_t data[dataLen] = {0x03, 0x00, 0x00, 0x00, 0x30, 0x00, 0xd1};
	wiringPiSPIDataRW(channel, data, dataLen);
}

void SetOutputConfiguration(int channel){
	int dataLen = 19;
	/*
	uint8_t data[dataLen] = {0x03, 0x00, 0x00, 0x00,
							 0xc0, 0x08, 0x40, 0x20,
							 0x00, 0x64, 0x80, 0x20,
							 0x00, 0x64, 0x71};
	*/
	
	uint8_t data[dataLen] = {0x03, 0x00, 0x00, 0x00,
							 0xc0, 0x0c, 0x20, 0x30,
							 0x00, 0x64, 0x40, 0x20,
							 0x00, 0x64, 0x80, 0x20,
							 0x00, 0x64, 0xb9};
	
	wiringPiSPIDataRW(channel, data, dataLen);
}



void measurement_imu(int channel){
	int dataLen = 7;
	//'03'opcode,'00''00''00'fillwords,'10'xbus-mid,'00'datalength,'F1'checksum
	//expected response: FA FF FF FF + '11'xbus-mid,'00'datalength,'Checksum'
	uint8_t data[dataLen] = {0x03, 0x00, 0x00, 0x00, 0x10, 0x00, 0xf1};
	wiringPiSPIDataRW(channel, data, dataLen);
}

float* readMeasurement_imu(int channel){
	int dataLen = 51;
	unsigned char data[dataLen] = {0x06, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
								0x00, 0x00, 0x00,  
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
								0x00, 0x00, 0x00,  
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
								};
	wiringPiSPIDataRW(channel, data, dataLen);
	/*
	for(int i=0; i<dataLen-1; i++){
		printf("%x, ", data[i]);
	}
	printf("%x \n", data[dataLen-1]);
	*/
	long roll_angle = ((long)data[9]<<24) | ((long)data[10]<<16) | ((long)data[11]<<8) | (long)data[12];
	long pitch_angle  = ((long)data[13]<<24) | ((long)data[14]<<16) | ((long)data[15]<<8) | (long)data[16];  
	long yaw_angle = ((long)data[17]<<24) | ((long)data[18]<<16) | ((long)data[19]<<8) | (long)data[20];
	//printf("[9]: %x, [10]: %x, [11]: %x, [12]: %x\n", data[9], data[10], data[11], data[12]);
	long acc_x = ((long)data[24]<<24) | ((long)data[25]<<16) | ((long)data[26]<<8) | (long)data[27];
	long acc_y = ((long)data[28]<<24) | ((long)data[29]<<16) | ((long)data[30]<<8) | (long)data[31];
	long acc_z = ((long)data[32]<<24) | ((long)data[33]<<16) | ((long)data[34]<<8) | (long)data[35];
	
	long w1 = ((long)data[39]<<24) | ((long)data[40]<<16) | ((long)data[41]<<8) | (long)data[42];
	long w2 = ((long)data[43]<<24) | ((long)data[44]<<16) | ((long)data[45]<<8) | (long)data[46];
	long w3 = ((long)data[47]<<24) | ((long)data[48]<<16) | ((long)data[49]<<8) | (long)data[50];
	
	float x =  *((float*)&roll_angle);
	float y =  *((float*)&pitch_angle);
	float z =  *((float*)&yaw_angle);
	float ax = *((float*)&acc_x);
	float ay = *((float*)&acc_y);
	float az = *((float*)&acc_z);
	float wx = *((float*)&w1);
	float wy = *((float*)&w2);
	float wz = *((float*)&w3);
	
	float result[9] = {x,y,z,ax,ay,az,wx,wy,wz};
	return result;
}

int readEncoder(int channel){
	int dataLen = 2;
	unsigned char data[dataLen] = {0xff, 0xff};
	wiringPiSPIDataRW(channel, data, dataLen);
	int EncReading = (((unsigned int) (data[0] & 0x7f)) << 6) + (((unsigned int) (data[1] & 0xf8)) >> 2);
	//printf("[0]: %x, [1]: %x \n", data[0], data[1]);
	return EncReading;
}

int main(int argc, char** argv){
	//Initialize ROS system
	ros::init(argc, argv, "imu");
	ros::NodeHandle nh;
	//Ros Rate
	int sampleRate=100;
	ros::Rate rate(sampleRate);
	//Setup wiringPi SPI for IMU 
	  //	SPI_MODE0 = 0,  // CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
	  //	SPI_MODE1 = 1,  // CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
	  //	SPI_MODE2 = 2,  // CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
	  //	SPI_MODE3 = 3,  // CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge
	int channel = 0; // Pi has 2 channels: 0 and 1
	int spi_freq = 1000000;
	int spi_mode = 3; // IMU needs mode3. // Encoder works for all the modes. (in mode 0, it reads different value.)
	if(wiringPiSPISetupMode(channel,spi_freq,spi_mode) == -1){
		cout << "wiringPi SPI setup error " << endl;
		return -1;
	}
	//Open channel 1 for encoder
	channel = 1;
	// spi_mode = 3;
	if(wiringPiSPISetupMode(channel,spi_freq,spi_mode) == -1){
		cout << "wiringPi SPI setup error " << endl;
		return -1;
	}
	//Setup IMU
	int channel_imu = 0;
	unsigned char* reset_result = reset_imu(channel_imu);
	config_imu(channel_imu);
	SetOutputConfiguration(channel_imu);
	measurement_imu(channel_imu);
	//Main
	float* result; 
	while(ros::ok()){
		//imu
		double currTime = ros::Time::now().toSec();
		
		result = readMeasurement_imu(channel_imu);
		printf("roll: %f pitch: %f yaw: %f \n", result[0], result[1], result[2]);
		printf("ax: %f ay: %f az: %f \n", result[3], result[4], result[5]);
		printf("w1: %f w2: %f w3: %f \n", result[6], result[7], result[8]);
		//encode
		int EncChannel = 1;
		int EncReading = readEncoder(EncChannel);
		cout <<"EncReading: "<< EncReading << endl;
		printf("===========OVER============\n");
		//Wait
		rate.sleep();
		double interval = ros::Time::now().toSec() - currTime;
		//printf("Time interval: %f", interval); 
		
	}
}
