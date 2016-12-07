#include <ros/ros.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

void notification_pip(int channel){
	int dataLen = 7;
	uint8_t data[dataLen] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	wiringPiSPIDataRW(channel, data, dataLen); 
}

void reset_imu(int channel){
	int dataLen = 7;
	unsigned char data[dataLen] = {0x03, 0x00, 0x00, 0x00, 0x40, 0x00, 0xc1};
	wiringPiSPIDataRW(channel, data, dataLen);
}

void config_imu(int channel){
	int dataLen = 7;
	//'03'opcode,'00''00''00'fillwords,'30'xbus-mid,'00'datalength,'D1'checksum
	//expected response: FA FF FF FF + '31'xbus-mid,'00'datalength,'Checksum'
	uint8_t data[dataLen] = {0x03, 0x00, 0x00, 0x00, 0x30, 0x00, 0xD1};
	wiringPiSPIDataRW(channel, data, dataLen);
}

void SetOutputConfiguration(int channel){
	//int dataLen = 11;
	//int dataLen = 15;
	int dataLen = 19;
	/*
	uint8_t data[dataLen] = {0x03, 0x00, 0x00, 0x00,
							 0xc0, 0x04, 0x20, 0x30,
							 0x00, 0x64, 0x89};
	*/
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
	uint8_t data[dataLen] = {0x03, 0x00, 0x00, 0x00, 0x10, 0x00, 0xF1};
	wiringPiSPIDataRW(channel, data, dataLen); 
}

float* readMeasurement_imu(int channel){
	int dataLen = 51;
	//int dataLen = 21;
	unsigned char data[dataLen] = {
								0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
								0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 
								};
	wiringPiSPIDataRW(channel, data, dataLen);
	// xyz angle
	long roll_angle = ((long)data[9]<<24) | ((long)data[10]<<16) | ((long)data[11]<<8) | (long)data[12];
	long pitch_angle  = ((long)data[13]<<24) | ((long)data[14]<<16) | ((long)data[15]<<8) | (long)data[16];  
	long yaw_angle = ((long)data[17]<<24) | ((long)data[18]<<16) | ((long)data[19]<<8) | (long)data[20];
	// Angular acceleration
	long acc_x = ((long)data[24]<<24) | ((long)data[25]<<16) | ((long)data[26]<<8) | (long)data[27];
	long acc_y = ((long)data[28]<<24) | ((long)data[29]<<16) | ((long)data[30]<<8) | (long)data[31];
	long acc_z = ((long)data[32]<<24) | ((long)data[33]<<16) | ((long)data[34]<<8) | (long)data[35];
	// Angular velocity
	long w1 = ((long)data[39]<<24) | ((long)data[40]<<16) | ((long)data[41]<<8) | (long)data[42];
	long w2 = ((long)data[43]<<24) | ((long)data[44]<<16) | ((long)data[45]<<8) | (long)data[46];
	long w3 = ((long)data[47]<<24) | ((long)data[48]<<16) | ((long)data[49]<<8) | (long)data[50];
	// Convert data into float
	float x =  *((float*)&roll_angle);
	float y =  *((float*)&pitch_angle);
	float z =  *((float*)&yaw_angle);
	float ax = *((float*)&acc_x);
	float ay = *((float*)&acc_y);
	float az = *((float*)&acc_z);
	float wx = *((float*)&w1);
	float wy = *((float*)&w2);
	float wz = *((float*)&w3);
	// Output result 
	float result[9] = {x,y,z,ax,ay,az,wx,wy,wz};
	return result;
}

int main(int argc, char** argv){
	//Initialize ROS system
	ros::init(argc, argv, "imu_node_isc");
	ros::NodeHandle nh;
	//Ros Rate
	int sampleRate=100;
	ros::Rate rate(sampleRate);
	//Setup wiringPi SPI for IMU 
	int channel = 0; // Pi has 2 channels: 0 and 1
	int spi_freq = 1000000;
	int spi_mode = 3; // IMU needs mode3. // Encoder works for all the modes except in mode 0, it reads different value.
	if(wiringPiSPISetupMode(channel,spi_freq,spi_mode) == -1){
		cout << "wiringPi SPI setup error " << endl;
		return -1;
	}
	//Setup IMU
	int delayTime = 10; 		// We need delay for each command in IMU, giving it enought time to response
	int channel_imu = 0;
	//reset_imu(channel_imu);
	delay(delayTime);
	config_imu(channel_imu);
	delay(delayTime);
	SetOutputConfiguration(channel_imu);
	delay(delayTime);
	measurement_imu(channel_imu);
	
	//Main
	float* result; 
	while(ros::ok()){
		//unsigned char data[10] = {0xff, 0x0f, 0xff, 0x0f, 0xff, 0x0f, 0xff, 0x0f, 0xff, 0x0f};
		//wiringPiSPIDataRW(channel_imu, data, 10);
		
		result = readMeasurement_imu(channel_imu);
		
		float x, y, z, az, wx, wy, wz;
		x = result[0]; 
		y = result[1]; 
		z = result[2]; 
		az = result[5]; wx = result[6]; wy = result[7]; wz = result[8];
		//Debug: check IMU output data. Comment out in official running
		printf("roll: %f pitch: %f yaw: %f \n", x, y, z);
		printf("ax: %f ay: %f az: %f \n", result[3], result[4], az);
		printf("w1: %f w2: %f w3: %f \n", wx, wy, wz);
		
		//Wait
		rate.sleep();
	}
}
