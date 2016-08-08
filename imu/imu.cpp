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

uint8_t* config_imu(int channel){
	int dataLen = 7;
	//'03'opcode,'00''00''00'fillwords,'30'xbus-mid,'00'datalength,'D1'checksum
	//expected response: FA FF FF FF + '31'xbus-mid,'00'datalength,'Checksum'
	uint8_t data[dataLen] = {0x03, 0x00, 0x00, 0x00, 0x30, 0x00, 0xd1};
	wiringPiSPIDataRW(channel, data, dataLen);
	return data;
}

uint8_t* SetOutputConfiguration(int channel){
	int dataLen = 11;
	uint8_t data[dataLen] = {0x03, 0x00, 0x00, 0x00,
							 0xc0, 0x04, 0x20, 0x30,
							 0x00, 0x64, 0x89};
	wiringPiSPIDataRW(channel, data, dataLen);
	return data;
}

uint8_t* measurement_imu(int channel){
	int dataLen = 7;
	//'03'opcode,'00''00''00'fillwords,'10'xbus-mid,'00'datalength,'F1'checksum
	//expected response: FA FF FF FF + '11'xbus-mid,'00'datalength,'Checksum'
	uint8_t data[dataLen] = {0x03, 0x00, 0x00, 0x00, 0x10, 0x00, 0xf1};
	wiringPiSPIDataRW(channel, data, dataLen);
	return data;
}

float* readMeasurement_imu(int channel){
	int dataLen = 21;
	unsigned char data[dataLen] = {0x06, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00,
								0x00, 0x00, 0x00, 0x00,
								0x00
								};
	wiringPiSPIDataRW(channel, data, dataLen);
	printf("[9]: %x, [10]: %x, [11]: %x, [12]: %x\n", data[9], data[10], data[11], data[12]);
	long roll_angle = ((long)data[9]<<24) | ((long)data[10]<<16) | ((long)data[11]<<8) | (long)data[12];
	long pitch_angle  = ((long)data[13]<<24) | ((long)data[14]<<16) | ((long)data[15]<<8) | (long)data[16];  
	long yaw_angle = ((long)data[17]<<24) | ((long)data[18]<<16) | ((long)data[19]<<8) | (long)data[20];
	float x =  *((float*)&roll_angle);
	float y =  *((float*)&pitch_angle);
	float z =  *((float*)&yaw_angle);
	printf("%x \n", roll_angle);
	printf("X: %f Y: %f Z: %f \n", x, y, z);
	static float result[3] = {x,y,z};
	return result;
}

int readEncoder(int channel){
	int dataLen = 2;
	unsigned char data[dataLen] = {0xff, 0xff};
	wiringPiSPIDataRW(channel, data, dataLen);
	int EncReading = (((unsigned int) (data[0] & 0x7f)) << 6) + (((unsigned int) (data[1] & 0xf8)) >> 2);
	printf("[0]: %x, [1]: %x \n", data[0], data[1]);
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
	spi_mode = 3;
	if(wiringPiSPISetupMode(channel,spi_freq,spi_mode) == -1){
		cout << "wiringPi SPI setup error " << endl;
		return -1;
	}
	
	//Setup IMU
	int channel_imu = 0;
	unsigned char* reset_result = reset_imu(channel_imu);
	uint8_t* config_result = config_imu(channel_imu);
	uint8_t* setoutconfig_result = SetOutputConfiguration(channel_imu);
	uint8_t* measure_result = measurement_imu(channel_imu);
	//Main
	float* result; 
	
	while(ros::ok()){
		//imu
		/*
		double currTime = ros::Time::now().toSec();
		result = readMeasurement_imu(channel_imu);
		double interval = ros::Time::now().toSec() - currTime;
		printf("Time interval: %f", interval); 
		*/
		//encode
		int EncChannel = 1;
		int EncReading = readEncoder(EncChannel);
		cout << EncReading << endl;
		/*
		unsigned char test = 0xc1;
		printf("%x \n", test);
		long test = 0xc0d3f735;
		float out = *((float*)&test);
		printf("Test: %f\n", out); 
		*/
		/*
		reset_imu(channel_imu);
		cout << "Reset: ";
		for(int i=0; i<7; i++){
			//cout << unsigned(reset_result[i]) << "," ;
			printf("%X, ", reset_result[i]);
		}
		cout << endl;
		/*
		result = readMeasurement_imu(channel_imu);
		cout << "Result: ";
		for(int i=0; i<3; i++){
			cout << result[i] << ",";
		}
		cout << endl;
		*/
		
		//Wait
		rate.sleep();
	}
}
