#include <ros/ros.h>
#include <wiringPiSPI.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
using namespace std;

std::string Convert (float number){
		std::ostringstream buff;
		buff<<number;
		return buff.str();
}

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
	
	long roll_angle = ((long)data[9]<<24) | ((long)data[10]<<16) | ((long)data[11]<<8) | (long)data[12];
	long pitch_angle  = ((long)data[13]<<24) | ((long)data[14]<<16) | ((long)data[15]<<8) | (long)data[16];  
	long yaw_angle = ((long)data[17]<<24) | ((long)data[18]<<16) | ((long)data[19]<<8) | (long)data[20];
	
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
	return EncReading;
}

int main(int argc, char** argv){
	//Initialize ROS system
	ros::init(argc, argv, "imu");
	ros::NodeHandle nh;
	//Ros Rate
	int sampleRate=2;
	int readlength=500;
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
	//Setup IMU
	int channel_imu = 0;
	//reset_imu(channel_imu);
	config_imu(channel_imu);
	SetOutputConfiguration(channel_imu);
	measurement_imu(channel_imu);
	//Open channel 1 for encoder
	channel = 1;
	// spi_mode = 3;
	if(wiringPiSPISetupMode(channel,spi_freq,spi_mode) == -1){
		cout << "wiringPi SPI setup error " << endl;
		return -1;
	}
	//Main
	float* result; 
	
	//Following is a test of SPI 
	//config_imu(channel_imu);
	//SetOutputConfiguration(channel_imu);
	//measurement_imu(channel_imu);
	//while(ros::ok()){
		////reset_imu(channel_imu);
		////for(int i=0;i<10000;i++);
		////config_imu(channel_imu);
		////notification_pip(channel_imu);
		////config_imu(channel_imu);
		
		////for(int i=0;i<10000;i++);
		////SetOutputConfiguration(channel_imu);
		////measurement_imu(channel_imu);
		//readMeasurement_imu(channel_imu);
		//rate.sleep();
	//}

	// zero the encoder and synchonize
	int test1 = readEncoder(1);
	
	int i=0;
	float* test2;
	/* 
	while(ros::ok()&& (i < 10)){
	test2 = readMeasurement_imu(channel_imu);
	printf("test2 is: %f \n", test2[0]);
	i++;
	}
	*/
	std::string ENC[readlength];
	std::string Roll[readlength];
	std::string AngVel1[readlength];
	std::string Acc[readlength];
	std::string AngVel2[readlength];
	std::string AngVel3[readlength];
	
	i=0;
	while(ros::ok()&& (i < readlength)){  // When wirting data into file, add this into the while codition ->  	&& (i < readlength)	
		result = readMeasurement_imu(channel_imu);
		
		float x, az, wx, wy, wz;
		x = result[0]; az = result[5]; wx = result[6]; wy = result[7]; wz = result[8];
		Roll[i]=Convert(x);
		Acc[i]=Convert(az);
		AngVel1[i]=Convert(wx);
		AngVel2[i]=Convert(wy);
		AngVel3[i]=Convert(wz);
		
		//Debug: check IMU output data
		
		printf("roll: %f pitch: %f yaw: %f \n", x, result[1], result[2]);
		printf("ax: %f ay: %f az: %f \n", result[3], result[4], az);
		printf("w1: %f w2: %f w3: %f \n", wx, wy, wz);
		
		/*
		printf("roll: %f pitch: %f yaw: %f \n", result[0], result[1], result[2]);
		printf("ax: %f ay: %f az: %f \n", result[3], result[4], result[5]);
		printf("w1: %f w2: %f w3: %f \n", result[6], result[7], result[8]);
		*/
	    		
		//encode
		int EncChannel = 1;
		int EncReading = readEncoder(EncChannel);
		float EncAngle =((float)(EncReading - test1 )/8192)*(360);  
		cout <<"Encoder angle: "<< EncAngle << endl;
		ENC[i]=Convert(EncAngle);
		i++;
		
		//Wait
		rate.sleep();
	}
	/*
	FILE * pfile;
	pfile = fopen ("data.txt","wb");
	fwrite (ENC,sizeof(std::string),15,pfile);
	fclose(pfile);
	
	int points =0;
	float *test1=new float[15];
	pfile = fopen ("data.txt","rb");
	fread(test1,sizeof(std::string),15,pfile);
	fclose(pfile);
	*/
	//for(int i =0; i<readlength;i++){
	//	cout<<ENC[i]<<endl;
		//printf("%s, \n",test1[i]);
	//}
	
	//delete[] test1;
	
	/*
	 * Write data into txt files
	 */
	
	std::ofstream data1("Roll.txt");
	std::ofstream data2("Acc.txt");
	std::ofstream data3("AngVel1.txt");
	std::ofstream data4("AngVel2.txt");
	std::ofstream data5("AngVel3.txt");
	std::ofstream data6("ENC.txt");
	
	for(i=0;i<readlength;i++){
		data1 << Roll[i]<<endl;
		data2 << Acc[i]<<endl;
		data3 << AngVel1[i]<<endl;
		data4 << AngVel2[i]<<endl;
		data5 << AngVel3[i]<<endl;
		data6 << ENC[i]<<endl;
	}
	 
}
