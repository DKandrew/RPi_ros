#include <iostream>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <cstdio>
// Below are for UART
#include <unistd.h>
#include <fcntl.h>
#include <termios.h> 

using namespace std;

int writeData(int fd, char *data, int dataLen){
	int result = write(fd, data, dataLen);
	return result;
}

int main(){
	int baud = 115200;	//baud rate
	int fd = serialOpen("/dev/ttyS0",baud);
	if(( fd < 0 )){
		std::cout << "Open file failed" << std::endl;
		return 0;
	}
	//Send reset
	
	//char data_reset[5] = {0xFA, 0xFF, 0x40, 0x00, 0xC1}; 
	//writeData(fd, data_reset, 5);
    //char data_resetAKL[5] = {0xFA, 0xFF, 0x3F, 0x00, 0xC2};
	//writeData(fd, data_resetAKL, 5);
	char data_2config[5] = {0xFA, 0xFF, 0x30, 0x00, 0xD1};
	writeData(fd, data_2config, 5);
	//char data_setConfig[9] = {0xFA, 0xFF, 0xC0, 0x04, 0x80, 0x20, 0x00, 0x64, 0x39};
	char data_setConfig[17] = {0xFA, 0xFF, 0xC0, 0x0C, 0x20, 0x30, 0x00, 0x64, 0x40, 0x20, 0x00, 0x64, 0x80, 0x20, 0x00, 0x64, 0xB9};
	writeData(fd, data_setConfig, 17);
	char data_2meas[5] = {0xFA, 0xFF, 0x10, 0x00, 0xF1};
	writeData(fd, data_2meas, 5);
	
	
	while(1){
		/*
		char tData[] = {0xFA, 0xFF, 0x00, 0x00, 0x01};
		writeData(fd, tData, 5);
		//writeData(fd, data_2config, 5);
		delay(10);
		*/
		/*
		int len = 10;
		char temp[len];
		
		if(serialDataAvail(fd)){
			int a = read(fd, (void*)temp, len);
			temp[a] = '\0';
			printf("%i bytes read: %s\n", a, temp);
		}
		*/
		
		//Flags for detecting FA, FF signal
		int fa=0; 
		char temp[1];
		read(fd, (void*)temp, 1);
		if(temp[0] == 0xFA){
			fa = 1;
		}
		/*
		read(fd, (void*)temp, 1);
		if(fa && temp[0] == 0xFF){
			ff = 1;
			cout << "Find" << endl;
		}
		*/
		//If we find a sequence of FA FF, we find the starting of signal. Start reading
		if(fa){ 
			int i = 0;
			int dataLen = 49;
			char data[dataLen];
			while(i<dataLen){

				read(fd, (void*)temp, 1);
				data[i] = temp[0];
				i++;
			}
			
			long roll_angle = ((long)data[6]<<24) | ((long)data[7]<<16) | ((long)data[8]<<8) | (long)data[9];
			long pitch_angle  = ((long)data[10]<<24) | ((long)data[11]<<16) | ((long)data[12]<<8) | (long)data[13];
			long yaw_angle = ((long)data[14]<<24) | ((long)data[15]<<16) | ((long)data[16]<<8) | (long)data[17];
			
			long acc_x = ((long)data[21]<<24) | ((long)data[22]<<16) | ((long)data[23]<<8) | (long)data[24];
			long acc_y = ((long)data[25]<<24) | ((long)data[26]<<16) | ((long)data[27]<<8) | (long)data[28];
			long acc_z = ((long)data[29]<<24) | ((long)data[30]<<16) | ((long)data[31]<<8) | (long)data[32];
	
			long w1 = ((long)data[36]<<24) | ((long)data[37]<<16) | ((long)data[38]<<8) | (long)data[39];
			long w2 = ((long)data[40]<<24) | ((long)data[41]<<16) | ((long)data[42]<<8) | (long)data[43];
			long w3 = ((long)data[44]<<24) | ((long)data[45]<<16) | ((long)data[46]<<8) | (long)data[47];
			
			float x =  *((float*)&roll_angle);
			float y =  *((float*)&pitch_angle);
			float z =  *((float*)&yaw_angle);
			
			float ax = *((float*)&acc_x);
			float ay = *((float*)&acc_y);
			float az = *((float*)&acc_z);
			
			float wx = *((float*)&w1);
			float wy = *((float*)&w2);
			float wz = *((float*)&w3);
		
			printf("x: %f, y: %f, z: %f | ax: %f, ay: %f, az: %f | wx: %f, wy: %f, wz: %f \n", x, y, z, ax, ay, az, wx, wy, wz); 
			
			
		}
		/*
		cout << "x: " << x "y: " << y << endl;
			cout << "z: " << z << endl;
			cout << "acc: " << x << endl;
			cout << "y: " << y << endl;
			cout << "z: " << z << endl;
			cout << "x: " << x << endl;
			cout << "y: " << y << endl;
			cout << "z: " << z << endl;
			* printf("\n", ax, ay, az);
			printf("wx: %f, wy: %f, wz: %f \n", wx, wy, wz);
		*/
		
		
		
		/*
		int dataAvail = serialDataAvail(fd);
		if(dataAvail > 0){
			int imuData[dataAvail];
			for(int i=0; i<dataAvail; i++){
				imuData[i] = serialGetchar(fd);
			}
			long roll_angle = ((long)imuData[9]<<24) | ((long)imuData[10]<<16) | ((long)imuData[11]<<8) | (long)imuData[12];
			long pitch_angle  = ((long)imuData[13]<<24) | ((long)imuData[14]<<16) | ((long)imuData[15]<<8) | (long)imuData[16];  
			long yaw_angle = ((long)imuData[17]<<24) | ((long)imuData[18]<<16) | ((long)imuData[19]<<8) | (long)imuData[20];
			
			float x =  *((float*)&roll_angle);
			float y =  *((float*)&pitch_angle);
			float z =  *((float*)&yaw_angle);
			cout<<"Data Len " << dataAvail << endl; 
			std::cout << "x: " << x << std::endl;
			std::cout << "y: " << y << std::endl;
			std::cout << "z: " << z << std::endl;
		}
		*/
		
		

		//int count = read(fd, (void*)buffer, bufferLen);
		
		/*
		int dataAvail = serialDataAvail(fd);
		int count = 0; 
		while(dataAvail){
			serialGetchar(fd);
			count++;
			dataAvail = serialDataAvail(fd);
			
		}
		*/
		
		
		
	}
	serialClose(fd);
}
