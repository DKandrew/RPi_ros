#include <iostream>
#include <wiringpi.h>
#include <siringserial.h>

int main(){
	int fd;
	int baud = 115200;	//baud rate
	if((fd = serialOpen("/dev/ttySO",baud) < 0 )){
		return 0;
	}
	//Send reset
	char* data = [0xFA, 0xFF, 0x64, 0x00, 0x9D]; 
	serialPuts(fd, data);
	data = [0xFA, 0xFF, 0x48, 0x00, 0xB9];
	serialPuts(fd, data);
	data = [0xFA, 0xFF, 0xC0, 0x04, 0x40, 0x20, 0x00, 0x64, 0x79];
	serialPuts(fd, data);
	data = [0xFA, 0xFF, 0x16, 0x00, 0xEB];
	serialPuts(fd, data);

	while(1){
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
			cout << "x: " << x << endl;
			cout << "y: " << y << endl;
			cout << "z: " << z << endl;
		}
	}
}